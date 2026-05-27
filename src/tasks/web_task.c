/**
 * @file web_task.c
 * @brief Web 服务与日志上传 — HTTP 静态文件 (SPIFFS) + WebSocket JSON 推送
 *
 * 初始化时注册 4 个 HTTP 处理器:
 *   GET /            → SPIFFS index.html
 *   GET /style.css   → SPIFFS style.css
 *   GET /script.js   → SPIFFS script.js
 *   GET /ws          → WebSocket 升级 → JSON 传感器日志推送
 *
 * 主循环阻塞等待 BIT_LOG_Q_READY, 读取 q_log + q_temp 后构建 JSON
 * 并通过 httpd 异步 API 推送到所有已连接的 WebSocket 客户端.
 */
#include "tasks/web_task.h"
#include "apf_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

#define WS_MAX_CLIENTS 4

static const char *TAG = "WEB_TASK ";

// HTTP/WebSocket 服务端共享状态
static httpd_handle_t ws_server = NULL;
static int            ws_fds[WS_MAX_CLIENTS];
static int            ws_count = 0;
static SemaphoreHandle_t ws_mutex = NULL;

// WebSocket 客户端集合 (互斥保护)
static void ws_add_client(int fd) {
    xSemaphoreTake(ws_mutex, portMAX_DELAY);
    if (ws_count < WS_MAX_CLIENTS) ws_fds[ws_count++] = fd;
    xSemaphoreGive(ws_mutex);
}

static void ws_remove_client(int fd) {
    xSemaphoreTake(ws_mutex, portMAX_DELAY);
    for (int i = 0; i < ws_count; i++) {
        if (ws_fds[i] == fd) { ws_fds[i] = ws_fds[--ws_count]; break; }
    }
    xSemaphoreGive(ws_mutex);
}

// WebSocket 广播 (httpd 异步帧发送, 避免裸 socket 冲突)
static void ws_send_all(const char *data, size_t len) {
    httpd_ws_frame_t pkt = {
        .type    = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)data,
        .len     = len,
    };
    xSemaphoreTake(ws_mutex, portMAX_DELAY);
    for (int i = 0; i < ws_count; i++)
        httpd_ws_send_frame_async(ws_server, ws_fds[i], &pkt);
    xSemaphoreGive(ws_mutex);
}

// MIME 类型推断
static const char *get_mime(const char *path) {
    if (strstr(path, ".html")) return "text/html";
    if (strstr(path, ".css"))  return "text/css";
    if (strstr(path, ".js"))   return "application/javascript";
    return "text/plain";
}

// HTTP GET 静态文件处理器 (从 SPIFFS 分块发送)
static esp_err_t file_get_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "HTTP GET %s", req->uri);

    // URI → SPIFFS 路径
    char filepath[640];
    if (strcmp(req->uri, "/") == 0)
        strcpy(filepath, "/spiffs/index.html");
    else {
        int n = snprintf(filepath, sizeof(filepath), "/spiffs%s", req->uri);
        if (n < 0 || (size_t)n >= sizeof(filepath)) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
    }

    FILE *f = fopen(filepath, "r");
    if (!f) {
        ESP_LOGW(TAG, "File not found: %s", filepath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Serving %s (%s)", filepath, get_mime(filepath));
    httpd_resp_set_type(req, get_mime(filepath));

    char buf[512];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
        httpd_resp_send_chunk(req, buf, n);
    httpd_resp_send_chunk(req, NULL, 0);   // 结束分块传输
    fclose(f);
    return ESP_OK;
}

// WebSocket 处理：阻塞等待客户端关闭或错误, 自动回复 PING → PONG 保持连接
static esp_err_t ws_handler(httpd_req_t *req) {
    int fd = httpd_req_to_sockfd(req);
    ws_add_client(fd);
    ESP_LOGI(TAG, "WS client connected, fd=%d, total=%d", fd, ws_count);

    uint8_t buf[256];
    httpd_ws_frame_t pkt = { .payload = buf };

    while (1) {
        esp_err_t ret = httpd_ws_recv_frame(req, &pkt, sizeof(buf));
        if (ret != ESP_OK) break;
        if (pkt.type == HTTPD_WS_TYPE_CLOSE) break;
        if (pkt.type == HTTPD_WS_TYPE_PING) {       // 浏览器 PING → 回复 PONG
            pkt.type = HTTPD_WS_TYPE_PONG;
            httpd_ws_send_frame(req, &pkt);
        }
    }

    ws_remove_client(fd);
    ESP_LOGI(TAG, "WS client disconnected, fd=%d, remaining=%d", fd, ws_count);
    return ESP_OK;
}

void web_task(void *pvParameters) {
    (void)pvParameters;
    ws_mutex = xSemaphoreCreateMutex();

    // 0. 启动 HTTP 服务器 (port 80, LRU 清理僵尸连接)
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port       = WEBSOCKET_PORT;
    cfg.lru_purge_enable  = true;
    cfg.recv_wait_timeout = 10;
    cfg.send_wait_timeout = 10;

    if (httpd_start(&ws_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed");
        vTaskDelete(NULL);
        return;
    }

    // 1. 注册静态文件 URI (非 WebSocket, 匹配普通 HTTP GET)
    httpd_uri_t file_uris[] = {
        {.uri = "/",          .method = HTTP_GET, .handler = file_get_handler},
        {.uri = "/style.css", .method = HTTP_GET, .handler = file_get_handler},
        {.uri = "/script.js", .method = HTTP_GET, .handler = file_get_handler},
    };
    for (int i = 0; i < sizeof(file_uris) / sizeof(file_uris[0]); i++)
        httpd_register_uri_handler(ws_server, &file_uris[i]);

    // 2. 注册 WebSocket URI (仅匹配 Upgrade: websocket 请求)
    httpd_uri_t ws_uri = {
        .uri          = "/ws",
        .method       = HTTP_GET,
        .handler      = ws_handler,
        .is_websocket = true,
    };
    httpd_register_uri_handler(ws_server, &ws_uri);

    ESP_LOGI(TAG, "HTTP + WebSocket server on port %d", WEBSOCKET_PORT);

    // 3. 验证 SPIFFS 文件就绪
    const char *check_files[] = {
        "/spiffs/index.html", "/spiffs/script.js", "/spiffs/style.css"
    };
    for (int i = 0; i < 3; i++) {
        FILE *f = fopen(check_files[i], "r");
        if (f) { fclose(f); ESP_LOGI(TAG, "SPIFFS: %s OK", check_files[i]); }
        else    ESP_LOGW(TAG, "SPIFFS: %s MISSING — run uploadfs", check_files[i]);
    }

    // 4. 数据推送主循环 — 每轮等待 APF 产出新一帧日志 */
    while (1) {
        xEventGroupWaitBits(eg_sync, BIT_LOG_Q_READY,
                            pdFALSE, pdFALSE, portMAX_DELAY);

        // 读取极坐标向量 (LiDAR + 火焰)
        vector_polar_t vectors[Q_POLAR_DEPTH];
        for (int i = 0; i < Q_POLAR_DEPTH; i++)
            xQueueReceive(q_log, &vectors[i], 0);

        // 温度可选 (非阻塞, 无新数据则为 NaN)
        float temp = NAN;
        xQueueReceive(q_temp, &temp, 0);

        xEventGroupClearBits(eg_sync, BIT_LOG_Q_READY | BIT_TEMP_Q_READY);
        int64_t ts_us = esp_timer_get_time();

        // 构建 JSON: { "ts":..., "temp":..., "vectors":[{a,d},...] }
        cJSON *root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "ts", (double)ts_us);
        cJSON_AddNumberToObject(root, "temp", (double)temp);

        cJSON *arr = cJSON_AddArrayToObject(root, "vectors");
        for (int i = 0; i < Q_POLAR_DEPTH; i++) {
            cJSON *v = cJSON_CreateObject();
            cJSON_AddNumberToObject(v, "a", (double)vectors[i].angle_deg);
            cJSON_AddNumberToObject(v, "d", (double)vectors[i].distance_mm);
            cJSON_AddItemToArray(arr, v);
        }

        char *json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            ws_send_all(json_str, strlen(json_str));
            ESP_LOGI(TAG, "Sent %d bytes to %d client(s)", (int)strlen(json_str), ws_count);
            free(json_str);
        }
        cJSON_Delete(root);
    }
}
