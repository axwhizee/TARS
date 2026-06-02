/**
 * @file web_task.c
 * @brief Web 服务 — 内存直出静态文件 + WebSocket JSON 推送
 *
 * 初始化流程:
 *   0. preload_static_files() — SPIFFS → PSRAM (一次性)
 *   1. httpd_start(port 80, max_open_sockets=16)
 *   2. 注册 3 个静态文件 URI + 1 个 WebSocket URI
 *   3. 主循环: BIT_LOG_Q_READY → JSON → 广播到全部 WS 客户端
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
// #include "cJSON.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>

#define WS_MAX_CLIENTS  4
#define HTTPD_MAX_FDS   16

static const char *TAG = "WEB_TASK ";

// HTTP/WebSocket 服务端共享状态
static httpd_handle_t ws_server = NULL;
static int            ws_fds[WS_MAX_CLIENTS];
static int            ws_count = 0;
static SemaphoreHandle_t ws_mutex = NULL;

// 预加载静态文件 (启动时从SPIFFS读取一次, 后续直接内存服务, 消灭fopen争用)
typedef struct {
    const char *uri;
    const char *mime;
    uint8_t    *data;
    size_t      len;
} static_file_t;

static static_file_t s_files[3];

static void preload_static_files(void) {
    const char *paths[] = {"/index.html", "/style.css", "/script.js"};
    const char *mimes[] = {"text/html", "text/css", "application/javascript"};
    for (int i = 0; i < 3; i++) {
        char sp[64];
        snprintf(sp, sizeof(sp), "/spiffs%s", paths[i]);
        FILE *f = fopen(sp, "r");
        if (!f) { ESP_LOGW(TAG, "Preload MISSING: %s", sp); continue; }
        fseek(f, 0, SEEK_END);
        long sz = ftell(f);
        fseek(f, 0, SEEK_SET);
        uint8_t *buf = malloc(sz);
        if (buf && fread(buf, 1, sz, f) == (size_t)sz) {
            s_files[i] = (static_file_t){paths[i], mimes[i], buf, (size_t)sz};
            ESP_LOGI(TAG, "Preloaded %s (%ld bytes)", paths[i], sz);
        }
        fclose(f);
    }
}

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

// WebSocket 广播 — 拷贝 fd 后发送, 不持锁避免阻塞 ws_handler
static void ws_send_all(const char *data, size_t len) {
    httpd_ws_frame_t pkt = {
        .type    = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)data,
        .len     = len,
    };
    int count;
    int fds[WS_MAX_CLIENTS];
    xSemaphoreTake(ws_mutex, portMAX_DELAY);
    count = ws_count;
    memcpy(fds, ws_fds, count * sizeof(int));
    xSemaphoreGive(ws_mutex);

    int live = 0;
    for (int i = 0; i < count; i++) {
        esp_err_t err = httpd_ws_send_frame_async(ws_server, fds[i], &pkt);
        if (err == ESP_OK)
            fds[live++] = fds[i];
        else
            ESP_LOGW(TAG, "WS send failed fd=%d err=%d, removing", fds[i], err);
    }

    xSemaphoreTake(ws_mutex, portMAX_DELAY);
    ws_count = live;
    memcpy(ws_fds, fds, live * sizeof(int));
    xSemaphoreGive(ws_mutex);
}

// HTTP GET 静态文件处理器 (内存直出, 无SPIFFS FILE*争用)
static esp_err_t file_get_handler(httpd_req_t *req) {
    const char *uri = req->uri;
    if (strcmp(uri, "/") == 0) uri = "/index.html";

    for (int i = 0; i < 3; i++) {
        if (s_files[i].data && strcmp(s_files[i].uri, uri) == 0) {
            httpd_resp_set_type(req, s_files[i].mime);
            httpd_resp_set_hdr(req, "Connection", "close");
            httpd_resp_send(req, (const char *)s_files[i].data, (ssize_t)s_files[i].len);
            return ESP_OK;
        }
    }
    ESP_LOGW(TAG, "404: %s", req->uri);
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

// WebSocket 处理：阻塞等待客户端关闭或错误, 自动回复 PING → PONG 保持连接
static esp_err_t ws_handler(httpd_req_t *req) {
    int fd = httpd_req_to_sockfd(req);
    ws_add_client(fd);
    ESP_LOGI(TAG, "WS client connected, fd=%d, total=%d", fd, ws_count);

    uint8_t buf[256];
    httpd_ws_frame_t pkt = { .payload = buf };
    int errs = 0;

    while (1) {
        esp_err_t ret = httpd_ws_recv_frame(req, &pkt, sizeof(buf));
        if (ret != ESP_OK) {
            // EAGAIN/EWOULDBLOCK 是接收端WS正常超时（客户端不发数据只收）
            // 不累计错误，避免误踢空闲客户端
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
            if (++errs > 32) break;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        errs = 0;
        if (pkt.type == HTTPD_WS_TYPE_CLOSE) break;
        if (pkt.type == HTTPD_WS_TYPE_PING) {
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

    // 0. 启动前预加载静态文件 (一次 SPIFFS 读取, 后续内存直出)
    preload_static_files();

    // 1. 启动 HTTP 服务器
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port        = WEBSOCKET_PORT;
    cfg.max_open_sockets   = HTTPD_MAX_FDS;
    cfg.lru_purge_enable   = true;
    cfg.recv_wait_timeout  = 2;
    cfg.send_wait_timeout  = 2;

    if (httpd_start(&ws_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed");
        vTaskDelete(NULL);
        return;
    }

    // 2. 注册静态文件 URI (非 WebSocket, 匹配普通 HTTP GET)
    httpd_uri_t file_uris[] = {
        {.uri = "/",          .method = HTTP_GET, .handler = file_get_handler},
        {.uri = "/style.css", .method = HTTP_GET, .handler = file_get_handler},
        {.uri = "/script.js", .method = HTTP_GET, .handler = file_get_handler},
    };
    for (int i = 0; i < sizeof(file_uris) / sizeof(file_uris[0]); i++)
        httpd_register_uri_handler(ws_server, &file_uris[i]);

    // 3. 注册 WebSocket URI (仅匹配 Upgrade: websocket 请求)
    httpd_uri_t ws_uri = {
        .uri          = "/ws",
        .method       = HTTP_GET,
        .handler      = ws_handler,
        .is_websocket = true,
    };
    httpd_register_uri_handler(ws_server, &ws_uri);

    ESP_LOGI(TAG, "HTTP + WebSocket server on port %d", WEBSOCKET_PORT);

    // 4. 数据推送主循环 — 每轮等待 APF 产出新一帧日志
    char json_buf[3600];
    while (1) {
        xEventGroupWaitBits(eg_sync, BIT_LOG_Q_READY, pdFALSE, pdFALSE, portMAX_DELAY);

        // 无客户端连接时仅消费队列, 跳过 JSON 构建
        int count;
        xSemaphoreTake(ws_mutex, portMAX_DELAY);
        count = ws_count;
        xSemaphoreGive(ws_mutex);
        if (count == 0) {
            vector_polar_t dummy;
            while (xQueueReceive(q_log, &dummy, 0) == pdTRUE) {}
            xEventGroupClearBits(eg_sync, BIT_LOG_Q_READY | BIT_TEMP_Q_READY);
            ESP_LOGI(TAG, "No client connected\n");
            continue;
        }

        vector_polar_t vectors[Q_POLAR_DEPTH];
        for (int i = 0; i < Q_POLAR_DEPTH; i++)
            xQueueReceive(q_log, &vectors[i], 0);
        float temp = NAN;
        vector_cart_t cart = g_cart_cmd;
        xQueueReceive(q_temp, &temp, 0);

        xEventGroupClearBits(eg_sync, BIT_LOG_Q_READY | BIT_TEMP_Q_READY);
        int64_t ts_us = esp_timer_get_time();

        // 手动构建 JSON (单缓冲区, 避免 cJSON 多次 malloc 导致堆碎片)
        int pos = snprintf(json_buf, sizeof(json_buf), "{\"ts\":%lld,\"temp\":", ts_us);
        if (isnan(temp))
            pos += snprintf(json_buf + pos, sizeof(json_buf) - pos, "null");
        else
            pos += snprintf(json_buf + pos, sizeof(json_buf) - pos, "%.2f", temp);
        pos += snprintf(json_buf + pos, sizeof(json_buf) - pos,
            ",\"cart\":{\"dx\":%.2f,\"dy\":%.2f},\"vectors\":[", cart.dx, cart.dy);

        for (int i = 0; i < Q_POLAR_DEPTH; i++) {
            if (pos >= (int)sizeof(json_buf) - 40) break;
            pos += snprintf(json_buf + pos, sizeof(json_buf) - pos,
                "{\"a\":%.2f,\"d\":%.2f}%s",
                vectors[i].angle, vectors[i].distance,
                (i < Q_POLAR_DEPTH - 1) ? "," : "");
        }

        pos += snprintf(json_buf + pos, sizeof(json_buf) - pos, "]}");
        ws_send_all(json_buf, pos);
        ESP_LOGI(TAG, "Sent %d bytes to %d client(s)\n", pos, ws_count);
    }
}
