/**
 * @file websocket_task.c
 * @brief WebSocket 服务端任务 — 接收 APF 转发数据 (+ 温度, 非阻塞) → JSON → WebSocket 推送
 */
#include "tasks/websocket_task.h"
#include "all_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "lwip/sockets.h"
#include <string.h>
#include <math.h>

#define WS_MAX_CLIENTS 4

static const char *TAG = "WS_TASK  ";

static httpd_handle_t ws_server = NULL;
static int            ws_fds[WS_MAX_CLIENTS];
static int            ws_count = 0;
static SemaphoreHandle_t ws_mutex = NULL;

/* ---------- 客户端连接管理 (HTTP server task 上下文) ---------- */

static void ws_add_client(int fd) {
    xSemaphoreTake(ws_mutex, portMAX_DELAY);
    if (ws_count < WS_MAX_CLIENTS) {
        ws_fds[ws_count++] = fd;
    }
    xSemaphoreGive(ws_mutex);
}

static void ws_remove_client(int fd) {
    xSemaphoreTake(ws_mutex, portMAX_DELAY);
    for (int i = 0; i < ws_count; i++) {
        if (ws_fds[i] == fd) {
            ws_fds[i] = ws_fds[--ws_count];
            break;
        }
    }
    xSemaphoreGive(ws_mutex);
}

/* ---------- WebSocket 帧封装 ---------- */

static void ws_send_all(const char *data, size_t len) {
    uint8_t header[10];
    size_t  header_len;

    header[0] = 0x81; /* FIN + Text opcode */

    if (len <= 125) {
        header[1] = (uint8_t)len;
        header_len = 2;
    } else if (len <= 65535) {
        header[1] = 126;
        header[2] = (uint8_t)((len >> 8) & 0xFF);
        header[3] = (uint8_t)(len & 0xFF);
        header_len = 4;
    } else {
        header[1] = 127;
        for (int i = 0; i < 8; i++) {
            header[2 + i] = (uint8_t)((len >> ((7 - i) * 8)) & 0xFF);
        }
        header_len = 10;
    }

    xSemaphoreTake(ws_mutex, portMAX_DELAY);
    for (int i = 0; i < ws_count; i++) {
        send(ws_fds[i], header, header_len, 0);
        send(ws_fds[i], data, len, 0);
    }
    xSemaphoreGive(ws_mutex);
}

/* ---------- WebSocket URI 处理函数 (HTTP server task 上下文) ---------- */

static esp_err_t ws_handler(httpd_req_t *req) {
    int fd = httpd_req_to_sockfd(req);
    ws_add_client(fd);
    ESP_LOGI(TAG, "Client connected, fd=%d, total=%d", fd, ws_count);

    uint8_t buf[64];
    httpd_ws_frame_t pkt = { .payload = buf };

    while (1) {
        esp_err_t ret = httpd_ws_recv_frame(req, &pkt, sizeof(buf));
        if (ret != ESP_OK) {
            break;
        }
        if (pkt.type == HTTPD_WS_TYPE_CLOSE) {
            break;
        }
    }

    ws_remove_client(fd);
    ESP_LOGI(TAG, "Client disconnected, fd=%d, remaining=%d", fd, ws_count);
    return ESP_OK;
}

/* ---------- 主任务 ---------- */

void websocket_task(void *pvParameters) {
    (void)pvParameters;

    ws_mutex = xSemaphoreCreateMutex();

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = WEBSOCKET_PORT;
    cfg.recv_wait_timeout = 3600;

    if (httpd_start(&ws_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed");
        vTaskDelete(NULL);
        return;
    }

    httpd_uri_t ws_uri = {
        .uri         = "/",
        .method      = HTTP_GET,
        .handler     = ws_handler,
        .user_ctx    = NULL,
        .is_websocket = true,
    };
    httpd_register_uri_handler(ws_server, &ws_uri);

    ESP_LOGI(TAG, "WebSocket server started on port %d", WEBSOCKET_PORT);

    while (1) {
        xEventGroupWaitBits(eg_sync,
            BIT_LOG_Q_READY | BIT_TEMP_Q_READY,
            pdFALSE, pdTRUE, portMAX_DELAY);

        vector_polar_t vectors[Q_POLAR_DEPTH];
        for (int i = 0; i < Q_POLAR_DEPTH; i++) {
            xQueueReceive(q_log, &vectors[i], 0);   // 接收极坐标数据
        }
        float temp = NAN;
        xQueueReceive(q_temp, &temp, 0);    // 接收温度数据
        xEventGroupClearBits(eg_sync, BIT_LOG_Q_READY | BIT_TEMP_Q_READY);
        int64_t ts_us = esp_timer_get_time();

        cJSON *root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "ts", (double)ts_us);     // 时间戳
        cJSON_AddNumberToObject(root, "temp", (double)temp);    // 温度数据

        cJSON *arr = cJSON_AddArrayToObject(root, "vectors");
        for (int i = 0; i < Q_POLAR_DEPTH; i++) {   // 所有向量数据
            cJSON *v = cJSON_CreateObject();
            cJSON_AddNumberToObject(v, "a", (double)vectors[i].angle_deg);
            cJSON_AddNumberToObject(v, "d", (double)vectors[i].distance_mm);
            cJSON_AddItemToArray(arr, v);
        }

        char *json_str = cJSON_PrintUnformatted(root);  // 转化为字节流
        if (json_str) {
            ws_send_all(json_str, strlen(json_str));
            ESP_LOGI(TAG, "Sent %d bytes to %d client(s)\n", (int)strlen(json_str), ws_count);
            free(json_str);
        }
        cJSON_Delete(root);
    }
}
