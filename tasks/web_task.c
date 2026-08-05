/**
 * @file web_task.c
 * @brief HTTP 静态文件服务 + WebSocket JSON 推送 + 手动遥控
 *
 * 数据路径:
 *   A) HTTP GET → 预加载的 PSRAM buffer (零 fopen 争用)
 *   B) WS 下行周期 → apf_task→q_log→主循环→snprintf JSON→ws_send_all 广播
 *   C) WS 下行事件 → ws_handler 内 snprintf 响应帧, 直接发送给请求者
 *   D) WS 上行 → 事件驱动 ws_handler (单帧即返)→cJSON 解析→统一 action 分发
 *
 * 上行协议: 所有消息统一使用 "action" 字段分发
 *   {"action":"set_mode",     "mode":"manual"|"auto"}
 *   {"action":"cmd",          "dx":N, "dy":N}
 *   {"action":"get_params"}
 *   {"action":"update_params","params":{...12 fields...}}
 *   {"action":"reset_params"}
 *
 * 下行协议: 两种类型, 不混合
 *   周期帧 (4Hz): {"ts":N, "temp":F|null, "cart":{dx,dy}, "vectors":[...]}
 *   事件帧:       {"action":"mode_changed","mode":"manual"|"auto"}
 *                 {"action":"params","params":{...12 fields...}}
 */

#include "tasks/web_task.h"
#include "tasks/apf_task.h"
#include "apf_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "cJSON/cJSON.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#define WS_MAX_CLIENTS  4   // 最大并发 WebSocket 客户端数
#define HTTPD_MAX_FDS   16  // httpd 内部最大文件描述符数 (含 WS socket + TCP)

static const char *TAG = "WEB_TASK ";

static httpd_handle_t ws_server = NULL;       // HTTP 服务器句柄
static int            ws_fds[WS_MAX_CLIENTS]; // 已连接 WS 客户端的 socket fd 数组
static int            ws_count = 0;           // 当前连接数
static SemaphoreHandle_t ws_mutex = NULL;     // 保护 ws_fds/ws_count 的互斥锁

// 静态文件预加载
typedef struct {
    const char *uri;   // 请求路径, 如 "/index.html"
    const char *mime;  // Content-Type, 如 "text/html"
    uint8_t    *data;  // 文件内容 (malloc 分配在 PSRAM, 启动后永不释放)
    size_t      len;   // 文件字节数
} static_file_t;

static static_file_t s_files[3];

// 启动时一次性将 SPIFFS 文件预加载到 PSRAM, 后续 HTTP GET 零 fopen
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

// 注册新 WS 客户端 fd, 超过 WS_MAX_CLIENTS 则静默丢弃
static void ws_add_client(int fd) {
    xSemaphoreTake(ws_mutex, portMAX_DELAY);
    if (ws_count < WS_MAX_CLIENTS) ws_fds[ws_count++] = fd;
    xSemaphoreGive(ws_mutex);
}

// O(1) 交换式 WS 删除: 用最后一个元素覆盖被删位置
static void ws_remove_client(int fd) {
    xSemaphoreTake(ws_mutex, portMAX_DELAY);
    for (int i = 0; i < ws_count; i++) {
        if (ws_fds[i] == fd) { ws_fds[i] = ws_fds[--ws_count]; break; }
    }
    xSemaphoreGive(ws_mutex);
}

// WS 下行: 广播给所有已连接客户端, 失败的 fd 在更新列表时懒清理
static void ws_send_all(const char *data, size_t len) {
    httpd_ws_frame_t pkt = {
        .type    = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)data,
        .len     = len,
    };

    // 步骤 1: 快照客户端列表 (持锁)
    int count;
    int fds[WS_MAX_CLIENTS];
    xSemaphoreTake(ws_mutex, portMAX_DELAY);
    count = ws_count;
    memcpy(fds, ws_fds, count * sizeof(int));
    xSemaphoreGive(ws_mutex);

    // 步骤 2: 逐个发送, 收集存活 fd
    int live = 0;
    for (int i = 0; i < count; i++) {
        esp_err_t err = httpd_ws_send_frame_async(ws_server, fds[i], &pkt);
        if (err == ESP_OK)
            fds[live++] = fds[i];        // 发送成功, 保留
        else
            ESP_LOGW(TAG, "WS send failed fd=%d err=%d, removing", fds[i], err);
    }

    // 步骤 3: 更新活跃客户端列表 (持锁)
    xSemaphoreTake(ws_mutex, portMAX_DELAY);
    ws_count = live;
    memcpy(ws_fds, fds, live * sizeof(int));
    xSemaphoreGive(ws_mutex);
}

// 单播: 向发起请求的客户端发送一帧事件响应
static void ws_send_to(httpd_req_t *req, const char *data, size_t len) {
    httpd_ws_frame_t rp = {
        .type    = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)data,
        .len     = len,
    };
    httpd_ws_send_frame(req, &rp);
}

/**
 * @brief APF 参数 JSON 构建 (复用于 get_params / reset_params)
 * 将 g_apf_params 序列化为 {"action":"params","params":{...}} 格式
 */
static int build_apf_params_json(char *buf, size_t bufsz) {
    return snprintf(buf, bufsz,
        "{\"action\":\"params\",\"params\":{"
        "\"range_rep\":%.2f,"
        "\"gain_rep_x\":%.2f,\"gain_rep_y\":%.2f,"
        "\"rep_nx\":%.2f,\"rep_ny\":%.2f,\"att_base\":%.2f,"
        "\"vfh_thresh\":%.2f,\"vfh_min_w\":%d,"
        "\"vfh_smooth_w\":%.2f,\"vfh_free_th\":%.2f,"
        "\"vfh_goal_bias\":%.2f,\"vfh_ema_alpha\":%.2f}}",
        g_apf_params.range_rep,
        g_apf_params.gain_rep_x, g_apf_params.gain_rep_y,
        g_apf_params.rep_nx, g_apf_params.rep_ny, g_apf_params.att_base,
        g_apf_params.vfh_thresh, g_apf_params.vfh_min_w,
        g_apf_params.vfh_smooth_w, g_apf_params.vfh_free_th,
        g_apf_params.vfh_goal_bias, g_apf_params.vfh_ema_alpha);
}

// APF 参数数据驱动校验表
typedef struct {
    const char *key;
    float *field_f;     // float 字段指针 (int 字段为 NULL)
    int   *field_i;     // int 字段指针 (float 字段为 NULL)
    float  min_val;
    float  max_val;
} param_field_t;

// clang-format off
static const param_field_t PARAM_FIELDS[] = {
    {"range_rep",     &g_apf_params.range_rep,     NULL, APF_RANGE_MIN,    APF_RANGE_MAX},
    {"gain_rep_x",    &g_apf_params.gain_rep_x,    NULL, 0.0f,             500.0f},
    {"gain_rep_y",    &g_apf_params.gain_rep_y,    NULL, 0.0f,             500.0f},
    {"rep_nx",        &g_apf_params.rep_nx,        NULL, 0.1f,             5.0f},
    {"rep_ny",        &g_apf_params.rep_ny,        NULL, 0.1f,             5.0f},
    {"att_base",      &g_apf_params.att_base,      NULL, 0.0f,             2000.0f},
    {"vfh_thresh",    &g_apf_params.vfh_thresh,    NULL, 200.0f,           5000.0f},
    {"vfh_min_w",     NULL, &g_apf_params.vfh_min_w,     1.0f,            20.0f},
    {"vfh_smooth_w",  &g_apf_params.vfh_smooth_w,  NULL, 0.1f,            10.0f},
    {"vfh_free_th",   &g_apf_params.vfh_free_th,   NULL, 0.01f,           1.0f},
    {"vfh_goal_bias", &g_apf_params.vfh_goal_bias, NULL, 0.0f,            1.0f},
    {"vfh_ema_alpha", &g_apf_params.vfh_ema_alpha, NULL, 0.0f,            1.0f},
};
// clang-format on

#define PARAM_FIELD_COUNT (sizeof(PARAM_FIELDS) / sizeof(PARAM_FIELDS[0]))

// 遍历校验表, 逐字段从 JSON 解析并写入 g_apf_params
static void apply_params_from_json(cJSON *params) {
    for (int i = 0; i < (int)PARAM_FIELD_COUNT; i++) {
        const param_field_t *f = &PARAM_FIELDS[i];
        cJSON *item = cJSON_GetObjectItem(params, f->key);
        if (!item || !cJSON_IsNumber(item)) continue;
        float v = (float)item->valuedouble;
        if (v < f->min_val || v > f->max_val) continue;
        if (f->field_i)
            *f->field_i = (int)v;
        else
            *f->field_f = v;
    }
}

// HTTP 静态文件处理，从预加载的 PSRAM buffer 中匹配 URI 并返回, 404 则返回 FAIL
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

// WS 断连回调，httpd 框架在连接断开时自动调用 (CLOSE/RST/超时)
static void ws_close_cb(httpd_handle_t hd, int sockfd) {
    (void)hd;
    ws_remove_client(sockfd);
    ESP_LOGI(TAG, "WS client disconnected (close_cb), fd=%d", sockfd);
}

/**
 * @brief WS 上行 handler: 统一 action 分发
 * 事件驱动回调 (无 while(1)), 每次只处理一帧, 关键约束见文件头注释 
 */
static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        ws_add_client(fd);
        ESP_LOGI(TAG, "WS client connected, fd=%d, total=%d", fd, ws_count);
        return ESP_OK;
    }

    httpd_ws_frame_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    // Step A: 读帧头 (max_len=0)
    esp_err_t ret = httpd_ws_recv_frame(req, &pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WS recv header failed: %d", ret);
        return ESP_FAIL;
    }

    if (pkt.type == HTTPD_WS_TYPE_CLOSE) return ESP_OK;    // close_fn 自动清理

    if (pkt.type == HTTPD_WS_TYPE_PING) {
        uint8_t ping_buf[128];
        if (pkt.len > 0 && pkt.len <= sizeof(ping_buf)) {
            pkt.payload = ping_buf;
            ret = httpd_ws_recv_frame(req, &pkt, pkt.len);
        }
        if (ret == ESP_OK) {
            pkt.type = HTTPD_WS_TYPE_PONG;
            httpd_ws_send_frame(req, &pkt);
        }
        return ESP_OK;
    }

    if (pkt.type == HTTPD_WS_TYPE_TEXT && pkt.len > 0) {
        if (pkt.len > 512) {
            ESP_LOGW(TAG, "WS msg too long (%d bytes)", (int)pkt.len);
            return ESP_FAIL;
        }

        uint8_t stack_buf[256];
        uint8_t *payload = stack_buf;
        if (pkt.len > sizeof(stack_buf)) {
            payload = malloc(pkt.len);
            if (!payload) return ESP_FAIL;
        }

        pkt.payload = payload;
        // Step B: 读完整载荷
        ret = httpd_ws_recv_frame(req, &pkt, pkt.len);

        if (ret == ESP_OK) {
            cJSON *root = cJSON_ParseWithLength((const char *)pkt.payload, pkt.len);
            if (root) {
                cJSON *action = cJSON_GetObjectItem(root, "action");
                if (action && cJSON_IsString(action)) {
                    const char *act = action->valuestring;

                    if (strcmp(act, "set_mode") == 0) {     // set_mode
                        cJSON *mode = cJSON_GetObjectItem(root, "mode");
                        if (mode && cJSON_IsString(mode)) {
                            if (strcmp(mode->valuestring, "manual") == 0) {
                                xEventGroupSetBits(eg_sync, BIT_MANUAL_MODE);
                                vector_cart_t stop = { .dx = 0.0f, .dy = 0.0f };
                                xQueueOverwrite(q_cart, &stop);
                                ESP_LOGI(TAG, "=== Manual mode ON ===");
                            } else if (strcmp(mode->valuestring, "auto") == 0) {
                                xEventGroupClearBits(eg_sync, BIT_MANUAL_MODE);
                                ESP_LOGI(TAG, "=== Auto mode restored ===");
                            }
                            // 回复 mode_changed 确认帧
                            char resp[64];
                            int n = snprintf(resp, sizeof(resp),
                                "{\"action\":\"mode_changed\",\"mode\":\"%s\"}",
                                (xEventGroupGetBits(eg_sync) & BIT_MANUAL_MODE)
                                    ? "manual" : "auto");
                            ws_send_to(req, resp, n);
                        }
                    } else if (strcmp(act, "cmd") == 0) {   // cmd (摇杆控制)
                        if (xEventGroupGetBits(eg_sync) & BIT_MANUAL_MODE) {
                            cJSON *dx = cJSON_GetObjectItem(root, "dx");
                            cJSON *dy = cJSON_GetObjectItem(root, "dy");
                            if (cJSON_IsNumber(dx) && cJSON_IsNumber(dy)) {
                                vector_cart_t c = {
                                    .dx = (float)dx->valuedouble,
                                    .dy = (float)dy->valuedouble,
                                };
                                xQueueOverwrite(q_cart, &c);
                            }
                        }
                    } else if (strcmp(act, "get_params") == 0) {    // get_params
                        char resp[512];
                        int n = build_apf_params_json(resp, sizeof(resp));
                        ws_send_to(req, resp, n);
                    }
                    else if (strcmp(act, "update_params") == 0) {   // update_params
                        if (xEventGroupGetBits(eg_sync) & BIT_MANUAL_MODE) {
                            cJSON *p = cJSON_GetObjectItem(root, "params");
                            if (p) {
                                apply_params_from_json(p);
                                ESP_LOGI(TAG, "APF params updated via WS");
                                // 回传最新参数值, 与 get_params 同格式
                                char resp[512];
                                int n = build_apf_params_json(resp, sizeof(resp));
                                ws_send_to(req, resp, n);
                            }
                        }
                    } else if (strcmp(act, "reset_params") == 0) {  // reset_params
                        if (xEventGroupGetBits(eg_sync) & BIT_MANUAL_MODE) {
                            params_init_defaults();
                            char resp[512];
                            int n = build_apf_params_json(resp, sizeof(resp));
                            ws_send_to(req, resp, n);
                        }
                    }
                }

                cJSON_Delete(root);
            }
        }
        if (payload != stack_buf) free(payload);
    }

    return ESP_OK;
}

void web_task(void *pvParameters) {
    (void)pvParameters;
    ws_mutex = xSemaphoreCreateMutex();

    preload_static_files();

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port        = WEBSOCKET_PORT;
    cfg.max_open_sockets   = HTTPD_MAX_FDS;
    cfg.lru_purge_enable   = true;
    cfg.close_fn            = ws_close_cb;
    cfg.send_wait_timeout   = 2;

    if (httpd_start(&ws_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed");
        vTaskDelete(NULL);
        return;
    }

    httpd_uri_t file_uris[] = {
        {.uri = "/",          .method = HTTP_GET, .handler = file_get_handler},
        {.uri = "/style.css", .method = HTTP_GET, .handler = file_get_handler},
        {.uri = "/script.js", .method = HTTP_GET, .handler = file_get_handler},
    };
    for (int i = 0; i < sizeof(file_uris) / sizeof(file_uris[0]); i++)
        httpd_register_uri_handler(ws_server, &file_uris[i]);

    httpd_uri_t ws_uri = {
        .uri = "/ws", .method = HTTP_GET, .handler = ws_handler, .is_websocket = true,
    };
    httpd_register_uri_handler(ws_server, &ws_uri);

    ESP_LOGI(TAG, "HTTP + WebSocket server on port %d", WEBSOCKET_PORT);

    char json_buf[4096];
    while (1) {
        xEventGroupWaitBits(eg_sync, BIT_LOG_Q_READY, pdFALSE, pdFALSE, portMAX_DELAY);

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
        vector_cart_t rep = g_apf_rep;
        vector_cart_t att = g_vfh_att;

        float temp = NAN;
        xQueueReceive(q_temp, &temp, 0);

        xEventGroupClearBits(eg_sync, BIT_LOG_Q_READY | BIT_TEMP_Q_READY);

        // 手写 JSON (不用 cJSON: 77 向量 × 4Hz = 200+ malloc/s, 堆碎片严重)
        int64_t ts_us = esp_timer_get_time();
        int pos = snprintf(json_buf, sizeof(json_buf), "{\"ts\":%lld,\"temp\":", ts_us);
        if (isnan(temp))
            pos += snprintf(json_buf + pos, sizeof(json_buf) - pos, "null");
        else
            pos += snprintf(json_buf + pos, sizeof(json_buf) - pos, "%.2f", temp);

        pos += snprintf(json_buf + pos, sizeof(json_buf) - pos,
            ",\"v_apf\":{\"dx\":%.2f,\"dy\":%.2f}"
            ",\"v_vfh\":{\"dx\":%.2f,\"dy\":%.2f}"
            ",\"v_polars\":[",
            rep.dx, rep.dy, att.dx, att.dy);

        for (int i = 0; i < Q_POLAR_DEPTH; i++) {
            if (pos >= (int)sizeof(json_buf) - 40) break;
            pos += snprintf(json_buf + pos, sizeof(json_buf) - pos,
                "{\"a\":%.2f,\"d\":%.2f}%s",
                vectors[i].ang, vectors[i].dst,
                (i < Q_POLAR_DEPTH - 1) ? "," : "");
        }
        pos += snprintf(json_buf + pos, sizeof(json_buf) - pos, "]}");

        ws_send_all(json_buf, pos);
        ESP_LOGI(TAG, "Sent %d bytes to %d client(s)\n", pos, count);
    }
}
