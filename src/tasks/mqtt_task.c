/**
 * @file mqtt_task.c
 * @brief MQTT 日志任务 — 接收 APF 转发数据 + 温度 → JSON → MQTT 发布
 *
 * 同步:
 *   等待 BIT_MQTT_Q_READY | BIT_TEMP_Q_READY 双就绪 → 消费 q_mqtt + q_temp
 *   → 清除两个事件位 (释放生产者) → 组装 JSON → MQTT 发布
 */
#include "tasks/mqtt_task.h"
#include "all_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "cJSON.h"
#include "mqtt_client.h"
#include <string.h>
#include <math.h>

static const char *TAG = "MQTT_TASK ";
static esp_mqtt_client_handle_t mqtt_client = NULL;

void mqtt_task(void *pvParameters) {
    (void)pvParameters;

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(mqtt_client);
    ESP_LOGI(TAG, "MQTT client started, broker=%s topic=%s", MQTT_BROKER_URI, MQTT_TOPIC);

    while (1) {
        /* 1. 等待 MQTT 批次 + 温度双就绪 */
        xEventGroupWaitBits(
            eg_sync,
            BIT_MQTT_Q_READY | BIT_TEMP_Q_READY,
            pdFALSE,
            pdTRUE,
            portMAX_DELAY
        );

        /* 2. 读取 q_mqtt: 41 个 vector_polar_t */
        vector_polar_t vectors[Q_POLAR_DEPTH];
        for (int i = 0; i < Q_POLAR_DEPTH; i++) {
            xQueueReceive(q_mqtt, &vectors[i], 0);
        }

        /* 3. 读取 q_temp: 温度 */
        float temp = NAN;
        xQueueReceive(q_temp, &temp, 0);

        /* 4. 清除事件位, 释放传感器 */
        xEventGroupClearBits(eg_sync, BIT_MQTT_Q_READY | BIT_TEMP_Q_READY);
        ESP_LOGI(TAG, "Batch consumed, bits cleared");

        /* 5. 时间戳 (MQTT 发帧时刻) */
        int64_t ts_us = esp_timer_get_time();

        /* 6. 组装 JSON */
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
            int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, json_str, 0, 0, 0);
            ESP_LOGI(TAG, "Published %d bytes, msg_id=%d", (int)strlen(json_str), msg_id);
            free(json_str);
        }
        cJSON_Delete(root);
    }
}
