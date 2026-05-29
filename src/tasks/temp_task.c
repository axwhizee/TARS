/**
 * @file temp_task.c
 * @brief DS18B20 温度传感器任务 — SENSOR_FREQ Hz 周期性采样, 非阻塞跳过模式
 */
#include "tasks/temp_task.h"
#include "drivers/ds18b20.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "TEMP_TASK";

void temp_task(void *pvParameters) {
    (void)pvParameters;

    const TickType_t period = pdMS_TO_TICKS(1000 / SENSOR_FREQ);    // 工作周期
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t skip_count = 0;
    ESP_LOGI(TAG, "Temperature task started @%dHz", SENSOR_FREQ);

    while (1) {
        vTaskDelayUntil(&last_wake, period);    // 任务周期性运行

        if (ds18b20_start_conversion() != ESP_OK) {
            ESP_LOGW(TAG, "Conversion start failed");
            continue;
        }

        /* 等待转换完成 (DS18B20_RES_BITS bit, 留足余量) */
        vTaskDelay(pdMS_TO_TICKS(200));

        float temp = ds18b20_read_temp();
        if (isnan((double)temp)) {
            ESP_LOGW(TAG, "Read failed (CRC or bus error)");
        } else {
            ESP_LOGI(TAG, "Temperature: %.2f°C", (double)temp);

            if (xEventGroupGetBits(eg_sync) & BIT_TEMP_Q_READY) {
                if ((skip_count++ & 0xF) == 0) {
                    ESP_LOGW(TAG, "Temp skipped: q_temp occupied (skip #%lu)", skip_count);
                }
            } else {
                xQueueSend(q_temp, &temp, 0);
                xEventGroupSetBits(eg_sync, BIT_TEMP_Q_READY);
                skip_count = 0;
            }
        }

    }
}
