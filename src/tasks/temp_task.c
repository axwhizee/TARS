/**
 * @file temp_task.c
 * @brief 温度传感器任务 — SENSOR_FREQ Hz 周期性采样, 非阻塞跳过模式
 *
 * 使用 ESP32-S3 内置温度传感器取代 DS18B20, 消除 1-Wire 位操作关中断
 * 对系统实时性 (LiDAR UART / WebSocket) 的干扰.
 *
 * 旧 DS18B20 实现保留在 #if 0 块内供参考.
 */
#include "tasks/temp_task.h"
// #include "drivers/ds18b20.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/temperature_sensor.h"
#include <math.h>

static const char *TAG = "TEMP_TASK";

void temp_task(void *pvParameters) {
    (void)pvParameters;

    temperature_sensor_handle_t tsens = NULL;
    temperature_sensor_config_t tsens_cfg =
        TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    temperature_sensor_install(&tsens_cfg, &tsens);
    temperature_sensor_enable(tsens);

    const TickType_t period = pdMS_TO_TICKS(1000 / SENSOR_FREQ);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t skip_count = 0;
    ESP_LOGI(TAG, "Internal temp task started @%dHz", SENSOR_FREQ);

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        float temp;
        if (temperature_sensor_get_celsius(tsens, &temp) != ESP_OK) {
            ESP_LOGW(TAG, "Read failed");
            continue;
        }

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

#if 0  // ---- 旧 DS18B20 实现 (1-Wire 关中断 ~10ms/次, 干扰实时性) ----

#include "drivers/ds18b20.h"

void temp_task_ds18b20(void *pvParameters) {
    (void)pvParameters;

    const TickType_t period = pdMS_TO_TICKS(1000 / SENSOR_FREQ);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t skip_count = 0;
    ESP_LOGI(TAG, "Temperature task started @%dHz", SENSOR_FREQ);

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        if (ds18b20_start_conversion() != ESP_OK) {
            ESP_LOGW(TAG, "Conversion start failed");
            continue;
        }

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

#endif  // ---- 旧 DS18B20 实现 ----
