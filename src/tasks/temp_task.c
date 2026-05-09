/**
 * @file temp_task.c
 * @brief DS18B20 温度传感器任务 — 4Hz 周期性采样
 *
 * 数据流:
 *   ds18b20_start_conversion() → 轮询 ds18b20_poll() → ds18b20_read_temp()
 *   → xQueueSend(q_temp) → xEventGroupSetBits(eg_sync, BIT_TEMP_READY)
 *
 * 周期: 250ms (4Hz), 禁 Tickless Idle 确保 vTaskDelay 可靠.
 * 10-bit 分辨率: 188ms 转换 + 10ms 读取 ≈ 200ms/cycle < 250ms.
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

    const TickType_t period = pdMS_TO_TICKS(1000 / DS18B20_TASK_FREQ);
    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "Temperature task started @%dHz", DS18B20_TASK_FREQ);

    while (1) {
        if (ds18b20_start_conversion() != ESP_OK) {
            ESP_LOGW(TAG, "Conversion start failed, retrying in %lums",
                     (unsigned long)(1000 / DS18B20_TASK_FREQ));
            vTaskDelay(period);
            last_wake = xTaskGetTickCount();
            continue;
        }

        /* 轮询转换完成 (读时隙: 0=转换中, 1=完成) */
        int timeout = 0;
        while (!ds18b20_poll() && timeout < 1200) {
            vTaskDelay(1);
            timeout++;
        }

        float temp = ds18b20_read_temp();
        if (isnan((double)temp)) {
            ESP_LOGW(TAG, "Read failed (CRC or bus error)");
        } else {
            ESP_LOGI(TAG, "Temperature: %.2f°C", (double)temp);

            if (xEventGroupGetBits(eg_sync) & BIT_TEMP_READY) {
                ESP_LOGW(TAG, "Temp dropped: previous data unread");
            } else {
                xQueueSend(q_temp, &temp, 0);
                xEventGroupSetBits(eg_sync, BIT_TEMP_READY);
            }
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
