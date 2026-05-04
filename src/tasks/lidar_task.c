#include "tasks/lidar_task.h"
#include "drivers/ld14p.h"
#include "all_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "LIDAR_TASK";

void ld14p_sensor_task(void *arg) {
    QueueHandle_t queue = (QueueHandle_t)arg;
    ESP_LOGI(TAG, "Sensor task started");

    vector_polar_t scan_buf[LD14P_POINTS_PER_REV];
    uint16_t count = 0;
    TickType_t last_rev = xTaskGetTickCount();
    TickType_t last_report = xTaskGetTickCount();

    while (1) {
        /* ---- 连续读取(阻塞1tick), 每200批强制退出一次 ---- */
        uint8_t buf[128];
        int len, batch = 0;
        while (batch < 200 && (len = uart_read_bytes(LD14P_UART_NUM, buf, sizeof(buf), 1)) > 0) {
            for (int i = 0; i < len; i++)
                ld14p_feed_byte(buf[i]);
            batch++;
        }

        if (ld14p_scan_ready()) {
            if (ld14p_get_scan(scan_buf, &count) == ESP_OK) {
                xQueueReset(queue);
                for (int i = 0; i < count; i++)
                    xQueueSend(queue, &scan_buf[i], 0);
            }
            last_rev = xTaskGetTickCount();
        }

        TickType_t now = xTaskGetTickCount();
        if (now - last_rev >= pdMS_TO_TICKS(500)) {
            if (ld14p_get_scan(scan_buf, &count) == ESP_OK) {
                xQueueReset(queue);
                for (int i = 0; i < count; i++)
                    xQueueSend(queue, &scan_buf[i], 0);
            }
            last_rev = now;
        }

        if (now - last_report >= pdMS_TO_TICKS(2000)) {
            size_t buffered = 0;
            uart_get_buffered_data_len(LD14P_UART_NUM, &buffered);
            ESP_LOGI(TAG, "STAT: fed=%lu buffered=%u batch=%d",
                     ld14p_get_total_bytes(), (unsigned)buffered, batch);
            last_report = now;
        }

        if (batch == 0)
            vTaskDelay(1);
    }
}
