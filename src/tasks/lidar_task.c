#include "tasks/lidar_task.h"
#include "drivers/ld14p.h"
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

    while (1) {
        uint8_t byte;
        while (uart_read_bytes(UART_NUM_1, &byte, 1, 0) > 0)
            ld14p_feed_byte(byte);

        if (ld14p_scan_ready()) {
            if (ld14p_get_scan(scan_buf, &count) == ESP_OK) {
                xQueueReset(queue);
                for (int i = 0; i < count; i++)
                    xQueueSend(queue, &scan_buf[i], 0);
            }
        } else {
            vTaskDelay(1);
        }
    }
}
