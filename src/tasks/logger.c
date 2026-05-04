#include "tasks/logger.h"
#include "all_defs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "LOGGER";

void logger_task(void *arg) {
    QueueHandle_t queue = (QueueHandle_t)arg;
    ESP_LOGI(TAG, "Logger task started on core %d", xPortGetCoreID());

    vector_polar_t scan_buf[LD14P_POINTS_PER_REV];

    while (1) {
        ESP_LOGI(TAG, "Waiting data");
        for (int i = 0; i < LD14P_POINTS_PER_REV; i++)
            xQueueReceive(queue, &scan_buf[i], portMAX_DELAY);

        uint16_t payload_len = LD14P_POINTS_PER_REV * sizeof(vector_polar_t);
        uint8_t hdr[] = {
            UPLOAD_HEADER,
            (uint8_t)(payload_len & 0xFF),
            (uint8_t)(payload_len >> 8)
        };

        uart_write_bytes(UART_NUM_0, hdr, sizeof(hdr));
        uart_write_bytes(UART_NUM_0, (const char *)scan_buf, payload_len);

        ESP_LOGI(TAG, "Uploaded %d bytes", sizeof(hdr) + payload_len);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
