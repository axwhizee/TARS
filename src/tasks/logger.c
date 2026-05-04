#include "tasks/logger.h"
#include "all_defs.h"
#include "drivers/ld14p.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "LOGGER";
static uint32_t upload_count = 0;

void logger_task(void *arg) {
    QueueHandle_t queue = (QueueHandle_t)arg;
    uint32_t alive = 0;
    ESP_LOGI(TAG, "Logger task started on core %d", xPortGetCoreID());

    vector_polar_t scan_buf[LD14P_POINTS_PER_REV];

    while (1) {
        ESP_LOGI(TAG, "Waiting data");
        int i;
        for (i = 0; i < LD14P_POINTS_PER_REV; i++) {
            if (xQueueReceive(queue, &scan_buf[i], pdMS_TO_TICKS(1000)) != pdTRUE) {
                /* 1000ms 无数据 → 打印心跳 + 驱动统计 */
                ESP_LOGW(TAG, "Beat #%lu: fed=%lu (got %d/360)", alive++,
                         ld14p_get_total_bytes(), i);
                break;
            }
        }
        if (i < LD14P_POINTS_PER_REV)
            continue;   /* 未收完, 重新等待 */

        uint32_t valid = 0;
        for (i = 0; i < LD14P_POINTS_PER_REV; i++)
            if (scan_buf[i].distance_mm < 60000) valid++;

        upload_count++;
        ESP_LOGI(TAG, "Upload #%lu: %lu / %d valid", upload_count, valid, LD14P_POINTS_PER_REV);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
