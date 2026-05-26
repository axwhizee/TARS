/**
 * @file lidar_task.c
 * @brief LD14P 传感器任务 — 非阻塞跳过模式
 */
#include "tasks/lidar_task.h"
#include "drivers/ld14p.h"
#include "all_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "LIDAR_TASK";

/**
 * @brief 360 点点云降采样 ，采用最小值加权平均，确保具备最小距离敏感性
 * @param raw 360个原始数据点
 * @param out 指定的降采样点数
 */
static void lidar_process(const vector_polar_t raw[LD14P_POINTS_PER_REV],
    vector_polar_t out[LIDAR_SECTORS]) {
    // 计算每个降采样区间的点数
    const int sector_width = LD14P_POINTS_PER_REV / LIDAR_SECTORS;

    for (int s = 0; s < LIDAR_SECTORS; s++) {
        int base = (s * sector_width);          // 区间起始点位置
        int end = ((s + 1) * sector_width);     // 区间终止点位置

        float d_min = 65535.0f, sum = 0.0f;
        for (int j = base; j < end; j++) {  // 寻找最小值
            float d_tmp = raw[j].distance_mm;
            sum += d_tmp;
            d_min = (d_tmp < d_min) ? d_tmp : d_min;
        }
        sum += LIDAR_MIN_WEIGHT * d_min;    // 最小距离加权

        out[s].angle_deg = (float)(base + end) / 2.0f;
        out[s].distance_mm = sum / (LIDAR_MIN_WEIGHT + sector_width);
    }
}

void ld14p_task(void *pvParameters) {
    (void)pvParameters;
    uint32_t skip_count = 0;
    ESP_LOGI(TAG, "Lidar task started @4Hz");

    while (1) {
        // 非阻塞 drain UART ring buffer
        uint8_t buf[256];
        int len;
        while ((len = uart_read_bytes(LD14P_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(25))) > 0) {
            for (int i = 0; i < len; i++) {
                const ld14p_frame_t *frm = ld14p_feed_byte(buf[i]);     // 仅在接收到完整数据帧时返回有效地址

                // 等待完整帧 & 完整一圈采集完毕（完整的数据存放在驱动中的 cloud_360 数组中）
                if (frm && ld14p_process_frame(frm)) {
                    vector_polar_t lidar_raw[LD14P_POINTS_PER_REV];
                    // 获取完整数据并检查 lidar_raw 中的有效点数量
                    uint16_t valid = ld14p_get_cloud(&lidar_raw);
                    ESP_LOGI(TAG, "REV: %lu / %d valid → %d sectors",
                        valid, LD14P_POINTS_PER_REV, LIDAR_SECTORS);
                    // 最小值加权平均降采样
                    vector_polar_t sectors[LIDAR_SECTORS];
                    lidar_process(lidar_raw, sectors);

                    // 确保雷达对应的事件位为空
                    if (!(xEventGroupGetBits(eg_sync) & BIT_LIDAR_Q_READY)) {
                        for (int i = 0; i < LIDAR_SECTORS; i++) {
                            xQueueSend(q_polar, &sectors[i], 0);
                        }
                        xEventGroupSetBits(eg_sync, BIT_LIDAR_Q_READY);
                        skip_count = 0;
                    } else {
                        if ((skip_count++ & 0xF) == 0) {
                            ESP_LOGW(TAG, "Lidar skipped: q_polar occupied (skip #%lu)", skip_count);
                        }
                    }
                }
            }
        }

        vTaskDelay(2);
    }
}
