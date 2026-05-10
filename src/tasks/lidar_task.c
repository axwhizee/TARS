/**
 * @file lidar_task.c
 * @brief LD14P 传感器任务
 *
 * 数据流:
 *   UART1 → ld14p_feed_byte(byte)  → ld14p_frame_t*
 *         → ld14p_process_frame(frm) → 更新 cloud_360[], 返回 true=圈完成
 *         → ld14p_get_cloud(raw)    → vector_polar_t[360]
 *         → lidar_process(raw, out) → 降采样 360→36 点 (最小距离加权平均)
 *         → xQueueSend(q_polar)     → 推送 36 点
 *         → xEventGroupSetBits       → 通知 APF 任务
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
 * @brief lidar_process — 360 点 → 36 扇区降采样 (最小距离加权平均)
 *
 * 将 360 个原始点按 10° 间隔划分为 36 个扇区 (0°~9°, 10°~19°, ...).
 * 每扇区内用 1/d² 加权平均: 距离越近的点权重越大, 障碍物信号被强化.
 * 输出角度为扇区中点: 5°, 15°, 25°, ..., 355°.
 */
static void lidar_process(const vector_polar_t raw[LD14P_POINTS_PER_REV],
                          vector_polar_t out[LIDAR_SECTORS]) {
    for (int s = 0; s < LIDAR_SECTORS; s++) {
        int base = s * 10;          /* 扇区起始角度索引 */

        float sum_wx = 0.0f;        /* Σ(weight × distance) */
        float sum_w  = 0.0f;        /* Σ(weight) */

        for (int j = 0; j < 10; j++) {
            float d = raw[base + j].distance_mm;
            if (d > 0.0f && d < 60000.0f) {     /* 有效点, d>0 防止 1/d² 除零产生 NaN */
                float w = 1.0f / (d * d);        /* 反平方权重: 近强远弱 */
                sum_wx += w * d;
                sum_w  += w;
            }
        }

        out[s].angle_deg   = 5.0f + s * 10.0f;    /* 扇区中点角度 */
        out[s].distance_mm = (sum_w > 0.0f) ? (sum_wx / sum_w) : 65535.0f;
    }
}

void ld14p_task(void *pvParameters) {
    (void)pvParameters;
    ESP_LOGI(TAG, "Lidar task started @4Hz");
    uint32_t rev_count = 0;

    while (1) {
        /* 自旋等待消费者取走上轮数据再开始下一轮 UART 读取 */
        while (xEventGroupGetBits(eg_sync) & BIT_LIDAR_READY) {
            vTaskDelay(2);  // 约20ms的任务挂起
        }   // 似乎没有能等待事件组变为0的办法，如果想要优化这段代码，只能调整系统架构，比如添加一个空闲标志位，或者改用二值信号量

        /* ─── 非阻塞 drain UART ring buffer ─── */
        uint8_t buf[256];
        int len;
        while ((len = uart_read_bytes(LD14P_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(10))) > 0) {
            for (int i = 0; i < len; i++) {
                const ld14p_frame_t *frm = ld14p_feed_byte(buf[i]);

                /* CRC 验证通过 + 完整一圈 */
                if (frm && ld14p_process_frame(frm)) {
                    vector_polar_t lidar_raw[LD14P_POINTS_PER_REV];
                    /* 检查 lidar_raw 中的有效点数量 */
                    uint32_t valid = ld14p_get_cloud(lidar_raw);
                    rev_count++;
                    ESP_LOGI(TAG, "REV #%lu: %lu / %d valid → %d sectors",
                             rev_count, valid, LD14P_POINTS_PER_REV, LIDAR_SECTORS);

                    /* 降采样: 360 点 → 36 扇区 */
                    vector_polar_t sectors[LIDAR_SECTORS];
                    lidar_process(lidar_raw, sectors);

                    for (int i = 0; i < LIDAR_SECTORS; i++) {
                        xQueueSend(q_polar, &sectors[i], 0);
                    }
                    xEventGroupSetBits(eg_sync, BIT_LIDAR_READY);
                }
            }
        }

        // 由于有自旋20ms挂起，暂时注释
        // vTaskDelay(2);
    }
}
