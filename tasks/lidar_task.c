/**
 * @file lidar_task.c
 * @brief LD14P 传感器任务 — 驱动轮询 → 360→72 扇区降采样 → 入队
 */
#include "tasks/lidar_task.h"
#include "drivers/ld14p/ld14p.h"
#include "apf_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

extern ld14p_handle_t g_ld14p;   // main.c 定义

static const char *TAG = "LIDAR_TASK";

/**
 * @brief LD14P_POINTS_ALL → LIDAR_SECTORS 扇区降采样, 最小值加权平均
 *
 * 加权公式: out = (Σ valid_points + LIDAR_MIN_WEIGHT × d_min) /
 *                 (valid_pts + LIDAR_MIN_WEIGHT)
 *
 * dst ≤ 0 的点跳过; 有效点计入 *valid_out.
 *
 * @param raw       原始全周点云
 * @param valid_out [出参] 有效感知区点数
 * @return 扇区数组指针 (静态 buffer, 下次调用覆盖)
 */
static const vector_polar_t *lidar_process(
    const ld14p_polar_t raw[LD14P_POINTS_ALL], uint16_t *valid_out) {
    static vector_polar_t sectors[LIDAR_SECTORS];
    const int sector_width = LD14P_POINTS_ALL / LIDAR_SECTORS;

    for (int s = 0; s < LIDAR_SECTORS; s++) {
        int base = s * sector_width;
        int end  = base + sector_width;

        float d_min = 65535.0f, sum = 0.0f;
        int valid_pts = 0;  // 区间内有效点

        for (int j = base; j < end; j++) {
            float d = raw[j].dst;
            if (d <= 0.0f) continue;    // 无效点跳过
            if (d < 6000.0f) *valid_out += 1;   // 统计有效感知区点数
            sum += d;
            valid_pts++;
            if (d < d_min) d_min = d;   // 寻找最小值
        }

        if (valid_pts == 0) {
            sectors[s].dst = 0;     // 采样后无效点标记
            continue;
        }

        // 添加雷达角度偏移值
        sectors[s].ang = (float)(base + end) / 2.0f + LIDAR_SHIFT_DEG;
        sum += LIDAR_MIN_WEIGHT * d_min;    // 最小值加权, 增强近距敏感性
        sectors[s].dst = sum / (LIDAR_MIN_WEIGHT + valid_pts);
    }

    return sectors;
}

void ld14p_task(void *pvParameters) {
    (void)pvParameters;
    uint32_t skip_count = 0;
    ESP_LOGI(TAG, "Lidar task started @%dHz", SENSOR_FREQ);

    while (1) {
        const ld14p_polar_t *cloud = NULL;
        ld14p_err_t err = ld14p_process(&g_ld14p, &cloud);
        if (err != LD14P_OK) {      // 无完整圈/帧不完整 → 让步继续轮询
            if (err == LD14P_ERR_UART) {
                ESP_LOGW(TAG, "UART read error, retrying...");
            }
            vTaskDelay(2);
            continue;
        }

        // cloud 指向驱动句柄内点云 (本任务独占消费, 无需额外快照)
        uint16_t valid = 0;
        const vector_polar_t *sectors = lidar_process(cloud, &valid);
        ESP_LOGI(TAG, "REV: %lu / %d valid (%d s)",
            valid, LD14P_POINTS_ALL, LIDAR_SECTORS);

        // 几何校准 (当前未启用, PCB 0° 反向待修正):
        //   需可变快照, 启用时改用静态缓冲:
        //   static ld14p_polar_t cal_buf[LD14P_POINTS_ALL];
        //   memcpy(cal_buf, cloud, sizeof(cal_buf));
        //   ld14p_calibrate(cal_buf, LD14P_POINTS_ALL, 5.9f, -18.975571f);
        //   sectors = lidar_process(cal_buf, &valid);

        // q_polar 被占时跳过, 每 16 跳告警一次
        if (!(xEventGroupGetBits(eg_sync) & BIT_LIDAR_Q_READY)) {
            for (int j = 0; j < LIDAR_SECTORS; j++) {
                xQueueSend(q_polar, &sectors[j], 0);
            }
            xEventGroupSetBits(eg_sync, BIT_LIDAR_Q_READY);
            skip_count = 0;
        } else {
            if ((skip_count++ & 0xF) == 0) {
                ESP_LOGW(TAG, "Skip #%lu: q_polar occupied", skip_count);
            }
        }
    }
}
