/**
 * @file apf_task.c
 * @brief 人工势场法避障实现
 *
 * 核心公式 (2D APF):
 *   引力:  F_att = (K_att, 0)              // 恒定正向拉力
 *   斥力:  F_rep = -K_rep * w(ra) * (dx/r³, dy/r³)
 *          其中 w(ra) = { danger_wt, ra ≤ 2m; safe_wt, 2m < ra ≤ 6m }
 *   合力:  F_total = F_att + Σ F_rep       // 钳位到 ±6m
 *
 * 距离分区:
 *   < 0.1m   → 丢弃 (死区, 避免自身 / 地面反射)
 *   0.1~2m   → 危险区 (斥力权重 ×2.5)
 *   2~6m     → 感知区 (斥力权重 ×1.0)
 *   >6m      → 噪声 / 忽略
 */
#include "tasks/apf_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "APF_TASK  ";

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


/**
 * @brief 极坐标 → 笛卡尔坐标
 *
 * 坐标系约定 (LD14P):
 *   angle = 0° 为正前方 (机器人 x 轴正向)
 *   angle 顺时针递增 (右侧为正 y)
 */
static inline void polar_to_cart(float distance, float angle_deg, float *dx, float *dy) {
    float rad = angle_deg * (M_PI / 180.0f);
    *dx = distance * cosf(rad);
    *dy = distance * sinf(rad);
}

/**
 * @brief 根据距离返回斥力权重倍率
 */
static inline float repulse_weight(float distance) {
    if (distance <= APF_DANGER_RANGE_MM) {
        return APF_DANGER_REPULSE_WT;   /* 危险区: 强斥力 */
    }
    /* d > 2m 且 d <= 6m → 感知区 */
    return APF_SAFE_REPULSE_WT;
}

static inline void force_check(vector_cart_t *result) {
    /* 1. 异常值拦截 */
    if (isnan(result->x) || isnan(result->y) ||
        isinf(result->x) || isinf(result->y)) {
        ESP_LOGW(TAG, "Invalid force vector, zeroing out");
        result->x = 0.0f;
        result->y = 0.0f;
        return;
    }

    /* 2. 计算合力幅值（使用 hypotf 提升数值稳定性） */
    float mag = hypotf(result->x, result->y);

    /* 3. 死区处理 & 幅值限幅（互斥分支，提升效率） */
    if (mag < APF_MIN_FORCE_MM) {
        result->x = 0.0f;
        result->y = 0.0f;
    } else if (mag > APF_MAX_FORCE_MM) {
        float scale = APF_MAX_FORCE_MM / mag; // 仅一次除法
        result->x *= scale;
        result->y *= scale;
    }
}

void apf_task(void *pvParameters) {
    (void)pvParameters;
    vector_polar_t  batch[Q_POLAR_DEPTH];  /* 批量读取缓冲区 */
    vector_polar_t  dummy, point;
    vector_cart_t   result;
    float rep_fx, rep_fy;
    float dx, dy, ra, r_cubed, f_rep;
    int danger_cnt, safe_cnt, noise_cnt;

    ESP_LOGI(TAG, "APF task started: K_att=%.0f K_rep=%.0f danger<%.0fmm safe<%.0fmm",
        (double)APF_ATTRACT_GAIN, (double)APF_REPULSE_GAIN,
        (double)APF_DANGER_RANGE_MM, (double)APF_SAFE_RANGE_MM
    );

    while (1) {
        /* 1. 等待 LiDAR + 火焰传感器数据全部就绪 */
        EventBits_t bits = xEventGroupWaitBits(
            eg_sync,
            BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY,
            pdFALSE,
            pdTRUE,
            pdMS_TO_TICKS(375)
        );

        if ((bits & (BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY)) != (BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY)) {
            result.x = result.y = 0.0f;
            xQueueOverwrite(q_cart, &result);
            ESP_LOGW(TAG, "Sensor sync timeout");
            continue;
        }

        /* 2. 看门狗: 上周期日志未被消费 → 清空 q_log + 清除 BIT_LOG_Q_READY */
        if (xEventGroupGetBits(eg_sync) & BIT_LOG_Q_READY) {
            ESP_LOGW(TAG, "Log timeout: draining q_log");
            while (xQueueReceive(q_log, &dummy, 0) == pdTRUE) {}
            xEventGroupClearBits(eg_sync, BIT_LOG_Q_READY);
        }

        /* 3. 读取 q_polar → 同时透传至 q_log */
        for (int i = 0; i < Q_POLAR_DEPTH; i++) {
            xQueueReceive(q_polar, &point, 0);
            batch[i] = point;
            xQueueSend(q_log, &point, 0);
        }

        /* 4. 清除传感器位, 通知日志任务 */
        xEventGroupClearBits(eg_sync, BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY);
        xEventGroupSetBits(eg_sync, BIT_LOG_Q_READY);

        /* 5. 离线批量计算 APF */
        rep_fx = 0.0f;
        rep_fy = 0.0f;
        danger_cnt = 0;
        safe_cnt = 0;
        noise_cnt = 0;

        for (int i = 0; i < Q_POLAR_DEPTH; i++) {
            point = batch[i];
            ra = point.distance_mm;

            if (isnan(ra) || isinf(ra) || ra < APF_PERCEPTION_MIN_MM || ra > APF_SAFE_RANGE_MM) {
                noise_cnt++;
                continue;
            }

            polar_to_cart(ra, point.angle_deg, &dx, &dy);
            r_cubed = ra * ra * ra;
            f_rep = APF_REPULSE_GAIN * repulse_weight(ra) / r_cubed;
            rep_fx -= f_rep * dx;
            rep_fy -= f_rep * dy;

            if (ra <= APF_DANGER_RANGE_MM) {
                danger_cnt++;
            } else {
                safe_cnt++;
            }
        }

        result.x = APF_ATTRACT_GAIN + rep_fx;
        result.y = rep_fy;

        force_check(&result);
        xQueueOverwrite(q_cart, &result);
        ESP_LOGI(TAG, "F_cmd = (%.0f,%.0f) | danger=%d safe=%d noise=%d",
            (double)result.x, (double)result.y, danger_cnt, safe_cnt, noise_cnt);
    }
}
