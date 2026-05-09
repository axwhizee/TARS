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

static inline void force_check(float total_fx, float total_fy, vector_cart_t *result) {
    /* 1. 异常值拦截 */
    if (isnan(total_fx) || isnan(total_fy) || 
        isinf(total_fx) || isinf(total_fy)) {
        ESP_LOGW(TAG, "Invalid force vector, zeroing out");
        result->x = 0.0f;
        result->y = 0.0f;
        return;
    }

    /* 2. 计算合力幅值（使用 hypotf 提升数值稳定性） */
    float mag = hypotf(total_fx, total_fy);

    /* 3. 死区处理 & 幅值限幅（互斥分支，提升效率） */
    if (mag < APF_MIN_FORCE_MM) {
        total_fx = 0.0f;
        total_fy = 0.0f;
    } else if (mag > APF_MAX_FORCE_MM) {
        float scale = APF_MAX_FORCE_MM / mag; // 仅一次除法
        total_fx *= scale;
        total_fy *= scale;
    }

    /* 4. 推送至电机控制任务 */
    result->x = total_fx;
    result->y = total_fy;
}

void apf_task(void *pvParameters) {
    (void)pvParameters;
    vector_polar_t point;       // 接收到的向量
    vector_cart_t   result;     // 最终结果
    float rep_fx, rep_fy;       // 斥力累加器
    float dx, dy, ra, r_cubed, f_rep;    // 单点转换
    float total_fx, total_fy;   // 合力
    float mag;                  // 合力幅值
    int danger_cnt, safe_cnt, noise_cnt;    // 分类统计

    ESP_LOGI(TAG, "APF task started: K_att=%.0f K_rep=%.0f danger<%.0fmm safe<%.0fmm",
        (double)APF_ATTRACT_GAIN, (double)APF_REPULSE_GAIN,
        (double)APF_DANGER_RANGE_MM, (double)APF_SAFE_RANGE_MM
    );

    while (1) {
        /* 1. 等待 LiDAR 数据就绪信号 (375ms 超时) */
        EventBits_t bits = xEventGroupWaitBits(
            eg_sync,
            BIT_LIDAR_READY,
            pdTRUE,    /* 读取后自动清零 */
            pdFALSE,   /* 任意一位即可 (这里只有 1 位) */
            pdMS_TO_TICKS(375)
        );

        if ((bits & BIT_LIDAR_READY) == 0) {
            /* 超时: 推送零指令停止机器人 */
            result.x = result.y = 0.0f;
            xQueueOverwrite(q_cart, &result);
            ESP_LOGW(TAG, "LiDAR sync timeout");
            continue;
        }

        /* 2. 从队列读取全部 LIDAR_SECTORS 个扇区数据 */
        rep_fx = 0.0f;
        rep_fy = 0.0f;
        danger_cnt = 0;
        safe_cnt = 0;
        noise_cnt = 0;

        for (int i = 0; i < LIDAR_SECTORS; i++) {
            /* 使用阻塞接收 (数据已在队列中, 应立即返回) */
            if (xQueueReceive(q_polar, &point, pdMS_TO_TICKS(50)) != pdTRUE) {
                ESP_LOGW(TAG, "q_polar underflow: got %d/%d points", i, LIDAR_SECTORS);
                break;
            }

            ra = point.distance_mm;
            /*  检查: 传感器数据异常（为空/无限、小于最小感知、进入噪声区）时直接丢弃，避免参与坐标转换 */
            if (isnan(ra) || isinf(ra) || ra < APF_PERCEPTION_MIN_MM || ra > APF_SAFE_RANGE_MM) {
                noise_cnt++;
                continue;
            }

            /* 极坐标 → 笛卡尔, 累积斥力 */
            /* 斥力公式: F_rep = -K_rep * w(ra) * (dx/r³, dy/r³) */
            /* - 方向: 远离障碍 = -unit(dx,dy) = -(dx/ra, dy/ra) */
            /* - 幅值: ∝ 1/r² (越近越强) */
            /* - F_rep = -K_rep * w * (dx/r³, dy/r³) */
            polar_to_cart(ra, point.angle_deg, &dx, &dy);

            /* r³ 用于 1/r² 衰减 */
            r_cubed = ra * ra * ra;
            f_rep   = APF_REPULSE_GAIN * repulse_weight(ra) / r_cubed;
            /* 斥力方向: 从障碍物指向机器人 (机器人位于原点) */
            rep_fx -= f_rep * dx;
            rep_fy -= f_rep * dy;

            if (ra <= APF_DANGER_RANGE_MM) {
                danger_cnt++;
            } else {
                safe_cnt++;
            }
        }
        // ESP_LOGI(TAG, "Polar data READY");

        /* 4. 引力 + 斥力 = 合力，引力: (K_att, 0) 恒定向前 */
        total_fx = APF_ATTRACT_GAIN + rep_fx;
        total_fy = rep_fy;  /* 引力无 y 分量 */

        force_check(total_fx, total_fy, &result);
        xQueueOverwrite(q_cart, &result);
        ESP_LOGI(TAG, "F_cmd = (%.0f,%.0f) | danger=%d safe=%d noise=%d",
                 (double)total_fx, (double)total_fy,
                 danger_cnt, safe_cnt, noise_cnt);
    }
}
