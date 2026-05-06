/**
 * @file apf_task.c
 * @brief 人工势场法避障实现
 *
 * 核心公式 (2D APF):
 *   引力:  F_att = (K_att, 0)              // 恒定正向拉力
 *   斥力:  F_rep = -K_rep * w(r) * (dx/r³, dy/r³)
 *          其中 w(r) = { danger_wt, r ≤ 2m; safe_wt, 2m < r ≤ 6m }
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

static const char *TAG = "APF";

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ---------------------------------------------------------- */
/* 辅助                                                         */
/* ---------------------------------------------------------- */

static inline float clampf_range(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/**
 * @brief 极坐标 → 笛卡尔坐标
 *
 * 坐标系约定 (LD14P):
 *   angle = 0° 为正前方 (机器人 x 轴正向)
 *   angle 顺时针递增 (右侧为正 y)
 */
static inline void polar_to_cart(float r, float angle_deg,
                                 float *dx, float *dy)
{
    float rad = angle_deg * (M_PI / 180.0f);
    *dx = r * cosf(rad);
    *dy = r * sinf(rad);
}

/**
 * @brief 根据距离返回斥力权重倍率
 */
static inline float repulse_weight(float r)
{
    if (r <= APF_DANGER_RANGE_MM) {
        return APF_DANGER_REPULSE_WT;   /* 危险区: 强斥力 */
    }
    /* r > 2m 且 r <= 6m → 感知区 */
    return APF_SAFE_REPULSE_WT;
}

/* ---------------------------------------------------------- */
/* 任务主体                                                     */
/* ---------------------------------------------------------- */

void apf_task(void *pvParameters)
{
    apf_task_params_t *params = (apf_task_params_t *)pvParameters;
    vector_polar_t point;
    vector_cart_t   result;

    /* 斥力累加器 */
    float rep_fx, rep_fy;

    /* 单点转换 */
    float dx, dy, r, r_cubed, f_rep;

    /* 合力 */
    float total_fx, total_fy;
    float mag;

    /* 统计 */
    int danger_cnt, safe_cnt, noise_cnt;

    ESP_LOGI(TAG, "APF task started: K_att=%.0f K_rep=%.0f danger<%.0fm safe<%.0fm",
             (double)APF_ATTRACT_GAIN, (double)APF_REPULSE_GAIN,
             (double)APF_DANGER_RANGE_MM / 1000.0,
             (double)APF_SAFE_RANGE_MM / 1000.0);

    while (1) {
        /* ============================================ */
        /* 1. 等待 LiDAR 数据就绪信号 (1s 超时)          */
        /* ============================================ */
        EventBits_t bits = xEventGroupWaitBits(
            params->eg_sync,
            BIT_LIDAR_READY,
            pdTRUE,    /* 读取后自动清零 */
            pdFALSE,   /* 任意一位即可 (这里只有 1 位) */
            pdMS_TO_TICKS(1000)
        );

        if ((bits & BIT_LIDAR_READY) == 0) {
            /* 超时: 推送零指令停止机器人 */
            result.x = 0.0f;
            result.y = 0.0f;
            xQueueOverwrite(params->q_cart, &result);
            continue;
        }

        /* ============================================ */
        /* 2. 从队列读取全部 LIDAR_SECTORS 个扇区数据    */
        /* ============================================ */
        rep_fx    = 0.0f;
        rep_fy    = 0.0f;
        danger_cnt = 0;
        safe_cnt   = 0;
        noise_cnt  = 0;

        for (int i = 0; i < LIDAR_SECTORS; i++) {
            /* 使用阻塞接收 (数据已在队列中, 应立即返回) */
            if (xQueueReceive(params->q_polar, &point, pdMS_TO_TICKS(50)) != pdTRUE) {
                ESP_LOGW(TAG, "q_polar underflow: got %d/%d points", i, LIDAR_SECTORS);
                break;
            }

            r = point.distance_mm;

            /* --- 距离分区过滤 --- */

            /* 死区 / 自身遮挡 / 地面反射 → 丢弃 */
            if (r < APF_PERCEPTION_MIN_MM) {
                noise_cnt++;
                continue;
            }

            /* 6m 以外 → 噪声 */
            if (r > APF_SAFE_RANGE_MM) {
                noise_cnt++;
                continue;
            }

            /* ============================================ */
            /* 3. 极坐标 → 笛卡尔, 累积斥力                  */
            /*                                              */
            /* 斥力公式:                                     */
            /*   F_rep = -K_rep * w(r) * (dx/r³, dy/r³)      */
            /*                                              */
            /* 推导:                                        */
            ///   方向: 远离障碍 = -unit(dx,dy) = -(dx/r, dy/r)  */
            /*   幅值: ∝ 1/r² (越近越强)                    */
            /*   ∴ F_rep = -K_rep * w * (dx/r³, dy/r³)       */
            /* ============================================ */
            polar_to_cart(r, point.angle_deg, &dx, &dy);

            /* r³ 用于 1/r² 衰减 */
            r_cubed = r * r * r;
            f_rep   = APF_REPULSE_GAIN * repulse_weight(r) / r_cubed;

            /* 斥力方向: 从障碍物指向机器人 (机器人位于原点) */
            rep_fx -= f_rep * dx;
            rep_fy -= f_rep * dy;

            if (r <= APF_DANGER_RANGE_MM) {
                danger_cnt++;
            } else {
                safe_cnt++;
            }
        }

        /* ============================================ */
        /* 4. 引力 + 斥力 = 合力                         */
        /*                                              */
        /*   引力: (K_att, 0) 恒定向前                   */
        /* ============================================ */
        total_fx = APF_ATTRACT_GAIN + rep_fx;
        total_fy = rep_fy;  /* 引力无 y 分量 */

        /* 钳位到安全范围 */
        total_fx = clampf_range(total_fx, -APF_MAX_FORCE_MM, APF_MAX_FORCE_MM);
        total_fy = clampf_range(total_fy, -APF_MAX_FORCE_MM, APF_MAX_FORCE_MM);

        /* 幅值死区: 接近零时归零, 避免微抖动 */
        mag = sqrtf(total_fx * total_fx + total_fy * total_fy);
        if (mag < APF_MIN_FORCE_MM) {
            total_fx = 0.0f;
            total_fy = 0.0f;
        }

        /* --- 幅值限幅 (保持方向) --- */
        if (mag > APF_MAX_FORCE_MM) {
            total_fx = total_fx / mag * APF_MAX_FORCE_MM;
            total_fy = total_fy / mag * APF_MAX_FORCE_MM;
        }

        /* ============================================ */
        /* 5. 将合力推入电机控制队列                      */
        /*   使用 xQueueOverwrite 防止旧数据堵塞          */
        /* ============================================ */
        result.x = total_fx;
        result.y = total_fy;

        if (xQueueOverwrite(params->q_cart, &result) != pdPASS) {
            xQueueSend(params->q_cart, &result, 0);
        }

        ESP_LOGD(TAG, "APF: F=(%.0f,%.0f) | danger=%d safe=%d noise=%d",
                 (double)total_fx, (double)total_fy,
                 danger_cnt, safe_cnt, noise_cnt);
    }
}
