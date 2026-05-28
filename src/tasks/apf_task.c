/**
 * @file apf_task.c
 * @brief 人工势场法避障实现
 *
 * 核心公式 (2D APF):
 *   引力:  F_att = (K_att, 0)              // 恒定正向拉力
 *   斥力:  F_rep = -K_rep * w(ra) * (dx/r³, dy/r³)
 *          其中 w(ra) = { danger_wt, ra ≤ 2m; safe_wt, 2m < ra ≤ 6m }
 *   合力:  F_total = F_att + Σ F_rep       // 钳位到 ±6m
 */
#include "tasks/apf_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "APF_TASK  ";
vector_cart_t g_cart_cmd;

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 极坐标 → 笛卡尔坐标
 */
static inline void polar_to_cart(vector_polar_t *polar, vector_cart_t *cart) {
    float rad = polar->angle_deg * (M_PI / 180.0f);
    cart->dx = polar->distance_mm * cosf(rad);
    cart->dy = polar->distance_mm * sinf(rad);
}

/**
 * @brief 根据距离返回斥力权重倍率
 */
static inline float repulse_weight(float distance) {
    if (distance <= APF_DANGER_RANGE) {
        return APF_DANGER_RE_WT;   // 危险区：斥力加权
    }
    return APF_SAFE_REP_WT;     // 感知区：低斥低权
}

void apf_task(void *pvParameters) {
    (void)pvParameters;
    const TickType_t period = pdMS_TO_TICKS(1000 / SENSOR_FREQ + 10);    // 等待周期（+10ms 余量）
    vector_polar_t  batch[Q_POLAR_DEPTH];  // 批量读取缓冲区
    vector_polar_t  dummy, point;
    vector_cart_t   result;
    float ra, r_cubed, rep_fx, rep_fy, f_rep;   // APF 计算过程量
    int danger_cnt, safe_cnt, noise_cnt;

    ESP_LOGI(TAG, "APF task started: K_att=%.0f K_rep=%.0f danger<%.0fmm safe<%.0fmm",
        APF_ATTRACT_GAIN, APF_REPULSE_GAIN, APF_DANGER_RANGE, APF_SAFE_RANGE);

    while (1) {
        // 阻塞等待事件组就绪（超时匹配传感器周期）
        EventBits_t bits = xEventGroupWaitBits(eg_sync, BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY,
            pdFALSE, pdTRUE, period);

        if ((bits & (BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY)) != (BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY)) {
            ESP_LOGW(TAG, "Sensor sync timeout");   // 传感器数据等待超时
            result.dx = result.dy = 0.0f;
            xQueueOverwrite(q_cart, &result);   // 空指令滑行
            g_cart_cmd = result;
            continue;
        }
        if (xEventGroupGetBits(eg_sync) & BIT_LOG_Q_READY) {
            ESP_LOGW(TAG, "Log timeout: draining q_log");   // 日志发送超时（日志队列未清空）
            while (xQueueReceive(q_log, &dummy, 0) == pdTRUE) {}    // 清空日志队列与事件位
            xEventGroupClearBits(eg_sync, BIT_LOG_Q_READY);
        }

        for (int i = 0; i < Q_POLAR_DEPTH; i++) {
            xQueueReceive(q_polar, &point, 0);  // 读取传感器数据队列
            batch[i] = point;
            xQueueSend(q_log, &point, 0);   // 透传到日志队列
        }

        // 清除传感器位, 通知日志任务
        xEventGroupClearBits(eg_sync, BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY);
        xEventGroupSetBits(eg_sync, BIT_LOG_Q_READY);

        noise_cnt = 0;
        danger_cnt = 0;
        safe_cnt = 0;
        rep_fx = 0.0f;
        rep_fy = 0.0f;
        for (int i = 0; i < Q_POLAR_DEPTH; i++) {   // 批量计算 APF
            point = batch[i];
            ra = point.distance_mm;

            if (isnan(ra) || isinf(ra) || ra < APF_PERCEPTION_MIN || ra > APF_SAFE_RANGE) {
                noise_cnt++;    // 噪声（空/无穷/死区/噪声区）数据，跳过
                continue;
            } else if (ra <= APF_DANGER_RANGE) {
                danger_cnt++;   // 危险区数据
            } else {
                safe_cnt++;     // 感知区数据
            }

            polar_to_cart(&point, &result);     // 转换为笛卡尔坐标
            // 斥力rep = gain*weight(ra)*dxy/ra^3
            r_cubed = ra * ra * ra;
            f_rep = APF_REPULSE_GAIN * repulse_weight(ra) / r_cubed;
            rep_fx -= f_rep * result.dx;
            rep_fy -= f_rep * result.dy;
        }
        // 不对合力数据作检查，留给电机任务处理
        result.dx = APF_ATTRACT_GAIN + rep_fx;  // 添加前向行进引力
        result.dy = rep_fy;

        xQueueOverwrite(q_cart, &result);
        g_cart_cmd = result;    // 复制一份供日志更新
        // 添加换行，表示一次完整任务流程结束
        ESP_LOGI(TAG, "F_cmd = (%.0f,%.0f) | danger=%d safe=%d noise=%d",
            result.dx, result.dy, danger_cnt, safe_cnt, noise_cnt);
    }
}
