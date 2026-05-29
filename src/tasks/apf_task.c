/**
 * @file apf_task.c
 * @brief 人工势场法避障实现
 *
 * 核心公式 (2D APF):
 *   斥力:  F_rep = -K_rep * w(ra) / ra² * û(θ)
 *          û = (cosθ, sinθ),  w(ra) = { danger_wt, ra ≤ 800mm; 1.0, 800 < ra ≤ 4000mm }
 *   引力:  F_att = (K_att, 0)
 *   合力:  F_total = F_att + Σ F_rep
 *   单位:  距离 mm, 合力无量纲 (电机任务归一化后转为占空比)
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
 * @brief 根据距离返回斥力权重倍率
 */
static inline float repulse_weight(float distance) {
    if (distance <= APF_DANGER_RANGE) {
        return APF_DANGER_RE_WT;   // 危险区：斥力加权
    }
    return 1.0f;    // 感知区：权重不变
}

void apf_task(void *pvParameters) {
    (void)pvParameters;
    const TickType_t period = pdMS_TO_TICKS(1000 / SENSOR_FREQ + 10);    // 等待周期（+10ms 余量）
    vector_polar_t  samples[Q_POLAR_DEPTH];  // 批量读取缓冲区: 72 lidar + 5 flame
    vector_polar_t  tmp;
    vector_cart_t   cmd;                     // 合力指令输出 (→ q_cart → motor_task)
    float rfx, rfy;                          // 斥力分量累计
    int n_danger, n_safe, n_noise;           // 各区间点数统计

    ESP_LOGI(TAG, "APF task started: K_att=%.0f K_rep=%.0f danger<%.0fmm safe<%.0fmm",
        APF_ATTRACT_GAIN, APF_REPULSE_GAIN, APF_DANGER_RANGE, APF_SAFE_RANGE);

    while (1) {
        // 阻塞等待事件组就绪（超时匹配传感器周期）
        EventBits_t bits = xEventGroupWaitBits(eg_sync, BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY,
            pdFALSE, pdTRUE, period);
        if ((bits & (BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY)) != (BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY)) {
            ESP_LOGW(TAG, "Sensor sync timeout");   // 传感器数据等待超时
            cmd.dx = cmd.dy = 0.0f;
            xQueueOverwrite(q_cart, &cmd);   // 空指令滑行
            g_cart_cmd = cmd;
            continue;
        }
        if (xEventGroupGetBits(eg_sync) & BIT_LOG_Q_READY) {
            vector_polar_t drain;
            ESP_LOGW(TAG, "Log timeout: draining q_log");
            while (xQueueReceive(q_log, &drain, 0) == pdTRUE) {}
            xEventGroupClearBits(eg_sync, BIT_LOG_Q_READY);
        }

        for (int i = 0; i < Q_POLAR_DEPTH; i++) {
            xQueueReceive(q_polar, &tmp, 0);  // 读取传感器数据队列
            samples[i] = tmp;
            xQueueSend(q_log, &tmp, 0);   // 透传到日志队列
        }

        // 清除传感器位, 通知日志任务
        xEventGroupClearBits(eg_sync, BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY);
        xEventGroupSetBits(eg_sync, BIT_LOG_Q_READY);

        n_noise = 0;
        n_danger = 0;
        n_safe = 0;
        rfx = 0.0f;
        rfy = 0.0f;

        for (int i = 0; i < Q_POLAR_DEPTH; i++) {
            float range = samples[i].distance;   // 距离 (mm), 用于分类和权重

            if (range < APF_PERCEPTION_MIN || range > APF_SAFE_RANGE) {
                n_noise++;      // 噪声/死区, 跳过不计
                continue;
            } else if (range <= APF_DANGER_RANGE) {
                n_danger++;     // 危险区
            } else {
                n_safe++;       // 感知区
            }

            float rad   = samples[i].angle * (M_PI / 180.0f);   // ° → rad
            float ra    = range * 0.001f;   // mm → m, 防止 ra² 溢出
            float ra_sq = ra * ra;  // 临时改为三次
            // F_rep = -K_rep * w(range) / ra² * û(θ) , û = (cosθ, sinθ)
            float f_rep = APF_REPULSE_GAIN * repulse_weight(range) / ra_sq;
            rfx -= f_rep * cosf(rad);   // x 分量 (负号: 力背离障碍物)
            rfy -= f_rep * sinf(rad);   // y 分量
        }

        // F_total = F_att + Σ F_rep (引力沿 x 轴正向, 驱动机器人前行)
        cmd.dx = rfx + APF_ATTRACT_GAIN;
        cmd.dy = rfy;
        g_cart_cmd = cmd;   // 复制一份供日志更新

        xQueueOverwrite(q_cart, &cmd);
        // 添加换行，表示一次完整任务流程结束
        ESP_LOGI(TAG, "F_cmd = (%.0f,%.0f) | danger=%d safe=%d noise=%d",
            cmd.dx, cmd.dy, n_danger, n_safe, n_noise);
    }
}
