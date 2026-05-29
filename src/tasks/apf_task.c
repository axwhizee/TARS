/**
 * @file apf_task.c
 * @brief 人工势场法避障实现
 *
 * 斥力:  F_rep = -K_rep * w(ra) / ra² * û(θ)
 *        û = (cosθ, sinθ)
 *        w(ra) = { APF_DANGER_RE_WT,  ra ≤ APF_DANGER_RANGE
 *                { APF_SAFE_RE_WT,    ra ≤ APF_SAFE_RANGE
 * 引力:  F_att = (K_att, 0)  +  F_open (前方 120° 开阔方向)
 *        x 分量 = 1/r² 控后退时机, y 分量 = 1/r 均化转向力
 * 单位: 距离 mm, 合力无量纲 (电机任务归一化后转为占空比)
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
        return APF_DANGER_RE_WT;   // 危险区权重
    }
    return APF_SAFE_RE_WT;  // 感知区权重
}

void apf_task(void *pvParameters) {
    (void)pvParameters;
    const TickType_t period = pdMS_TO_TICKS(1000 / SENSOR_FREQ + 10);    // 等待周期（+10ms 余量）
    vector_polar_t  samples[Q_POLAR_DEPTH];
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
        float max_range = 0.0f;     // 追加引导矢量
        float open_angle = 0.0f;

        for (int i = 0; i < Q_POLAR_DEPTH; i++) {
            float range = samples[i].distance;
            float angle = samples[i].angle;

            // 查找前方 120° 最大距离，用于指示开阔方向
            if (range > max_range && (angle <= 60.0f || angle >= 300.0f)) {
                max_range = range;
                open_angle = angle;
            }

            if (range < APF_PERCEPTION_MIN || range > APF_SAFE_RANGE) {
                n_noise++;      // 噪声/死区, 跳过
                continue;
            } else if (range <= APF_DANGER_RANGE) {
                n_danger++;     // 危险区
            } else {
                n_safe++;       // 感知区
            }

            float rad = angle * (M_PI / 180.0f);
            float ra = range * 0.001f;  // mm → m
            float rep = APF_REPULSE_GAIN * repulse_weight(range);
            rfx -= (rep / (ra * ra)) * cosf(rad);   // 1/r²: 切向采用平方反比
            rfy -= (rep / ra) * sinf(rad);          // 1/r : 垂向采用线性反比
        }

        cmd.dx = rfx + APF_ATTRACT_GAIN;
        cmd.dy = rfy;
        if (max_range > APF_PERCEPTION_MIN) {
            float open_rad = open_angle * (M_PI / 180.0f);
            cmd.dx += APF_OPEN_GAIN * cosf(open_rad);   // 合成追加引导矢量
            cmd.dy += APF_OPEN_GAIN * sinf(open_rad);
        }
        g_cart_cmd = cmd;   // 复制一份供日志更新

        xQueueOverwrite(q_cart, &cmd);
        ESP_LOGI(TAG, "F_cmd = (%.0f,%.0f) | danger=%d safe=%d noise=%d",
            cmd.dx, cmd.dy, n_danger, n_safe, n_noise);
    }
}
