/**
 * @file apf_task.c
 * @brief 人工势场法 (APF) + 360° VFH 方向选择避障实现
 *
 * 流程:
 *   1. 采集传感器 → APF 斥力 + VFH 直方图 (360° 全向)
 *   2. 平滑直方图 → 扫描可通行通道 → 评分选最优 (宽度 + 正前方偏好)
 *   3. 角度 EMA 平滑 → 合成: APF 斥力 + VFH 引力 (方向由 VFH 决定)
 *
 * g_cart_cmd = 完整合力 (APF + VFH), 供 Web 可视化调试.
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
#define M_PI 3.1416f
#endif
#define M_PI_RAD M_PI / 180.0f

/**
 * @brief 势场解算
 * @param samples 极坐标传感器数据 (77 点: 72 LiDAR + 5 Flame)
 * @return 笛卡尔合力指令 {dx, dy}
 */
static vector_cart_t apf_compute(const vector_polar_t *samples) {
    vector_cart_t cmd = {0};
    float rep_x = 0.0f, rep_y = 0.0f;
    const float r_ref_m = APF_RANGE_REP * 0.001f;

    /* EMA 平滑后的 VFH 目标角度 (跨帧保持, 初始化为正前方) */
    static float vfh_angle = 0.0f;

    float hist_raw[VFH_BINS] = {0};
    float hist[VFH_BINS] = {0};

    /* 1. 遍历采样: APF 斥力 + VFH 直方图填充 */
    for (int i = 0; i < Q_POLAR_DEPTH; i++) {
        float range = samples[i].distance;
        float angle = samples[i].angle;
        if (range < APF_RANGE_MIN || range > APF_RANGE_MAX) continue;

        float rad = angle * M_PI_RAD;
        float ra  = range * 0.001f;
        float ratio = r_ref_m / ra;

        /* APF 斥力 (连续幂律, 无跳变) */
        rep_x -= APF_GAIN_REP_X * powf(ratio, APF_REP_NX) * cosf(rad);
        rep_y -= APF_GAIN_REP_Y * powf(ratio, APF_REP_NY) * sinf(rad);

        /* VFH 直方图: 距离越近, 障碍置信度越高 [0~1] */
        int bin = (int)(angle / 5.0f + 0.5f) % VFH_BINS;
        if (range < VFH_THRESH_MM) {
            hist_raw[bin] += (VFH_THRESH_MM - range) / VFH_THRESH_MM;
        }
    }

    /* 2. 直方图平滑 (3 点加权移动平均, 消除单点噪波) */
    for (int i = 0; i < VFH_BINS; i++) {
        hist[i] = (hist_raw[(i - 1 + VFH_BINS) % VFH_BINS] +
                   hist_raw[i] * VFH_SMOOTH_W +
                   hist_raw[(i + 1) % VFH_BINS]) / (2.0f + VFH_SMOOTH_W);
    }

    /* 3. 360° 全向扫描可通行通道, 选宽度+正前方偏好综合最优 */
    float target_deg = 0.0f;
    int pass_w = 0;
    float score_best = -1e9f;
    int pass_start = -1, pass_cur = 0;

    for (int i = 0; i < VFH_BINS * 2; i++) {
        int idx = i % VFH_BINS;
        if (hist[idx] < VFH_IS_FREE_TH) {
            if (pass_cur == 0) pass_start = idx;
            pass_cur++;
        } else {
            if (pass_cur >= VFH_MIN_WIDTH) {
                float center_deg = (pass_start + pass_cur * 0.5f) * 5.0f;
                if (center_deg >= 360.0f) center_deg -= 360.0f;

                float w = (float)pass_cur;
                /* 宽度 + 正前方偏好: cos(0°)=+1, cos(180°)=-1 */
                float goal_bias = cosf(center_deg * M_PI_RAD);
                float score = w + VFH_GOAL_BIAS * w * goal_bias;

                if (score > score_best) {
                    score_best = score;
                    pass_w = pass_cur;
                    target_deg = center_deg;
                }
            }
            pass_cur = 0;
        }
    }
    /* 处理环绕通道 (末尾 free 段与开头 free 段连通) */
    if (pass_cur >= VFH_MIN_WIDTH) {
        float center_deg = (pass_start + pass_cur * 0.5f) * 5.0f;
        if (center_deg >= 360.0f) center_deg -= 360.0f;
        float w = (float)pass_cur;
        float goal_bias = cosf(center_deg * M_PI_RAD);
        float score = w + VFH_GOAL_BIAS * w * goal_bias;
        if (score > score_best) {
            score_best = score;
            pass_w = pass_cur;
            target_deg = center_deg;
        }
    }

    /* 4. 角度 EMA 平滑 (环绕安全) */
    float angle_diff = target_deg - vfh_angle;
    if (angle_diff > 180.0f)  angle_diff -= 360.0f;
    if (angle_diff < -180.0f) angle_diff += 360.0f;
    vfh_angle += VFH_EMA_ALPHA * angle_diff;
    if (vfh_angle >= 360.0f) vfh_angle -= 360.0f;
    if (vfh_angle < 0.0f)    vfh_angle += 360.0f;

    /* 5. 合成: APF 斥力 + VFH 引力 (方向由 VFH 决定, 幅值随通道宽度缩放) */
    float target_rad = vfh_angle * M_PI_RAD;
    float att_gain = APF_ATT_BASE * (1.0f + 0.05f * pass_w);
    float att_x = att_gain * cosf(target_rad);
    float att_y = att_gain * sinf(target_rad);
    cmd.dx = rep_x + att_x;
    cmd.dy = rep_y + att_y;

    /* 可视化: 完整合力 (APF 斥力 + VFH 引力) */
    g_cart_cmd.dx = att_x;
    g_cart_cmd.dy = att_y;

    return cmd;
}

void apf_task(void *pvParameters) {
    (void)pvParameters;
    const TickType_t period = pdMS_TO_TICKS(1000 / SENSOR_FREQ + 10);
    vector_polar_t samples[Q_POLAR_DEPTH];
    vector_polar_t tmp;
    vector_cart_t  cmd;

    ESP_LOGI(TAG, "APF+VFH started: RepX=%.0f(nx=%.0f) RepY=%.0f(ny=%.0f) AttBase=%.0f GoalBias=%.2f EMA=%.2f",
        APF_GAIN_REP_X, APF_REP_NX,
        APF_GAIN_REP_Y, APF_REP_NY,
        APF_ATT_BASE, VFH_GOAL_BIAS, VFH_EMA_ALPHA);

    while (1) {
        EventBits_t bits = xEventGroupWaitBits(eg_sync,
            BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY, pdFALSE, pdTRUE, period);
        if ((bits & (BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY))
                != (BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY)) {
            if (!(xEventGroupGetBits(eg_sync) & BIT_MANUAL_MODE)) {
                ESP_LOGW(TAG, "Sensor sync timeout");
                cmd.dx = cmd.dy = 0.0f;
                xQueueOverwrite(q_cart, &cmd);
                g_cart_cmd = cmd;
            }
            continue;
        }
        if (xEventGroupGetBits(eg_sync) & BIT_LOG_Q_READY) {
            vector_polar_t drain;
            ESP_LOGW(TAG, "Log timeout: draining q_log");
            while (xQueueReceive(q_log, &drain, 0) == pdTRUE) {}
            xEventGroupClearBits(eg_sync, BIT_LOG_Q_READY);
        }

        for (int i = 0; i < Q_POLAR_DEPTH; i++) {
            xQueueReceive(q_polar, &tmp, 0);
            samples[i] = tmp;
            xQueueSend(q_log, &tmp, 0);
        }

        xEventGroupClearBits(eg_sync, BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY);
        xEventGroupSetBits(eg_sync, BIT_LOG_Q_READY);

        if (xEventGroupGetBits(eg_sync) & BIT_MANUAL_MODE) continue;

        cmd = apf_compute(samples);
        xQueueOverwrite(q_cart, &cmd);
        ESP_LOGI(TAG, "F_cmd = (%.0f,%.0f)", cmd.dx, cmd.dy);
    }
}
