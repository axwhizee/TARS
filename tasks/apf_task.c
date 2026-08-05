/**
 * @file apf_task.c
 * @brief 人工势场法 (APF) + 360° VFH 方向选择避障实现
 * APF (人工势场法):
 *   每个障碍物产生斥力: F = -K × (r_ref / r)^n, 方向背离障碍.
 *   斥力在近距离急剧增大, 迫使小车远离墙壁.
 *
 * VFH (矢量场直方图):
 *   将 360° 划分为 72 个 bin (每 bin 5°), 障碍物距离越近 bin 值越高.
 *   扫描直方图找 "可通行通道" (连续低值 bin 段), 按宽度+正前方偏好评分.
 *   选中的通道中心角即为 VFH 引力方向.
 *
 * 合力 = APF 斥力 (斥离障碍) + VFH 引力 (驶向通道中心)
 */
#include "tasks/apf_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "APF_TASK  ";
vector_cart_t g_apf_rep;
vector_cart_t g_vfh_att;

// 参数期默认值
static const apf_params_t PARAMS_DEFAULT = {
    .range_rep     = APF_RANGE_REP,
    .gain_rep_x    = APF_GAIN_REP_X,
    .gain_rep_y    = APF_GAIN_REP_Y,
    .rep_nx        = APF_REP_NX,
    .rep_ny        = APF_REP_NY,
    .att_base      = APF_ATT_BASE,
    .vfh_thresh    = VFH_THRESH_MM,
    .vfh_min_w     = VFH_MIN_WIDTH,
    .vfh_smooth_w  = VFH_SMOOTH_W,
    .vfh_free_th   = VFH_IS_FREE_TH,
    .vfh_goal_bias = VFH_GOAL_BIAS,
    .vfh_ema_alpha = VFH_EMA_ALPHA,
};

void params_init_defaults(void) {
    g_apf_params = PARAMS_DEFAULT;
    ESP_LOGI(TAG, "APF params: reset to defaults");
}

// VFH 通道评分，通道宽度越宽、越靠近正前方 (0°), 分数越高
static inline float vfh_eval_pass(int width, float center_deg) {
    float w = (float)width;
    float goal_bias = cosf(center_deg * DEG_2_RAD);
    return w + g_apf_params.vfh_goal_bias * w * goal_bias;
}

// 势场解算: APF 斥力 + VFH 方向选择 → 合力向量
static vector_cart_t apf_compute(const vector_polar_t *samples) {
    vector_cart_t cmd = {0};
    float rep_x = 0.0f, rep_y = 0.0f;
    const float r_ref_m = g_apf_params.range_rep * 0.001f;

    // VFH 目标角度: 跨帧 EMA 平滑, 避免每帧方向跳变
    static float vfh_angle = 0.0f;

    // VFH 直方图
    float hist_raw[VFH_BINS] = {0};
    float hist[VFH_BINS] = {0};

    // 遍历 77 个采样点, 同时填充 APF 斥力和 VFH 直方图
    for (int i = 0; i < Q_POLAR_DEPTH; i++) {
        float range = samples[i].dst;
        float angle = samples[i].ang;

        // 跳过无效点: 太近 (死区) 或太远 (噪声)
        if (range < APF_RANGE_MIN || range > APF_RANGE_MAX) continue;

        float rad = angle * DEG_2_RAD;
        float ra  = range * 0.001f;     // mm → m
        float ratio = r_ref_m / ra;     // r_ref / r, >1 表示距离小于锚点

        // APF 斥力: 幂律衰减, 方向背离障碍，x、y轴独立调节
        rep_x -= g_apf_params.gain_rep_x * powf(ratio, g_apf_params.rep_nx) * cosf(rad);
        rep_y -= g_apf_params.gain_rep_y * powf(ratio, g_apf_params.rep_ny) * sinf(rad);

        // VFH: 角度 → bin 索引 (四舍五入到最近的 5° bin)
        int bin = (int)(angle / 5.0f + 0.5f) % VFH_BINS;
        if (range < g_apf_params.vfh_thresh) {
            // 距离越近, 置信度越高; 多个点在同一 bin 内累加
            hist_raw[bin] += (g_apf_params.vfh_thresh - range) / g_apf_params.vfh_thresh;
        }
    }

    // VFH 直方图平滑 — 3 点加权移动平均
    for (int i = 0; i < VFH_BINS; i++) {
        hist[i] = (hist_raw[(i - 1 + VFH_BINS) % VFH_BINS] +
                   hist_raw[i] * g_apf_params.vfh_smooth_w +
                   hist_raw[(i + 1) % VFH_BINS]) / (2.0f + g_apf_params.vfh_smooth_w);
    }

    // 360° 全向扫描可通行通道, 选最优，评分公式: score = width × (1 + goal_bias × cos(center))
    float target_deg = 0.0f;    // 最优通道中心角
    int pass_w = 0;             // 最优通道宽度 (bin 数)
    float score_best = -1e9f;   // 当前最高分
    int pass_start = -1, pass_cur = 0;

    for (int i = 0; i < VFH_BINS * 2; i++) {
        int idx = i % VFH_BINS;
        if (hist[idx] < g_apf_params.vfh_free_th) {
            // 当前 bin 可通行: 开始或延续通道
            if (pass_cur == 0) pass_start = idx;
            pass_cur++;
        } else {
            // 当前 bin 被阻断: 评估刚结束的通道
            if (pass_cur >= g_apf_params.vfh_min_w) {
                // 通道中心角 = (起始 + 半宽度) × 5°
                float center_deg = (pass_start + pass_cur * 0.5f) * 5.0f;
                if (center_deg >= 360.0f) center_deg -= 360.0f;

                float score = vfh_eval_pass(pass_cur, center_deg);
                if (score > score_best) {
                    score_best = score;
                    pass_w = pass_cur;
                    target_deg = center_deg;
                }
            }
            pass_cur = 0;   // 重置, 等待下一个通道
        }
    }
    // 处理环绕通道: 循环结束后 pass_cur > 0 表示末尾有未闭合的 free 段
    if (pass_cur >= g_apf_params.vfh_min_w) {
        float center_deg = (pass_start + pass_cur * 0.5f) * 5.0f;
        if (center_deg >= 360.0f) center_deg -= 360.0f;
        float score = vfh_eval_pass(pass_cur, center_deg);
        if (score > score_best) {
            score_best = score;
            pass_w = pass_cur;
            target_deg = center_deg;
        }
    }

    // VFH 目标角度 EMA 平滑 (环绕安全)
    float angle_diff = target_deg - vfh_angle;
    if (angle_diff > 180.0f)  angle_diff -= 360.0f;   // 环绕: 350°→10° 差 = +20°
    if (angle_diff < -180.0f) angle_diff += 360.0f;   // 环绕: 10°→350° 差 = -20°
    vfh_angle += g_apf_params.vfh_ema_alpha * angle_diff;
    if (vfh_angle >= 360.0f) vfh_angle -= 360.0f;
    if (vfh_angle < 0.0f)    vfh_angle += 360.0f;

    // APF 斥力 + VFH 引力合成
    float target_rad = vfh_angle * DEG_2_RAD;
    float att_gain = g_apf_params.att_base * (1.0f + 0.05f * pass_w);
    float att_x = att_gain * cosf(target_rad);
    float att_y = att_gain * sinf(target_rad);

    g_apf_rep = (vector_cart_t){rep_x, rep_y};
    g_vfh_att = (vector_cart_t){att_x, att_y};

    cmd.dx = rep_x + att_x;
    cmd.dy = rep_y + att_y;

    return cmd;
}

void apf_task(void *pvParameters) {
    (void)pvParameters;
    const TickType_t period = pdMS_TO_TICKS(1000 / SENSOR_FREQ + 10);
    vector_polar_t samples[Q_POLAR_DEPTH];   // 本帧传感器快照 (72 LiDAR + 5 Flame)
    vector_polar_t tmp;
    vector_cart_t  cmd;

    ESP_LOGI(TAG, "APF+VFH started: RepX=%.0f(nx=%.0f) RepY=%.0f(ny=%.0f) AttBase=%.0f GoalBias=%.2f EMA=%.2f",
        g_apf_params.gain_rep_x, g_apf_params.rep_nx, g_apf_params.gain_rep_y, g_apf_params.rep_ny,
        g_apf_params.att_base, g_apf_params.vfh_goal_bias, g_apf_params.vfh_ema_alpha);

    while (1) {
        // 等待传感器数据就绪
        EventBits_t bits = xEventGroupWaitBits(eg_sync,
            BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY, pdFALSE, pdTRUE, period);

        // 传感器超时
        if ((bits & (BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY)) != (BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY)) {
            ESP_LOGW(TAG, "Sensor sync timeout");
            /* auto 模式下发送零向量停车, 防止残留旧指令导致失控.
             * manual 模式下不写 q_cart (web_task 已在控制). */
            // 超时给出刹车指令，手动模式除外
            if (!(xEventGroupGetBits(eg_sync) & BIT_MANUAL_MODE)) {
                cmd = (vector_cart_t){0};   // C99 支持的写法，初始化结构体再赋值
                xQueueOverwrite(q_cart, &cmd);
            }
            g_apf_rep = g_vfh_att = (vector_cart_t){0};
            continue;   // 跳过本帧, 回到 Phase 1 重新等待
        }

        // 检查 log 队列积压，清空 log 队列
        if (xEventGroupGetBits(eg_sync) & BIT_LOG_Q_READY) {
            vector_polar_t drain;
            ESP_LOGW(TAG, "Log timeout: draining q_log");
            while (xQueueReceive(q_log, &drain, 0) == pdTRUE) {}
            xEventGroupClearBits(eg_sync, BIT_LOG_Q_READY);
        }

        // 读取所有的传感器数据
        for (int i = 0; i < Q_POLAR_DEPTH; i++) {
            xQueueReceive(q_polar, &tmp, 0);
            samples[i] = tmp;
            xQueueSend(q_log, &tmp, 0);
        }

        xEventGroupClearBits(eg_sync, BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY);
        xEventGroupSetBits(eg_sync, BIT_LOG_Q_READY);

        // APF+VFH 势场解算 (manual 模式也执行, 为 Web 可视化提供斥力/引力)
        cmd = apf_compute(samples);

        // 仅 auto 模式: 将合力发送给电机
        if (!(xEventGroupGetBits(eg_sync) & BIT_MANUAL_MODE)) {
            xQueueOverwrite(q_cart, &cmd);
        }
        ESP_LOGI(TAG, "F_cmd = (%.0f,%.0f)", cmd.dx, cmd.dy);
    }
}
