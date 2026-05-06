/**
 * @file motor_task.c
 * @brief 电机控制任务: EMA 低通滤波 + 笛卡尔→差速转换 + 状态机 + 换向死区
 *
 * 数据流:
 *   APF 任务 → q_cart(vector_cart_t) → 本任务
 *     → EMA 滤波 → 状态分类 → 差速解算 → 死区管理 → motor_set()
 *
 * 状态机 (5 态):
 *   IDLE → FORWARD / REVERSE / FORWARD_TURN / REVERSE_TURN
 *   换向反转 (前→后 / 后→前) 时插入 MOTOR_DEADTIME_MS 制动保持
 */

#include "tasks/motor_task.h"
#include "drivers/motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MOTOR_CTL";

/* ---------------------------------------------------------- */
/* 辅助函数                                                     */
/* ---------------------------------------------------------- */

/**
 * @brief 钳位浮点值到 [lo, hi] 范围
 */
static inline float clampf(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/**
 * @brief EMA 低通滤波更新
 *
 * y[n] = α·x[n] + (1-α)·y[n-1]
 * α 越小 → 越平滑但响应越慢
 * α=0.3 → 约 3 个周期达到 63% 稳态值
 */
static inline float ema_update(float *state, float input, float alpha)
{
    *state = alpha * input + (1.0f - alpha) * (*state);
    return *state;
}

/**
 * @brief 根据滤波后的笛卡尔分量判定当前状态
 *
 * 分类逻辑:
 *   |x|<DZ 且 |y|<DZ          → IDLE       (死区)
 *   |y| < TURN_THRES * |x|   → FORWARD / REVERSE (近似直线)
 *   |y| >= TURN_THRES * |x|  → FORWARD_TURN / REVERSE_TURN (转向)
 *
 * 符号: x>0 为前进方向 (FORWARD), x<0 为后退方向 (REVERSE)
 */
static motor_state_t classify_state(float x, float y)
{
    float ax = fabsf(x);
    float ay = fabsf(y);

    /* 死区内 → IDLE */
    if (ax < MOTOR_DEADZONE_MM && ay < MOTOR_DEADZONE_MM) {
        return MOTOR_STATE_IDLE;
    }

    /* 纯旋转 (x≈0, y≠0): 归为转向, x=0 视为正向 (避免除零) */
    if (ax < MOTOR_DEADZONE_MM) {
        return (y > 0.0f) ? MOTOR_STATE_FORWARD_TURN
                          : MOTOR_STATE_REVERSE_TURN;
    }

    /* 直线 vs 转向判定 */
    if (ay < MOTOR_TURN_THRESHOLD * ax) {
        /* 近似直线运动 */
        return (x > 0.0f) ? MOTOR_STATE_FORWARD : MOTOR_STATE_REVERSE;
    } else {
        /* 差速转向 */
        return (x > 0.0f) ? MOTOR_STATE_FORWARD_TURN : MOTOR_STATE_REVERSE_TURN;
    }
}

/**
 * @brief 判断状态迁移是否发生 "前进<->后退" 方向反转
 *
 * 方向反转 → 需要插入制动死区保护 DRV8833
 * 同一方向内的状态切换 (如 FORWARD→FORWARD_TURN) 不需要死区
 */
static bool is_direction_reversal(motor_state_t prev, motor_state_t next)
{
    if (prev == next || prev == MOTOR_STATE_IDLE || next == MOTOR_STATE_IDLE) {
        return false;
    }

    /* 前进类状态: FORWARD / FORWARD_TURN */
    bool prev_fwd = (prev == MOTOR_STATE_FORWARD || prev == MOTOR_STATE_FORWARD_TURN);
    bool next_fwd = (next == MOTOR_STATE_FORWARD || next == MOTOR_STATE_FORWARD_TURN);

    return (prev_fwd != next_fwd);
}

/* ---------------------------------------------------------- */
/* 任务主体                                                     */
/* ---------------------------------------------------------- */

void motor_task(void *pvParameters)
{
    motor_task_params_t *params = (motor_task_params_t *)pvParameters;
    vector_cart_t cmd;

    /* --- EMA 滤波器状态 --- */
    float ema_x = 0.0f;
    float ema_y = 0.0f;

    /* --- 当前运行状态 --- */
    motor_state_t prev_state = MOTOR_STATE_IDLE;
    motor_state_t new_state;

    /* --- PWM 速度值 --- */
    float linear, angular;
    float left_spd, right_spd;
    int16_t left_pwm, right_pwm;
    float max_spd;

    ESP_LOGI(TAG, "Motor control task started, alpha=%.2f deadzone=%.0fmm",
             (double)MOTOR_EMA_ALPHA, (double)MOTOR_DEADZONE_MM);

    while (1) {
        /* ============================================ */
        /* 1. 等待上游指令 (带 100ms 超时)              */
        /* ============================================ */
        if (xQueueReceive(params->q_cart, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            /* 收到新指令 → EMA 滤波平滑 */
            ema_x = ema_update(&ema_x, cmd.x, MOTOR_EMA_ALPHA);
            ema_y = ema_update(&ema_y, cmd.y, MOTOR_EMA_ALPHA);
        } else {
            /* 上游断流 (超时) → 向零衰减, 自然停车 */
            ema_x *= MOTOR_DECAY_FACTOR;
            ema_y *= MOTOR_DECAY_FACTOR;

            /* 接近零点时直接清零，避免无限微小残留 */
            if (fabsf(ema_x) < (MOTOR_DEADZONE_MM * 0.5f)) ema_x = 0.0f;
            if (fabsf(ema_y) < (MOTOR_DEADZONE_MM * 0.5f)) ema_y = 0.0f;
        }

        /* ============================================ */
        /* 2. 钳位到有效范围 (±6m)                      */
        /* ============================================ */
        ema_x = clampf(ema_x, -6000.0f, 6000.0f);
        ema_y = clampf(ema_y, -6000.0f, 6000.0f);

        /* ============================================ */
        /* 3. 状态分类                                   */
        /* ============================================ */
        new_state = classify_state(ema_x, ema_y);

        /* ============================================ */
        /* 4. 换向死区保护                               */
        /*   前进→后退 或 后退→前进 → 先制动再延迟      */
        /* ============================================ */
        if (is_direction_reversal(prev_state, new_state)) {
            ESP_LOGD(TAG, "Direction reversal: %d -> %d, dead-time %dms",
                     (int)prev_state, (int)new_state, MOTOR_DEADTIME_MS);
            motor_brake();
            vTaskDelay(pdMS_TO_TICKS(MOTOR_DEADTIME_MS));
        }

        /* ============================================ */
        /* 5. 笛卡尔 → 差速转换                          */
        /*                                              */
        /*   归一化到 [-1, 1]:                           */
        /*     linear  = x / 6000                        */
        /*     angular = y / 6000 * TURN_RATIO           */
        /*                                              */
        /*   差速公式:                                   */
        /*     left  = linear - angular                  */
        /*     right = linear + angular                  */
        /*                                              */
        /*   比例保持归一化: 若任一侧超出 [-1,1],         */
        /*   同时缩小两侧以保持转向比例                   */
        /* ============================================ */
        linear  = ema_x / 6000.0f;
        angular = ema_y / 6000.0f * MOTOR_TURN_RATIO;

        left_spd  = linear - angular;
        right_spd = linear + angular;

        /* 比例归一化: max(|L|, |R|) > 1 → 等比例缩放 */
        max_spd = fabsf(left_spd);
        if (fabsf(right_spd) > max_spd) max_spd = fabsf(right_spd);
        if (max_spd > 1.0f) {
            left_spd  /= max_spd;
            right_spd /= max_spd;
        }

        /* IDLE 状态强制归零 */
        if (new_state == MOTOR_STATE_IDLE) {
            left_spd  = 0.0f;
            right_spd = 0.0f;
        }

        /* ============================================ */
        /* 6. 转换为 PWM 占空比并发送                    */
        /* ============================================ */
        left_pwm  = (int16_t)(left_spd  * MOTOR_MAX_DUTY);
        right_pwm = (int16_t)(right_spd * MOTOR_MAX_DUTY);

        /* 驱动层二次钳位 (安全检查) */
        motor_set(left_pwm, right_pwm);

        prev_state = new_state;
    }
}
