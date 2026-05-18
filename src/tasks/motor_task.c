/**
 * @file motor_task.c
 * @brief 电机控制任务: EMA 插值平滑 + 笛卡尔→差速转换 + 状态机 + 换向死区
 *
 * 架构:
 *   以固定 8Hz (125ms) 周期运行, 配合 4Hz 传感器实现帧间 EMA 插值平滑.
 *
 *   主循环:
 *     q_cart ──非阻塞接收──▶ motor_cmd_to_ema()     → ema_x/y
 *                              motor_clamp_classify()  → state
 *                              motor_diff_to_pwm()     → left/right pwm
 *                              motor_set()
 *
 *   三大 static 函数:
 *     1. motor_cmd_to_ema()     — 指令→EMA 滤波 (含目标衰减)
 *     2. motor_clamp_classify() — 钳位 + 5 态分类
 *     3. motor_diff_to_pwm()    — 笛卡尔→差速 PWM
 */
#include "tasks/motor_task.h"
#include "drivers/motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <math.h>


static const char *TAG = "MOTOR_TASK";

// EMA滤波器状态结构
typedef struct {
    float ema_x, ema_y;      // 当前滤波值
    float target_x, target_y; // 目标值（指令或零）
    TickType_t last_update;   // 最后收到指令的时刻
} ema_filter_t;

/**
 * @brief 将上游 raw cmd 注入 EMA 滤波器, 输出平滑后的笛卡尔分量
 *
 * @param cmd       新指令指针 (NULL 表示本周期无新指令)
 * @param ema_x     [in/out] x 分量 EMA 状态
 * @param ema_y     [in/out] y 分量 EMA 状态
 * @param target_x  [in/out] EMA 目标 (有指令时更新, 超时时归零)
 * @param target_y  [in/out] 同上
 * @param last_tick [in/out] 最后收到指令的时刻 (tick count)
 *
 *  有 cmd → 更新 target = cmd.xy, 记录时间戳
 *  无 cmd 且距上次 > 1s → target → 0 (安全停车)
 *  每周期: ema = α·target + (1-α)·ema
 *  钳位到 ±6m
 */
static void motor_cmd_to_ema(const vector_cart_t *cmd,
    float *ema_x, float *ema_y,
    float *target_x, float *target_y,
    TickType_t *last_tick
) {
    TickType_t now = xTaskGetTickCount();

    if (cmd != NULL) {
        *target_x = cmd->x;
        *target_y = cmd->y;
        *last_tick = now;
    }

    /* 上游断流超过超时 → 目标归零 (EMA 将自然趋近零) */
    if ((now - *last_tick) > pdMS_TO_TICKS(MOTOR_INPUT_TIMEOUT_MS)) {
        *target_x = 0.0f;
        *target_y = 0.0f;
    }

    /* EMA 低通: y[n] = α·x[n] + (1-α)·y[n-1] */
    *ema_x = MOTOR_EMA_ALPHA * (*target_x) + (1.0f - MOTOR_EMA_ALPHA) * (*ema_x);
    *ema_y = MOTOR_EMA_ALPHA * (*target_y) + (1.0f - MOTOR_EMA_ALPHA) * (*ema_y);

    /* 钳位到有效范围 */
    if (*ema_x > 6000.0f) *ema_x = 6000.0f;
    if (*ema_x < -6000.0f) *ema_x = -6000.0f;
    if (*ema_y > 6000.0f) *ema_y = 6000.0f;
    if (*ema_y < -6000.0f) *ema_y = -6000.0f;
}

/**
 * @brief 根据滤波后的笛卡尔分量判定电机运行状态
 *
 * 分类逻辑:
 *   |x|<DZ 且 |y|<DZ         → IDLE
 *   |y| < TURN_THRES * |x|   → FORWARD / REVERSE (近似直线)
 *   |y| >= TURN_THRES * |x|  → FORWARD_TURN / REVERSE_TURN
 *
 * @param x  EMA 滤波后的 x 分量 (mm), >0 前进 / <0 后退
 * @param y  EMA 滤波后的 y 分量 (mm), >0 右转 / <0 左转
 * @return motor_state_t
 */
static motor_state_t motor_clamp_classify(float x, float y) {
    float ax = fabsf(x);
    float ay = fabsf(y);

    if (ax < MOTOR_DEADZONE_MM && ay < MOTOR_DEADZONE_MM) {
        return MOTOR_STATE_IDLE;
    }

    /* x≈0 但 y≠0: 纯原地旋转 */
    if (ax < MOTOR_DEADZONE_MM) {
        return (y > 0.0f) ? MOTOR_STATE_FORWARD_TURN
            : MOTOR_STATE_REVERSE_TURN;
    }

    if (ay < MOTOR_TURN_THRESHOLD * ax) {
        return (x > 0.0f) ? MOTOR_STATE_FORWARD : MOTOR_STATE_REVERSE;
    } else {
        return (x > 0.0f) ? MOTOR_STATE_FORWARD_TURN : MOTOR_STATE_REVERSE_TURN;
    }
}

/**
 * @brief 将笛卡尔合力 (x, y) 转换为左右轮 PWM 占空比
 *
 * 算法:
 *   1. 归一化: linear = x/6000, angular = y/6000 * TURN_RATIO
 *   2. 差速:   left = linear - angular, right = linear + angular
 *   3. 比例保持: 若任一侧超出 [-1,1], 等比例缩放两侧
 *   4. IDLE 状态强制归零
 *   5. PWM: pwm = speed * 100 (%)
 *
 * @param x         EMA 滤波后的 x (mm)
 * @param y         EMA 滤波后的 y (mm)
 * @param state     当前电机状态
 * @param left_duty  [out] 左轮 PWM 占空比
 * @param right_duty [out] 右轮 PWM 占空比
 */
static void motor_diff_to_pwm(float x, float y, motor_state_t state,
    int8_t *left_duty, int8_t *right_duty) {
    float linear = x / 6000.0f;
    float angular = y / 6000.0f * MOTOR_TURN_RATIO;

    float left = linear - angular;
    float right = linear + angular;

    /* 比例归一化: 保持转向比, 不超出 [-1,1] */
    float m = fabsf(left);
    if (fabsf(right) > m) m = fabsf(right);
    if (m > 1.0f) {
        left /= m;
        right /= m;
    }

    if (state == MOTOR_STATE_IDLE) {
        left = 0.0f;
        right = 0.0f;
    }

    *left_duty = (int8_t)(left * 100.0f);
    *right_duty = (int8_t)(right * 100.0f);
}

/* 换向死区判断 (工具) */
static bool is_direction_reversal(motor_state_t prev, motor_state_t next) {
    if (prev == next || prev == MOTOR_STATE_IDLE || next == MOTOR_STATE_IDLE) {
        return false;
    }

    bool prev_fwd = (prev == MOTOR_STATE_FORWARD || prev == MOTOR_STATE_FORWARD_TURN);
    bool next_fwd = (next == MOTOR_STATE_FORWARD || next == MOTOR_STATE_FORWARD_TURN);

    return (prev_fwd != next_fwd);
}

void motor_task(void *pvParameters) {
    (void)pvParameters;

    /* EMA 滤波器状态 */
    float ema_x = 0.0f, ema_y = 0.0f;
    float tgt_x = 0.0f, tgt_y = 0.0f;
    TickType_t last_cmd_tick = 0;

    /* 状态机 */
    motor_state_t prev_state = MOTOR_STATE_IDLE;
    motor_state_t new_state;
    int8_t left_duty, right_duty;
    TickType_t wake_time = xTaskGetTickCount();     // 固定周期调度

    ESP_LOGI(TAG, "Motor task started @ %dHz (period=%ums), alpha=%.3f tau=%.0fms",
        MOTOR_CTL_HZ, MOTOR_CTL_PERIOD_MS,
        (double)MOTOR_EMA_ALPHA, (double)MOTOR_EMA_TC_MS);

    while (1) {
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(MOTOR_CTL_PERIOD_MS));

        /* ---- 非阻塞取指令 ---- */
        vector_cart_t cmd;
        const vector_cart_t *pcmd = NULL;
        if (xQueueReceive(q_cart, &cmd, 0) == pdTRUE) {
            ESP_LOGD(TAG, "Vector_cart RECEIVED (%.0f, %.0f)", (double)cmd.x, (double)cmd.y);
            pcmd = &cmd;
        }

        /* ---- Step 1: 指令→EMA 滤波 ---- */
        motor_cmd_to_ema(pcmd, &ema_x, &ema_y, &tgt_x, &tgt_y, &last_cmd_tick);

        /* ---- Step 2: 钳位 + 状态分类 ---- */
        new_state = motor_clamp_classify(ema_x, ema_y);

        /* ---- Step 3: 换向死区保护 ---- */
        if (is_direction_reversal(prev_state, new_state)) {
            // ESP_LOGD(TAG, "Reversal %d→%d, dead-time %dms",
            //          (int)prev_state, (int)new_state, MOTOR_DEADTIME_MS);
            motor_brake();
            vTaskDelay(pdMS_TO_TICKS(MOTOR_DEADTIME_MS));
        }

        /* ---- Step 4: 笛卡尔→差速 PWM ---- */
        motor_diff_to_pwm(ema_x, ema_y, new_state, &left_duty, &right_duty);

        /* ---- Step 5: 驱动输出 ---- */
        motor_set(left_duty, right_duty);
        ESP_LOGI(TAG, "Motor set with (L: %d, R: %d)\n", left_duty, right_duty);

        prev_state = new_state;
    }
}
