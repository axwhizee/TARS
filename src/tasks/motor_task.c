/**
 * @file motor_task.c
 * @brief 电机控制任务: EMA 平滑 → 分类 → PWM → 换向死区
 *
 * 架构: motor_classify() 分离判定, motor_apply() 独立计算 PWM.
 *
 * 判定优先级 (从上到下):
 *   1. M_DEAD   | |x|<50 且 |y|<50                | L=R=2
 *   2. M_SPIN_L | |y| > 2*|x| (垂向主导)         | ang × 0.5 纯旋转
 *   3. M_SPIN   | -MAX/2 < x < 0 (弱后退)        | ang × 1.0 纯旋转
 *   4. M_REV    | -MAX < x < -MAX/2 (强后退)     | 反向 ang + 速度减半
 *   5. M_FWD    | 其余                            | L=lin-ang, R=lin+ang
 */
#include "tasks/motor_task.h"
#include "drivers/drv8833.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MOTOR_TASK";

// EMA滤波器状态（仅保留必要字段）
typedef struct {
    float dx,dy;               // 滤波输出 (当前值)
    float target_x, target_y;   // 滤波目标
    TickType_t last_cmd;        // 最后收到指令的时刻
} motor_ema_t;

/**
 * @brief 标准一阶低通滤波 + 超时归零 + 幅值钳位
 * 公式: y[n] = α*x[n] + (1-alpha)*y[n-1]
 */
static inline void motor_ema_update(const vector_cart_t *cmd, motor_ema_t *ema) {
    const float alpha = MOTOR_EMA_ALPHA;
    const float alpha_inv = 1.0f - alpha;
    TickType_t now = xTaskGetTickCount();
    
    // 若有新指令 → 更新目标 + 刷新时间戳
    if (cmd) {
        ema->target_x = cmd->dx;
        ema->target_y = cmd->dy;
        ema->last_cmd = now;
    }
    
    // 超时保护 → 目标归零 (自然停车)
    if ((now - ema->last_cmd) > pdMS_TO_TICKS(MOTOR_TIMEOUT_MS)) {
        ema->target_x = ema->target_y = 0.0f;
    }
    
    // 3. 一阶低通滤波 + 钳位 (使用标准库函数)
    ema->dx = fmaxf(-MOTOR_MAX_MM, fminf(MOTOR_MAX_MM, alpha * ema->target_x + alpha_inv * ema->dx));
    ema->dy = fmaxf(-MOTOR_MAX_MM, fminf(MOTOR_MAX_MM, alpha * ema->target_y + alpha_inv * ema->dy));
}

// 运动模式枚举
typedef enum {
    M_DEAD,     // 死区: |x|≈|y|≈0 → 怠速
    M_SPIN_L,   // 垂向主导 原地旋转 (灵敏度×0.5)
    M_SPIN,     // 弱后退 原地旋转 (正常灵敏度)
    M_FWD,      // 前进差速
    M_REV,      // 后退差速 |x|≥3m
} motor_mode_t;

// ─── 状态分类: 直接用原始力 (x, y) 判定 ──────────────────────
static inline motor_mode_t motor_classify(float x, float y) {
    if (fabsf(x) < MOTOR_DEADZONE_MM && fabsf(y) < MOTOR_DEADZONE_MM)
        return M_DEAD;

    // |y| > 2|x| → 垂向主导, 低灵敏度旋转
    if (fabsf(y) > fabsf(x) * 2.0f)
        return M_SPIN_L;

    // 后退: |x|<3m → 正常旋转, |x|≥3m → 差速倒车
    if (x < 0.0f)
        return (fabsf(x) >= MOTOR_MAX_MM * 0.5f) ? M_REV : M_SPIN;

    return M_FWD;
}

// ─── 原地旋转: L=-ang, R=+ang, sensitivity 控制灵敏度 ──────────
static inline void motor_spin(float y, float sensitivity) {
    float ang = (y / MOTOR_MAX_MM) * MOTOR_TURN_RATIO * sensitivity;
    ang = fmaxf(-1.0f, fminf(1.0f, ang));
    int8_t l = (int8_t)roundf(-ang * 100.0f);
    int8_t r = (int8_t)roundf( ang * 100.0f);
    motor_set(l, r);
}

// ─── 差速计算 + PWM 输出 ───────────────────────────────────────
static inline void motor_apply(motor_mode_t mode, float x, float y) {
    float lin, ang, left = 0.0f, right = 0.0f, m;
    int8_t l, r;

    switch (mode) {
    case M_DEAD:
        motor_set(2, 2);
        return;

    case M_SPIN_L:  motor_spin(y, 1.0f);  return;   // 垂向主导
    case M_SPIN:    motor_spin(y, 1.0f);  return;   // 弱后退

    case M_FWD:
        lin = x / MOTOR_MAX_MM;
        ang = (y / MOTOR_MAX_MM) * MOTOR_TURN_RATIO;
        left  = lin - ang;
        right = lin + ang;
        break;

    case M_REV:
        lin = fabsf(x) / MOTOR_MAX_MM * 0.5f;
        ang = (y / MOTOR_MAX_MM) * MOTOR_TURN_RATIO * 2.0f;
        ang = -ang;
        left  = -lin - ang;
        right = -lin + ang;
        break;
    }

    m = fmaxf(fabsf(left), fabsf(right));
    if (m > 1.0f) { left /= m; right /= m; }
    l = (int8_t)roundf(left  * 100.0f);
    r = (int8_t)roundf(right * 100.0f);
    motor_set(l, r);
}

// ─── 统一公式差速 (动态转向增益, lateral 感知) ─────────────────
static inline void motor_apply_simple(float x, float y) {
    float lin = x / MOTOR_MAX_MM;
    float ang = (y / MOTOR_MAX_MM) * MOTOR_TURN_RATIO;

    // 动态转向: dx↓→转向↑, dy↓→走廊中保持直行
    // 走廊(dy≈0,dx大): boost≈1.15→不转  靠墙(dy有明显值): boost逐增
    ang *= 1.0f + (fabsf(ang) + 0.12f) / (fabsf(lin) + 0.10f);
    if (lin < 0.0f) {
        ang += copysignf(0.08f, y);   // 对称破缺偏置
        lin *= 0.4f;                  // 后退减速
    }

    float left  = lin - ang;
    float right = lin + ang;

    // 比例限幅 (保持转向比)
    float m = fmaxf(fabsf(left), fabsf(right));
    if (m > 1.0f) { left /= m; right /= m; }

    // 死区怠速
    if (fabsf(left) * 100.0f < 3.0f && fabsf(right) * 100.0f < 3.0f) {
        motor_set(2, 2);
        return;
    }

    int8_t l = (int8_t)roundf(left  * 100.0f);
    int8_t r = (int8_t)roundf(right * 100.0f);
    motor_set(l, r);
}

// ─── 换向制动: 仅 FWD↔REV 触发, SPIN/DEAD 不干涉 ─────────────
static inline bool motor_needs_brake(motor_mode_t prev, motor_mode_t curr) {
    if (prev == M_FWD && curr == M_REV) return true;
    if (prev == M_REV && curr == M_FWD) return true;
    return false;
}

// ─── 任务入口 ───────────────────────────────────────────────────
void motor_task(void *pvParameters) {
    (void)pvParameters;
    
    const TickType_t period = pdMS_TO_TICKS(1000 / MOTOR_FREQ_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    motor_ema_t ema = {0};
    // motor_mode_t prev_mode = M_DEAD;  // 状态机保留
    ESP_LOGI(TAG, "Motor task started @ %dHz, alpha=%.2f (τ=%dms)",
        MOTOR_FREQ_HZ, (float)MOTOR_EMA_ALPHA, MOTOR_EMA_TAU_MS);
    
    while (1) {
        vTaskDelayUntil(&last_wake, period);

        vector_cart_t cmd;
        const vector_cart_t *pcmd = (xQueueReceive(q_cart, &cmd, 0) == pdTRUE) ? &cmd : NULL;
        motor_ema_update(pcmd, &ema);

        // --- 统一公式 (测试中) ---
        motor_apply_simple(ema.dx, ema.dy);

        // --- 状态机 (保留) ---
        // motor_mode_t mode = motor_classify(ema.dx, ema.dy);
        // if (motor_needs_brake(prev_mode, mode)) {
        //     motor_brake();
        //     vTaskDelay(pdMS_TO_TICKS(MOTOR_DEADTIME_MS));
        // }
        // motor_apply(mode, ema.dx, ema.dy);
        // prev_mode = mode;

        #ifdef DEBUG
        ESP_LOGD(TAG, "x=%.0f y=%.0f", ema.dx, ema.dy);
        #endif
    }
}
