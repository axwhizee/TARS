/**
 * @file motor_task.c
 * @brief 电机控制任务: EMA 平滑 → 分类 → PWM → 换向死区
 *
 * 架构: motor_classify() 分离判定, motor_apply() 独立计算 PWM.
 *
 * 判定优先级 (从上到下):
 *   1. M_DEAD   | |x|<DEADZONE 且 |y|<DEADZONE    | L=R=2
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
static inline void ema_update(const vector_cart_t *cmd, motor_ema_t *ema) {
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

// 统一公式差速 (动态转向系数)
static inline void diff_control(float x, float y) {
    float lin = x / MOTOR_MAX_MM;
    float ang = (y / MOTOR_MAX_MM) * MOTOR_TURN_RATIO;

    /**
     * 动态转向增益：
     * 与 lin 正相关，高速行进时转向更灵敏
     * 与 ang 负相关，防止急转时转向过度
     */
    float gain = MOTOR_TURN_GAIN_MI +
        MOTOR_TURN_GAIN_MX * fabsf(lin) * (1.0f - fabsf(ang));
    ang *= gain;
    if (lin < 0.0f) {
        if (lin < -0.3f) {
            ang += copysignf(0.1f, y);  // 对称破缺偏置
        }
        lin *= 0.4f;
    }

    float left  = lin - ang;
    float right = lin + ang;

    // 比例限幅 (保持转向比)
    float m = fmaxf(fabsf(left), fabsf(right));
    if (m > 1.0f) { left /= m; right /= m; }

    // 死区怠速
    if (fabsf(left) * 100.0f < 3.0f && fabsf(right) * 100.0f < 3.0f) {
        motor_set(0, 0);
        return;
    }

    int8_t l = (int8_t)roundf(left  * 100.0f);
    int8_t r = (int8_t)roundf(right * 100.0f);
    motor_set(l, r);
}

void motor_task(void *pvParameters) {
    (void)pvParameters;
    
    const TickType_t period = pdMS_TO_TICKS(1000 / MOTOR_FREQ_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    motor_ema_t ema = {0};
    ESP_LOGI(TAG, "Motor task started @ %dHz, alpha=%.2f (τ=%dms)",
        MOTOR_FREQ_HZ, (float)MOTOR_EMA_ALPHA, MOTOR_EMA_TAU_MS);
    
    while (1) {
        vTaskDelayUntil(&last_wake, period);

        vector_cart_t cmd = {0};
        if (xQueueReceive(q_cart, &cmd, 0) != pdTRUE) cmd.dx = cmd.dy = 0;
        ema_update(&cmd, &ema);

        // 简化的x、y分量限幅
        // if (cmd.dx > APF_RANGE_MAX || cmd.dy > APF_RANGE_MAX) {
        //     if (cmd.dx > cmd.dy) {
        //         cmd.dy *= APF_RANGE_MAX / cmd.dx;
        //         cmd.dx = APF_RANGE_MAX;
        //     } else {
        //         cmd.dx *= APF_RANGE_MAX / cmd.dy;
        //         cmd.dy = APF_RANGE_MAX;
        //     }
        // }
        // 统一公式 (测试中)
        diff_control(ema.dx, ema.dy);
    }
}
