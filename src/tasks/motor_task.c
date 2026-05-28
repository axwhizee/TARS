/**
 * @file motor_task.c
 * @brief 电机控制任务: EMA 平滑 + 三态差速转换 + 换向死区
 *
 * 三态决策 (基于 EMA 滤波后的 cmd):
 *   1. 死区: |x|=|y|≈0      → 停车 (M_IDLE)
 *   2. 差速: 正常行驶/后退   → lin ± ang
 *   3. 原地转向: x<0 且侧向力主导 → 清零前进分量, 纯差速旋转
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

// 简化版状态枚举（仅用于换向检测）
typedef enum { M_IDLE, M_FWD, M_REV } motor_dir_t;

/**
 * @brief 滤波后 cmd → 差速占空比 (三态决策)
 * @return 当前运动方向 (用于换向死区判断)
 */
static inline motor_dir_t motor_cmd_to_duty(const motor_ema_t *ema, int8_t *l, int8_t *r) {
    float x = ema->dx, y = ema->dy;

    // --- 态 1: 死区 ---
    if (fabsf(x) < MOTOR_DEADZONE_MM && fabsf(y) < MOTOR_DEADZONE_MM) {
        *l = *r = 10;   // 低速前进避免停滞
        return M_IDLE;
    }

    // cmd → [-1, 1]: 除以 MOTOR_MAX_MM 等效于除以 60 → duty%
    float lin = x / MOTOR_MAX_MM;
    float ang = (y / MOTOR_MAX_MM) * MOTOR_TURN_RATIO;

    // --- 态 3: 原地转向 (x<0 且侧向力主导时清零前进分量) ---
    if (x < 0 && fabsf(ang) > fabsf(lin) * 2.0f) {
        lin = 0;
    }

    float left  = lin - ang;
    float right = lin + ang;

    // 比例限幅 (保持转向比)
    float m = fmaxf(fabsf(left), fabsf(right));
    if (m > 1.0f) { left /= m; right /= m; }

    *l = (int8_t)roundf(left  * 100.0f) / 2;    // 减慢后退速度
    *r = (int8_t)roundf(right * 100.0f);

    // 态 2: 正常差速 (x>0 前进, x<0 后退 — 公式自动处理 reversed steering)
    return (x > 0) ? M_FWD : M_REV;
}

// 换向检测工具函数
static inline bool motor_is_reversal(motor_dir_t prev, motor_dir_t curr) {
    return (prev != M_IDLE && curr != M_IDLE && prev != curr);
}

void motor_task(void *pvParameters) {
    (void)pvParameters;
    
    const TickType_t period = pdMS_TO_TICKS(1000 / MOTOR_FREQ_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    motor_ema_t ema = {0};          // EMA状态清零
    motor_dir_t prev_dir = M_IDLE;  // 上一周期方向
    ESP_LOGI(TAG, "Motor task started @ %dHz, alpha=%.2f (τ=%dms)",
        MOTOR_FREQ_HZ, (float)MOTOR_EMA_ALPHA, MOTOR_EMA_TAU_MS);
    
    while (1) {
        vTaskDelayUntil(&last_wake, period);    // 任务周期性运行
        
        // 非阻塞获取指令
        vector_cart_t cmd;
        const vector_cart_t *pcmd = (xQueueReceive(q_cart, &cmd, 0) == pdTRUE) ? &cmd : NULL;
        
        // EMA滤波 (含超时归零)
        motor_ema_update(pcmd, &ema);
        
        // 笛卡尔→差速PWM (三态决策)
        int8_t l_duty, r_duty;
        motor_dir_t curr_dir = motor_cmd_to_duty(&ema, &l_duty, &r_duty);
        
        // 换向死区保护 (仅方向反转时制动)
        if (motor_is_reversal(prev_dir, curr_dir)) {
            motor_brake();
            vTaskDelay(pdMS_TO_TICKS(MOTOR_DEADTIME_MS));
        }
        
        // 输出PWM
        motor_set(l_duty, r_duty);
        
        // 更新状态 (调试日志按需开启)
        #ifdef DEBUG
        ESP_LOGD(TAG, "PWM L:%+d R:%+d | dir:%d", l_duty, r_duty, curr_dir);
        #endif
        prev_dir = curr_dir;
    }
}
