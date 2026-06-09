/**
 * @file motor_task.c
 * @brief 电机控制任务: EMA 平滑 → 直接 dx/dy 差速映射 → PWM 输出
 *
 * 架构: dx=前进分量, dy=转向分量, 直接映射到左右轮.
 *   propulsion = dx × scale(|dy|)      // |dy| 越大前进越慢, 避免转向过快导致雷达畸变
 *   steering   = dy × STEER_GAIN       // 转向差速与 dy 成正比
 *   left/right = propulsion ± steering
 *
 * 优势: 无角度分解的非线性跳跃, 转向比例始终可预测, 调参直观
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

// 直接 dx/dy 差速映射
// dx → 前进分量 (两侧同向), dy → 转向分量 (两侧反向)
// dx-dy 耦合: |dy| 越大前进越低, 保证急转弯时降速
static inline void diff_control(float x, float y) {
    float dx_u = x / MOTOR_MAX_MM;          // 归一化到 [-1, 1]
    float dy_u = y / MOTOR_MAX_MM;

    // |dy| 越大, 前进分量衰减越多
    float scale = 1.0f - fabsf(dy_u) * MOTOR_DX_COUPLING;
    if (scale < 0.0f) scale = 0.0f;

    float propulsion = dx_u * scale;        // 动力: 被 dy 抑制的前进量
    float steering   = dy_u * MOTOR_STEER_GAIN; // 转向: 直接反映 dy

    float left  = propulsion + steering;
    float right = propulsion - steering;

    // 比例限幅 (保持左右转向比)
    float m = fmaxf(fabsf(left), fabsf(right));
    if (m > 1.0f) { left /= m; right /= m; }

    // 死区
    if (fabsf(left) < MOTOR_DEAD_ZONE && fabsf(right) < MOTOR_DEAD_ZONE) {
        motor_set(0, 0);
        return;
    }

    motor_set((int8_t)roundf(left  * 100.0f),
              (int8_t)roundf(right * 100.0f));
}

void motor_task(void *pvParameters) {
    (void)pvParameters;
    
    const TickType_t period = pdMS_TO_TICKS(1000 / MOTOR_FREQ_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    motor_ema_t ema = {0};
    ESP_LOGI(TAG, "Motor task started @ %dHz, alpha=%.2f coupling=%.2f steer_gain=%.2f",
        MOTOR_FREQ_HZ, (float)MOTOR_EMA_ALPHA,
        (float)MOTOR_DX_COUPLING, (float)MOTOR_STEER_GAIN);
    
    while (1) {
        vTaskDelayUntil(&last_wake, period);

        vector_cart_t cmd;
        if (xQueueReceive(q_cart, &cmd, 0) == pdTRUE) {
            ema_update(&cmd, &ema);
        } else {
            ema_update(NULL, &ema);    // 队列空 → 保持当前目标, EMA 自然收敛
        }

        diff_control(ema.dx, ema.dy);
    }
}
