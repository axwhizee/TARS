/**
 * @file motor_task.c
 * @brief 电机控制任务 — 周期读取 q_cart 指令并驱动 drv8833 底盘库
 *
 * 控制算法（一阶低通滤波 + 超时归零 + 四轮差速映射）已迁移至
 * drivers/drv8833/ 核心层，本任务只做：队列接收 → set_velocity → 周期 update。
 */
#include "tasks/motor_task.h"
#include "drivers/drv8833/drv8833.h"
#include "apf_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

extern drv8833_handle_t g_motor;   // main.c 定义

static const char *TAG = "MOTOR_TASK";

void motor_task(void *pvParameters) {
    (void)pvParameters;

    const TickType_t period = pdMS_TO_TICKS(1000 / MOTOR_FREQ_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    ESP_LOGI(TAG, "Motor task started @ %dHz (chassis=%s, lpf=%d)",
        MOTOR_FREQ_HZ, "diff_4wd", g_motor.lpf_enabled);

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        // 从 q_cart 读取 APF/手动指令
        vector_cart_t cmd;
        if (xQueueReceive(q_cart, &cmd, 0) == pdTRUE) {
            drv8833_set_velocity(&g_motor, cmd.dx, cmd.dy);
        }
        // 无新指令 → 目标保持, 由驱动内部超时归零 + LPF 自然收敛
        drv8833_update(&g_motor);
    }
}
