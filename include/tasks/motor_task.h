/**
 * @file motor_task.h
 * @brief 电机控制任务 (EMA滤波 + 笛卡尔→差速转换 + 状态机 + 死区管理)
 *
 * 上游任务通过 q_cart 队列提供 vector_cart_t (x, y, 单位 mm, 范围 ±6m)
 * 本任务完成:
 *   1. EMA 低通滤波 — 平滑笛卡尔分量跳变
 *   2. 笛卡尔 → 差速转换 — 将 (x, y) 映射为左/右轮 PWM 速度
 *   3. 四态状态机 — 前进 / 后退 / 前进转向 / 后退转向
 *   4. 换向死区 — 方向反转时插入制动保持，保护 DRV8833
 *
 * 所有 RTOS 句柄来自 all_defs.h 的 extern 声明，无需参数传递。
 */
#pragma once
#include "all_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 电机状态枚举 (状态机) */
typedef enum {
    MOTOR_STATE_IDLE = 0,        /**< 静止 */
    MOTOR_STATE_FORWARD,         /**< 前进 (直线 / 微调) */
    MOTOR_STATE_REVERSE,         /**< 后退 (直线 / 微调) */
    MOTOR_STATE_FORWARD_TURN,    /**< 前进转向 */
    MOTOR_STATE_REVERSE_TURN,    /**< 后退转向 */
} motor_state_t;

/**
 * @brief 电机控制任务入口 (FreeRTOS 任务)
 * @param pvParameters 未使用 (传 NULL)
 */
void motor_task(void *pvParameters);

#ifdef __cplusplus
}
#endif
