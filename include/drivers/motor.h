/**
 * @file motor.h
 * @brief 电机驱动底层 API (DRV8833 IN/IN + LEDC PWM)
 *
 * 硬件:
 *   - DRV8833 双路电机驱动, IN/IN + Fast Decay 控制模式
 *   - 左右两侧各 2 个电机，同侧对置安装，错位并联 (反向并联)
 *   - GPIO4/5 控制左侧，GPIO6/7 控制右侧
 *   - LEDC 输出 20kHz PWM, 10-bit 分辨率
 *
 * 控制逻辑 (Fast Decay):
 *   正转: IN1=PWM, IN2=0
 *   反转: IN1=0,   IN2=PWM
 *   制动: IN1=1,   IN2=1  (Low-side brake)
 *   滑行: IN1=0,   IN2=0  (Hi-Z coast)
 */
#pragma once

#include "all_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化电机驱动 (配置 GPIO + LEDC PWM 通道)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t motor_init(void);

/**
 * @brief 设置左右电机转速 / 方向
 *
 * @param left  左侧电机: [-MOTOR_MAX_DUTY, MOTOR_MAX_DUTY]
 *              正数 = 前进, 负数 = 后退, 零 = 滑行
 * @param right 右侧电机: 同上
 * @return ESP_OK 成功
 *
 * @note 调用方负责在换向 (正转↔反转) 前插入制动死区
 */
esp_err_t motor_set(int16_t left, int16_t right);

/**
 * @brief 主动制动 (四轮同时 Low-side Brake)
 *        所有 IN 引脚置高 → 电机绕组短接 → 快速制动
 */
void motor_brake(void);

/**
 * @brief 滑行停车 (四轮同时 Hi-Z Coast)
 *        所有 IN 引脚置低 → 电机惯性转动
 */
void motor_coast(void);

/**
 * @brief 释放 LEDC 资源 (仅调试 / 重启前调用)
 */
void motor_deinit(void);

#ifdef __cplusplus
}
#endif
