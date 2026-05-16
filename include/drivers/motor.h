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
 *
 * 百分比死区:
 *   motor_set_side() 内部将 1%~100% 映射到 [MIN_EFF, MAX] 占空比,
 *   0% 为滑行停车。调用方无需关心底层 PWM 分辨率。
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
 * @brief 设置左右电机转速 / 方向 (百分比制)
 *
 * @param left  左侧电机: [-100, 100]
 *              正数 = 前进, 负数 = 后退, 0 = 滑行
 * @param right 右侧电机: 同上
 * @return ESP_OK 成功
 *
 * @note 内部将 [1,100] 重映射到 [MIN_EFF_DUTY, MAX_DUTY];
 *       调用方负责在换向 (正转↔反转) 前插入制动死区。
 */
esp_err_t motor_set(int8_t left, int8_t right);

void motor_brake(void);
void motor_coast(void);

#ifdef __cplusplus
}
#endif
