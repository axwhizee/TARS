/**
 * @file motor.h
 * @brief 电机驱动底层 API (DRV8833 IN/IN + LEDC PWM)
 */
#pragma once
#include "all_defs.h"

/**
 * @brief 初始化电机驱动 (配置 GPIO + LEDC PWM 通道)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t motor_init(void);

/**
 * @brief 设置左右电机转速（-100~100），已修正克服静摩擦力的占空比下限
 * 
 * @param left  左侧电机: [-100, 100]（正数 = 前进, 负数 = 后退, 0 = 滑行）
 * @param right 右侧电机: 同上
 * @return ESP_OK 成功
 * @note 内部将 [1,100] 重映射到 [MIN_EFF_DUTY, MAX_DUTY];
 *       调用方负责在换向 (正转↔反转) 前插入制动死区。
 */
esp_err_t motor_set(int8_t left, int8_t right);

// 刹车
void motor_brake(void);
// 滑行（无动力）
void motor_coast(void);
