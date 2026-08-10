/**
 * @file drv8833_port.h
 * @brief 移植层接口声明 — 移植到新平台时只需实现本文件声明的全部函数
 *
 * 注意：本文件不包含任何平台 HAL 头文件或宏，相关定义见 drv8833_port_def.h。
 */
#pragma once
#include "drv8833_types.h"

/**
 * @brief 初始化 LEDC 外设（定时器 + 通道 + 引脚）
 *
 * 硬件资源见 drv8833_port_def.h；成功后应将平台外设句柄填入 h->bus。
 *
 * @param h 实例句柄
 * @return DRV8833_OK 成功；DRV8833_ERR_INIT 初始化失败
 */
drv8833_err_t drv8833_port_pwm_init(drv8833_handle_t *h);

/**
 * @brief 反初始化 LEDC 外设，释放资源
 *
 * @param h 实例句柄
 * @return DRV8833_OK 成功；DRV8833_ERR_INIT 释放失败
 */
drv8833_err_t drv8833_port_pwm_deinit(drv8833_handle_t *h);

/**
 * @brief 设置单个逻辑通道占空比（通道索引见 drv8833_config.h 的 DRV8833_IN_*）
 *
 * @param h     实例句柄
 * @param ch    逻辑通道号（0~3）
 * @param duty  占空比计数值 [0, DRV8833_MAX_DUTY]
 * @return DRV8833_OK 成功；DRV8833_ERR_PARAM 参数非法
 */
drv8833_err_t drv8833_port_set_duty(drv8833_handle_t *h, uint8_t ch, uint32_t duty);

/**
 * @brief 毫秒级延时
 *
 * @note RTOS / 裸机实现由 drv8833_config.h 中的 DRV8833_RTOS_ACTIVE 控制
 * @param ms 延时毫秒数
 */
void drv8833_port_delay_ms(uint32_t ms);

/**
 * @brief 获取单调递增的毫秒时间戳（指令超时检测用）
 *
 * @return 当前毫秒时间戳
 */
uint32_t drv8833_port_get_tick_ms(void);
