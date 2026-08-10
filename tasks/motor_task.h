/**
 * @file motor_task.h
 * @brief 电机控制任务 (队列接收 → drv8833 底盘库周期驱动)
 *
 * 控制算法已迁移至 drivers/drv8833/（一阶低通滤波 + 差速映射），本任务为薄封装。
 */
#pragma once
#include "apf_common.h"

/**
 * @brief 电机控制任务入口
 */
void motor_task(void *pvParameters);
