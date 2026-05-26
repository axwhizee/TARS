/**
 * @file motor_task.h
 * @brief 电机控制任务 (EMA滤波 + 笛卡尔→差速转换 + 状态机 + 死区管理)
 */
#pragma once
#include "all_defs.h"

/**
 * @brief 电机控制任务入口
 */
void motor_task(void *pvParameters);
