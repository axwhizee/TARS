/**
 * @file flame_task.h
 * @brief 火焰传感器任务 — 5 路 GPIO 检测 + 虚拟障碍物注入
 */
#pragma once
#include "apf_common.h"

// 火焰传感器初始化
esp_err_t flame_sensor_init(void);
/**
 * @brief 火焰传感器任务入口
 */
void flame_task(void *pvParameters);
