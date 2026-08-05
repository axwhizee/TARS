/**
 * @file temp_task.h
 * @brief DS18B20 温度传感器任务 — 非阻塞跳过模式
 */
#pragma once
#include "apf_common.h"

/**
 * @brief 温度传感器任务入口
 */
void temp_task(void *pvParameters);
