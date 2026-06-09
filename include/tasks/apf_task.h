/**
 * @file apf_task.h
 * @brief 人工势场法 (APF) 避障任务
 */
#pragma once
#include "apf_common.h"

/**
 * @brief APF 避障任务入口 (FreeRTOS task)
 */
void apf_task(void *pvParameters);

/**
 * @brief 重置 APF+VFH 参数为编译期默认值
 */
void params_init_defaults(void);
