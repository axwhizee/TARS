/**
 * @file sysmon_task.h
 * @brief 系统监控任务 — 心跳 LED + FreeRTOS 运行时统计周期输出
 */
#pragma once
#include "apf_common.h"

void sysmon_task(void *pvParameters);
