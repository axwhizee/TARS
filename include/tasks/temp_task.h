/**
 * @file temp_task.h
 * @brief DS18B20 温度传感器任务
 *
 * 职责: 以 4Hz 频率启动温度转换 → 轮询完成 → 读取温度 →
 *       推送 q_temp → 置 BIT_TEMP_READY
 */
#pragma once
#include "all_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

void temp_task(void *pvParameters);

#ifdef __cplusplus
}
#endif
