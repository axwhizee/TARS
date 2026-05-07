/**
 * @file lidar_task.h
 * @brief LD14P 传感器任务
 *
 * 职责: 轮询 UART1 → 喂协议状态机 → 检测完整一圈 →
 *       降采样 360→36 点 → 推送 q_polar → 置 BIT_LIDAR_READY
 */
#pragma once
#include "all_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

void ld14p_task(void *pvParameters);

#ifdef __cplusplus
}
#endif
