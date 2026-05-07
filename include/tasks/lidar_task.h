#pragma once
#include "all_defs.h"

/**
 * lidar_task.h — LD14P 传感器任务
 *
 * 职责: 轮询 UART1 → 喂协议状态机 → 检测完整一圈 →
 *       降采样 360→36 点 → 推送 q_polar → 置 BIT_LIDAR_READY
 */
void ld14p_task(void *arg);
