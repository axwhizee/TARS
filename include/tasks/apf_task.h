/**
 * @file apf_task.h
 * @brief 人工势场法 (APF) 避障任务
 *
 * 上游: LiDAR + 火焰传感器通过 q_polar 推送极坐标点, 置位 BIT_LIDAR_Q_READY / BIT_FLAME_Q_READY
 * 本任务:
 *   1. 等待 BIT_LIDAR_Q_READY | BIT_FLAME_Q_READY (eg_sync)
 *   2. 从 q_polar 读取 Q_POLAR_DEPTH 个扇区数据, 同步透传至 q_log
 *   3. 清除传感器位, 置位 BIT_LOG_Q_READY
 *   4. 按距离分区: 0~2m 危险区, 2~6m 感知区, >6m 噪声 (丢弃)
 *   5. 计算引力 (向前) + 斥力 (远离障碍) = 合力
 *   6. 将合力 vector_cart_t 推入 q_cart 供电机控制任务消费
 *
 * 所有 RTOS 句柄来自 all_defs.h 的 extern 声明，无需参数传递。
 */
#pragma once
#include "all_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief APF 避障任务入口 (FreeRTOS 任务)
 * @param pvParameters 未使用 (传 NULL)
 */
void apf_task(void *pvParameters);

#ifdef __cplusplus
}
#endif
