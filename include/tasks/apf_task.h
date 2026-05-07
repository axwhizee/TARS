/**
 * @file apf_task.h
 * @brief 人工势场法 (APF) 避障任务
 *
 * 上游: LiDAR 传感器任务通过 q_polar 推送极坐标点, 置位 BIT_LIDAR_READY
 * 本任务:
 *   1. 等待 BIT_LIDAR_READY 信号 (eg_sync)
 *   2. 从 q_polar 读取 LIDAR_SECTORS 个扇区数据
 *   3. 按距离分区: 0~2m 危险区, 2~6m 感知区, >6m 噪声 (丢弃)
 *   4. 计算引力 (向前) + 斥力 (远离障碍) = 合力
 *   5. 将合力 vector_cart_t 推入 q_cart 供电机控制任务消费
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
