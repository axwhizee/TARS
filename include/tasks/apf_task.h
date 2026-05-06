/**
 * @file apf_task.h
 * @brief 人工势场法 (APF) 避障任务
 *
 * 上游: LiDAR 传感器任务通过 q_polar 推送极坐标点, 置位 BIT_LIDAR_READY
 * 本任务:
 *   1. 等待 BIT_LIDAR_READY 信号
 *   2. 从 q_polar 读取 LIDAR_SECTORS 个扇区数据
 *   3. 按距离分区: 0~2m 危险区, 2~6m 感知区, >6m 噪声 (丢弃)
 *   4. 计算引力 (向前) + 斥力 (远离障碍) = 合力
 *   5. 将合力 vector_cart_t 推入 q_cart 供电机控制任务消费
 *
 * 算法:
 *   F_total = F_att + Σ F_rep_i
 *   F_att = (K_att, 0)                        // 恒定向前引力
 *   F_rep_i = -K_rep * w(r) * (dx/r^3, dy/r^3) // 远离障碍的斥力
 *   w(r) = danger_wt (r<=2m) / safe_wt (2m<r<=6m)
 */
#pragma once

#include "all_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

/** APF 任务参数 */
typedef struct {
    QueueHandle_t      q_polar;   /**< 输入: LiDAR 极坐标队列 */
    QueueHandle_t      q_cart;    /**< 输出: 笛卡尔合力指令队列 → motor_task */
    EventGroupHandle_t eg_sync;   /**< 事件组: BIT_LIDAR_READY 信号 */
} apf_task_params_t;

/**
 * @brief APF 避障任务入口 (FreeRTOS 任务)
 * @param pvParameters 指向 apf_task_params_t 的指针
 */
void apf_task(void *pvParameters);

#ifdef __cplusplus
}
#endif
