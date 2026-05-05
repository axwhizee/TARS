#pragma once
#include "all_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

/*
 * lidar_task.h — LD14P 传感器任务
 *
 * 职责: 轮询 UART1 → 喂协议状态机 → 检测完整一圈 →
 *       降采样 360→36 点 → 推送 q_polar → 置 BIT_LIDAR_READY
 */

typedef struct {
    QueueHandle_t       q_polar;   /* 输出队列, 36 个 vector_polar_t */
    EventGroupHandle_t  eg_sync;   /* 同步事件组, BIT_LIDAR_READY */
} lidar_sensor_params_t;

void ld14p_sensor_task(void *arg);
