#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief 通用数据日志任务
 *        每2秒从队列取出360个 vector_polar_t, 通过UART0上传
 *        与传感器完全解耦, 仅消费队列中的数据
 * @param arg QueueHandle_t (队列元素: vector_polar_t, 深度360)
 */
void logger_task(void *arg);
