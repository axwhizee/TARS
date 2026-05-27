/**
 * @file web_task.h
 * @brief Web 服务与日志上传 — HTTP 静态文件 + WebSocket JSON 推送
 */
#pragma once
#include "apf_common.h"

/**
 * @brief Web 任务入口 (HTTP 服务器初始化 + WebSocket 数据推送循环)
 */
void web_task(void *pvParameters);
