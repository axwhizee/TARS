/**
 * @file flame_task.h
 * @brief 火焰传感器任务 — 5 路 GPIO 检测 + 虚拟障碍物注入
 *
 * 每个传感器对应一个固定角度的 vector_polar_t:
 *   GPIO11→300°, GPIO12→330°, GPIO15→0°, GPIO13→30°, GPIO14→60°
 * LOW  = 检测到火焰 → distance = 500mm (危险区)
 * HIGH = 无火焰       → distance = 65535 (无效)
 */
#pragma once
#include "all_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t flame_sensor_init(void);
void flame_task(void *pvParameters);

#ifdef __cplusplus
}
#endif
