/**
 * @file main.c
 * @brief 主入口 — 系统初始化 → 硬件驱动 → RTOS 通信对象 → 任务创建
 *
 * 启动流程:
 *   1. sys_nvs_init()       — NVS flash 存储
 *   2. sys_spiffs_init()    — SPIFFS 挂载 (网页文件)
 *   3. sys_wifi_init()      — Wi-Fi AP 启动 (WEB_ADDR:80)
 *   4. 硬件驱动初始化        — LD14P, DRV8833, DS18B20, Flame
 *   5. 队列 + 事件组创建     — RTOS IPC 基础设施
 *   6. xTaskCreatePinnedToCore ×7 — 双核分工
 *
 * 双核分配:
 *   Core 0: WiFi + lwIP + web_task (网络专用, 不被传感器/控制打断)
 *   Core 1: lidar, flame, temp, apf, motor, sysmon (传感器/控制专用)
 *
 * app_main() 返回后 FreeRTOS 调度器自动启动.
 * 所有单位: 距离 mm, 时间 ms, 角度 °.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "apf_common.h"
#include "sys_init.h"
#include "drivers/ld14p/ld14p.h"
// #include "drivers/ds18b20.h"
#include "drivers/drv8833/drv8833.h"
#include "tasks/lidar_task.h"
#include "tasks/flame_task.h"
#include "tasks/temp_task.h"
#include "tasks/apf_task.h"
#include "tasks/motor_task.h"
#include "tasks/web_task.h"
#include "tasks/sysmon_task.h"

static const char *TAG = "MAIN";

// APF+VFH 可调参数 (由 sys_init.c NVS 加载, web_task.c 写入)
apf_params_t g_apf_params;

// 全局 RTOS 通信对象 (定义在此, extern 供所有任务引用)

QueueHandle_t        q_polar;   // LD14P + 火焰传感器极坐标数据
QueueHandle_t        q_cart;    // APF 笛卡尔合力结果 (xQueueOverwrite, depth=1)
QueueHandle_t        q_temp;    // DS18B20 温度数据 (depth=4)
QueueHandle_t        q_log;     // 日志透传队列 (vector_polar_t, 供 web_task 读取)
EventGroupHandle_t   eg_sync;   // 传感器就绪 + 日志就绪事件组

// LD14P 驱动实例 (lidar_task.c 中 extern 引用)

ld14p_handle_t       g_ld14p;
static const ld14p_cfg_t ld14p_cfg = { .target_freq_hz = SENSOR_FREQ };

// DRV8833 电机驱动实例 (motor_task.c 中 extern 引用)

drv8833_handle_t     g_motor;
static const drv8833_cfg_t motor_cfg = { .vel_max_mm = APF_RANGE_MAX };

void app_main(void) {
    ESP_LOGI(TAG, "\nSystem Initializing...\n");

    // 1. 系统（NVS、SPIFFS、Wi-Fi AP）初始化

    if (sys_nvs_init()    != ESP_OK) { ESP_LOGE(TAG, "NVS init failed"); return; }
    params_init_defaults();
    if (sys_spiffs_init() != ESP_OK) { ESP_LOGE(TAG, "SPIFFS init failed"); return; }
    if (sys_wifi_init()   != ESP_OK) { ESP_LOGE(TAG, "Wi-Fi init failed"); return; }

    // 2. 外设驱动初始化 (LD14P / DRV8833 / DS18B20 / Flame)

    // LiDAR 传感器驱动
    if (ld14p_init(&g_ld14p, &ld14p_cfg) != LD14P_OK) {
        ESP_LOGE(TAG, "LD14P init failed"); return;
    }
    // 电机 PWM 驱动
    if (drv8833_init(&g_motor, &motor_cfg) != DRV8833_OK) { ESP_LOGE(TAG, "Motor init failed"); return; }
    // DS18B20 — 已改用 ESP32-S3 内置温度传感器 (消除 1-Wire 关中断干扰)
    // if (ds18b20_init() != ESP_OK) { ESP_LOGW(TAG, "DS18B20 init failed"); }
    // 火焰传感器 初始化
    if (flame_sensor_init() != ESP_OK) { ESP_LOGW(TAG, "Flame sensor init failed"); return; }

    // 3. 任务间通信对象 — 队列 + 事件组

    q_polar = xQueueCreate(Q_POLAR_DEPTH, sizeof(vector_polar_t));
    if (!q_polar) { ESP_LOGE(TAG, "q_polar create fail"); return; }
    q_cart = xQueueCreate(1, sizeof(vector_cart_t));
    if (!q_cart)  { ESP_LOGE(TAG, "q_cart create fail");  return; }
    q_temp = xQueueCreate(4, sizeof(float));
    if (!q_temp)  { ESP_LOGE(TAG, "q_temp create fail");  return; }
    q_log = xQueueCreate(Q_POLAR_DEPTH, sizeof(vector_polar_t));
    if (!q_log)   { ESP_LOGE(TAG, "q_log create fail");   return; }
    eg_sync = xEventGroupCreate();
    if (!eg_sync) { ESP_LOGE(TAG, "eg_sync create fail");  return; }

    // 4. 创建 FreeRTOS 任务
    // Core 0: WiFi (固定) + lwIP (固定) + web_task
    // Core 1: 所有传感器/控制任务
    //  xTaskCreatePinnedToCore(func, name, stack, param, prio, handle, core)

    drv8833_set_speed(&g_motor, 0, 0);
    xEventGroupSetBits(eg_sync, BIT_MANUAL_MODE);   // 初始状态设置为手动模式，需要手动切换
    xTaskCreatePinnedToCore(flame_task, "flame",    3072, NULL, 7, NULL, 1);    // Core1
    xTaskCreatePinnedToCore(temp_task,  "temp",     4096, NULL, 6, NULL, 1);    // Core1
    xTaskCreatePinnedToCore(ld14p_task, "ld14p",    8192, NULL, 5, NULL, 1);    // Core1
    xTaskCreatePinnedToCore(apf_task,   "apf",      4096, NULL, 4, NULL, 1);    // Core1
    xTaskCreatePinnedToCore(motor_task, "motor",    8192, NULL, 8, NULL, 1);    // Core1
    xTaskCreatePinnedToCore(web_task,   "web_task", 8192, NULL, 3, NULL, 0);    // Core0 网络专用
    xTaskCreatePinnedToCore(sysmon_task,"sysmon",   4096, NULL, 1, NULL, 1);    // Core1 心跳+统计
    vTaskDelay(pdMS_TO_TICKS(1000));    // 1s 缓冲

    ESP_LOGI(TAG, "App_main done, scheduler starting...\n");
}
