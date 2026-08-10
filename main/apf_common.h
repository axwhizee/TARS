/**
 * @file apf_common.h
 * @brief 项目公共定义、数据结构和RTOS句柄
 * 
 * 所有数据的单位统一为：距离mm，时间ms，角度°
 */
#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include <stdbool.h>
#include <stdint.h>

#define DEG_2_RAD (float)(2.0f * 3.1416f / 360.0f)

// WiFi 与全局配置

#define WIFI_SSID           "TARS"   // 未加密的开发 AP
#define WEB_ADDR            "192.168.10.1"
#define WEBSOCKET_PORT      80      // HTTP 协议的默认端口
// 传感器同步频率，直接决定温度/火焰传感器采样频率，间接影响APF法处理频率，取决于雷达频率（2~8Hz）
#define SENSOR_FREQ         4

// 极坐标向量（统一传感器数据）
typedef struct {
    float dst;  // 带显式角度的输出结构 (公用数据类型)
    float ang;  // 传感器正前方为 0°, 顺时针递增 (LD14P 原生坐标系)
} vector_polar_t;

// 笛卡尔坐标向量（APF计算结果，控制指令）
typedef struct {
    float dx;   // 前向
    float dy;   // 侧向
} vector_cart_t;

// RTOS 全局句柄（已在 main 中声明并分配内存）

extern QueueHandle_t        q_polar;    // LD14P 数据队列
extern QueueHandle_t        q_cart;     // APF 计算结果队列，深度 1
extern QueueHandle_t        q_temp;     // DS18B20 温度数据队列
extern QueueHandle_t        q_log;      // 日志上传数据队列 (透传 vector_polar_t)
extern EventGroupHandle_t   eg_sync;    // 传感器同步事件组
extern vector_cart_t        g_apf_rep;  // APF 斥力分量 (web 可视化)
extern vector_cart_t        g_vfh_att;  // VFH 引力分量 (web 可视化)

// 任务间同步事件组

#define BIT_LIDAR_Q_READY   (1 << 0)    // q_polar 中有新的激光雷达数据
#define BIT_TEMP_Q_READY    (1 << 1)    // q_temp 中有新的温度数据
#define BIT_FLAME_Q_READY   (1 << 2)    // q_polar 中有新的火焰传感器数据
#define BIT_LOG_Q_READY     (1 << 3)    // q_log 中有新的一帧日志数据
#define BIT_MANUAL_MODE     (1 << 4)    // 0=自动模式(APF), 1=手动模式(WebSocket)

// 引脚分配

#define WS2812_PIN          48  // 开发板WS2812灯珠数据引脚
// 电机引脚已随驱动迁移至 drivers/drv8833/port/drv8833_port_def.h
// LD14P 引脚与串口配置已随驱动迁移至 drivers/ld14p/port/ld14p_port_def.h
// 注意；修改该参数同时应修改`flame_task.c`中的`flame_sensors`
#define FLAME_GPIO_MASK     ((1ULL << 16) | (1ULL << 14) | (1ULL << 13) | (1ULL << 12) | (1ULL << 11))
#define DS18B20_PIN         9   // TEMP-DQ

/**
上述为临时设置，下面是正常设置（匹配最新PCB）
WS2812_PIN          48  // 开发板WS2812灯珠数据引脚
FLAME_GPIO_MASK     ((1ULL << 4) | (1ULL << 5) | (1ULL << 6) | (1ULL << 7) | (1ULL << 8))
DS18B20_PIN         9   // TEMP-DQ
 */

// 激光雷达后处理配置（LD14P 线协议参数已迁移至 drivers/ld14p/）

#define LIDAR_SHIFT_DEG     -5.0f   // 雷达测量值偏移，叠加后得到实际值
#define LIDAR_SECTORS       72      // 降采样后的点数，请确保该值是360的因数
#define LIDAR_MIN_WEIGHT    2 - 1   // 区间最小值权重，在降采样时对区间最小值会加权

// 火焰传感器配置

#define FLAME_SENSOR_COUNT  5       // 火焰传感器路数
#define FLAME_DETECT_MM     400.0f  // 火焰信号转换的距离

// DS18B20 温度传感器配置

#define DS18B20_RES_BITS    10  // 10-bit 精度 (0.25°C, 188ms 转换), 支持 4Hz

// APF 人工势场法参数，距离单位统一为 mm

#define Q_POLAR_DEPTH   (LIDAR_SECTORS + FLAME_SENSOR_COUNT)    // 传感器数据统一队列
#define APF_RANGE_MAX   4000.0f     // 最大感知范围，超出视为噪声
#define APF_RANGE_REP   400.0f      // 斥力归一化锚点 (r = r_ref 时力 = K)
#define APF_RANGE_MIN   100.0f      // 死区距离 (避免 1/0 发散)
#define APF_GAIN_REP_X  160.0f      // 斥力 X 增益
#define APF_GAIN_REP_Y  100.0f      // 斥力 Y 增益 (r = r_ref 处的转向分量)
#define APF_REP_NX      0.6f        // X 衰减指数 (1/r^n, 越大近距离制动越猛)
#define APF_REP_NY      0.8f        // Y 衰减指数 (1/r^n, 越小远距离 转向越灵敏)
#define APF_ATT_BASE    1200.0f     // VFH 引力基础增益 (× 通道宽度缩放)

// VFH (Vector Field Histogram) 参数

#define VFH_BINS    LIDAR_SECTORS   // 72 bins, 5°/bin
#define VFH_THRESH_MM   1600.0f      // 障碍判定阈值 (<此值视为不可通行)
#define VFH_MIN_WIDTH   3           // 最小有效通道宽度 (4 bins ≈ 20°)
#define VFH_SMOOTH_W    1.50f       // 直方图平滑权重 (3点加权移动平均)
#define VFH_IS_FREE_TH  0.40f       // 平滑直方图低于此值视为可通行
#define VFH_GOAL_BIAS   0.60f       // 正前方偏好 (0=无偏好, 1=强偏好)
#define VFH_EMA_ALPHA   0.80f       // 角度 EMA 平滑 (0=纯惯性, 1=无平滑)

// APF+VFH 可调参数结构体 (运行时由 ws_handler 修改, NVS 持久化)
// APF_RANGE_MAX / APF_RANGE_MIN 为安全边界, 不参与热调

typedef struct {
    float range_rep;        // APF_RANGE_REP
    float gain_rep_x;       // APF_GAIN_REP_X
    float gain_rep_y;       // APF_GAIN_REP_Y
    float rep_nx;           // APF_REP_NX
    float rep_ny;           // APF_REP_NY
    float att_base;         // APF_ATT_BASE
    float vfh_thresh;       // VFH_THRESH_MM
    int   vfh_min_w;        // VFH_MIN_WIDTH (int)
    float vfh_smooth_w;     // VFH_SMOOTH_W
    float vfh_free_th;      // VFH_IS_FREE_TH
    float vfh_goal_bias;    // VFH_GOAL_BIAS
    float vfh_ema_alpha;    // VFH_EMA_ALPHA
} apf_params_t;

extern apf_params_t g_apf_params;

// 电机控制任务频率（控制频率为传感器工作频率的两倍）
// 其余电机配置已迁移至 drivers/drv8833/（drv8833_config.h + port_def.h）

#define MOTOR_FREQ_HZ       SENSOR_FREQ * 2     // 电机控制任务频率，为传感器工作频率的两倍
