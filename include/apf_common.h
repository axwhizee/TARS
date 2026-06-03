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

// WiFi 与全局配置

#define WIFI_SSID           "APF-NVC"   // 未加密的开发 AP
#define WEBSOCKET_PORT      80      // HTTP 协议的默认端口
// 传感器同步频率，直接决定温度/火焰传感器采样频率，间接影响APF法处理频率，取决于雷达频率（2~8Hz）
#define SENSOR_FREQ         4

// 极坐标向量（统一传感器数据）
typedef struct {
    float distance;     // 带显式角度的输出结构 (公用数据类型)
    float angle;    // 传感器正前方为 0°, 顺时针递增 (LD14P 原生坐标系)
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
extern vector_cart_t        g_cart_cmd; // 最新 APF 合力指令 (web_task 绕过争用直接读)

// 任务间同步事件组

#define BIT_LIDAR_Q_READY   (1 << 0)    // q_polar 中有新的激光雷达数据
#define BIT_TEMP_Q_READY    (1 << 1)    // q_temp 中有新的温度数据
#define BIT_FLAME_Q_READY   (1 << 2)    // q_polar 中有新的火焰传感器数据
#define BIT_LOG_Q_READY     (1 << 3)    // q_log 中有新的一帧日志数据
#define BIT_MANUAL_MODE     (1 << 4)    // 0=自动模式(APF), 1=手动模式(WebSocket)

// 引脚分配

#define WS2812_PIN          48  // 开发板WS2812灯珠数据引脚
#define MOTOR_AIN1_PIN      5   // AIN1
#define MOTOR_AIN2_PIN      6   // AIN2
#define MOTOR_BIN1_PIN      7   // BIN1
#define MOTOR_BIN2_PIN      15  // BIN2
#define LD14P_UTX_PIN       17  // UTX，接LD14P-RX
#define LD14P_URX_PIN       18  // URX，接LD14P-TX
// 注意；修改该参数同时应修改`flame_task.c`中的`flame_sensors`
#define FLAME_GPIO_MASK     ((1ULL << 16) | (1ULL << 14) | (1ULL << 13) | (1ULL << 12) | (1ULL << 11))
#define DS18B20_PIN         9   // TEMP-DQ

// LD14P连线指南：1(Red)<->UTX, 2(Black)<->GND, 3(Yellow)<->URX, 4(Green)<->VCC

/**
临时方案：
1. 电机：
    * 4 -> 5
    * 5 -> 6
    * 6 -> 7
    * 7 -> 15
2. 雷达：
    * 17 -> 由MCU|17飞线到PCB|03
    * 18 -> 由MCU|18飞线到PCB|43
3. 火焰：
    * 11
    * 12
    * 13
    * 14
    * 16 -> PCB|16飞线到飞线到PCB|18，暂不执行
4. 温度：
    * 9
MCU的3、46引脚已经被剪断，避免干扰电路
 */
/**
原始设置，因为PCB设计失误，暂放弃，修复方案：
LED_PIN             48  // 开发板LED
MOTOR_AIN1_PIN      11  // AIN1
MOTOR_AIN2_PIN      12  // AIN2
MOTOR_BIN1_PIN      13  // BIN1
MOTOR_BIN2_PIN      14  // BIN2
LD14P_UTX_PIN       17  // UTX，接LD14P-RX
LD14P_URX_PIN       18  // URX，接LD14P-TX
FLAME_GPIO_MASK     ((1ULL << 4) | (1ULL << 5) | (1ULL << 6) | (1ULL << 7) | (1ULL << 8))
DS18B20_PIN         9   // TEMP-DQ
 */

// LD14P 配置

#define LD14P_POINTS_PER_PACK   12          // 每个雷达数据帧包含的点数量
#define LD14P_POINTS_PER_REV    360         // 雷达完整一周的点云数量
#define LIDAR_SECTORS           72          // 降采样后的点数，请确保该值是360的因数
#define LIDAR_MIN_WEIGHT        2 - 1       // 区间最小值权重，在降采样时对区间最小值会加权
#define LD14P_UART_NUM          UART_NUM_1  // 通信串口
#define LD14P_UART_BAUD         115200      // 波特率
#define LD14P_UART_RX_BUF       2048        // 接收缓冲区大小

// 火焰传感器配置

#define FLAME_SENSOR_COUNT      5       // 火焰传感器路数
#define FLAME_DETECT_MM         500.0f  // 火焰信号转换的距离

// DS18B20 温度传感器配置

#define DS18B20_RES_BITS    10  // 10-bit 精度 (0.25°C, 188ms 转换), 支持 4Hz

// APF 人工势场法参数，距离单位统一为 mm

#define Q_POLAR_DEPTH   (LIDAR_SECTORS + FLAME_SENSOR_COUNT)    // 传感器数据统一队列
#define APF_SAFE_RANGE      4000.0f     // 安全感知范围，超出视为噪声。由于实际雷达数据偏小，应小于6000
#define APF_DANGER_RANGE    800.0f      // 危险区阈值
#define APF_PERCEPTION_MIN  100.0f      // 最小感知（死区）距离（避免自身/地面对 1/r² 的无穷大）
#define APF_ATTRACT_GAIN    4000.0f     // 引力增益 (K_att)，目前仅用于产生前向行进引力，影响小车的速度
// #define APF_REPULSE_GAIN    APF_ATTRACT_GAIN * APF_DANGER_RANGE / 14  // 斥力增益 (K_rep)
#define APF_REPULSE_GAIN    40.0f       // 斥力增益 (K_rep)，较高则曲线更加极端，近距离更敏感，远距离不敏感
#define APF_DANGER_RE_WT    1.2f        // 危险区斥力权重倍率
#define APF_SAFE_RE_WT      0.8f        // 感知区斥力权重倍率
#define APF_OPEN_GAIN       (APF_ATTRACT_GAIN * 0.3f)  // 开阔方向引力 (补充前向力)

// 电机驱动配置

#define MOTOR_PWM_FREQ      20000       // PWM 频率 20kHz
#define MOTOR_PWM_RES_BITS  10          // 定时器计数器位数（分辨率）
#define MOTOR_MAX_COUNT     ((1U << MOTOR_PWM_RES_BITS) - 1)    // 计数器大小（2^BITS - 1）
#define MOTOR_MIN_DUTY      75          // 能克服电机静摩擦力的最低有效占空比，需根据实测情况设置
#define MOTOR_MIN_COUNT     (MOTOR_MAX_COUNT * MOTOR_MIN_DUTY / 100U)    // 占空比对应的计数器值

#define MOTOR_FREQ_HZ       SENSOR_FREQ * 2     // 电机控制任务频率，为传感器工作频率的两倍
#define MOTOR_EMA_TAU_MS    50          // EMA时间常数 τ (ms), 控制平滑收敛速度
// α = 1 - exp(-dt/τ),  dt = 1000/MOTOR_FREQ_HZ,  τ = MOTOR_EMA_TAU_MS
// α 越大响应越快 (τ 越小时 α 越接近 1)
#define MOTOR_EMA_ALPHA     (1.0f - expf(-(float)(1000 / MOTOR_FREQ_HZ) / MOTOR_EMA_TAU_MS))
#define MOTOR_MAX_MM        APF_SAFE_RANGE      // 输出幅值上限，用于向量归一化，影响平均速度
#define MOTOR_TIMEOUT_MS    1000    // 指令超时阈值，超时停车
#define MOTOR_DEADZONE_MM   80.0f   // 笛卡控制尔死区
#define MOTOR_TURN_RATIO    0.25f   // 差速控制中角分量的基础灵敏度系数
#define MOTOR_TURN_GAIN_MI  0.5f    // 差速控制中角分量的动态灵敏度下限
#define MOTOR_TURN_GAIN_MX  2.0f - MOTOR_TURN_GAIN_MI   // 差速控制中角分量的动态灵敏度上限
