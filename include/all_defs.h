/**
 * @file all_defs.h
 * @brief 项目公共定义、数据结构和RTOS句柄
 */
#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include <stdbool.h>
#include <stdint.h>


/**
 * vector_polar_t — 带显式角度的输出结构 (公用数据类型)
 * angle_deg: 传感器正前方为 0°, 顺时针递增 (LD14P 原生坐标系)
 */
typedef struct {
    float distance_mm;
    float angle_deg;
} vector_polar_t;

// ==================== 笛卡尔坐标向量（APF计算结果） ====================
typedef struct {
    float x;
    float y;
} vector_cart_t;

/* ---------- RTOS 全局句柄 (定义在 main.c, extern 供所有任务引用) ---------- */

extern QueueHandle_t        q_polar;    /* LD14P 数据队列 */
extern QueueHandle_t        q_cart;     /* APF 计算结果队列，深度 1 */
extern QueueHandle_t        q_temp;     /* DS18B20 温度数据队列 */
extern EventGroupHandle_t   eg_sync;    /* 传感器同步事件组 */

/* ---------- 任务间同步 ---------- */

#define BIT_LIDAR_READY       (1 << 0) /* APF 任务的 lidar 数据就绪位 */
#define BIT_TEMP_READY        (1 << 1) /* DS18B20 温度数据就绪位 */
#define BIT_FLAME_READY       (1 << 2) /* 火焰传感器数据就绪位 */

/* ---------- 板级硬件配置 ---------- */

#define LED_PIN              48

/* ---------- LD14P 数据协议 ---------- */

// #define LD14P_TASK_FREQ       (uint8_t)4
#define LD14P_POINTS_PER_PACK   12
#define LD14P_POINTS_PER_REV    360
#define LIDAR_SECTORS           36       /* 降采样: 每个扇区 10°, 共 36 个输出点 */
#define LIDAR_FREQ              4

#define LD14P_UART_NUM       UART_NUM_1
#define LD14P_UART_BAUD      115200
#define LD14P_UART_TX_PIN    17 // <-> R <-> RX
#define LD14P_UART_RX_PIN    18 // <-> Y <-> TX
#define LD14P_UART_RX_BUF    2048

// LD14P：1(R)->RX, 2(B)->GND, 3(Y)->TX, 4(G)->VCC

/* ---------- 火焰传感器 ---------- */

#define FLAME_SENSOR_COUNT      5
#define FLAME_DETECT_MM        500.0f  /* 检测到火焰时的虚拟距离 (进入危险区) */
#define FLAME_GPIO_MASK        ((1ULL << 11) | (1ULL << 12) | (1ULL << 13) | (1ULL << 14) | (1ULL << 15))

/* ---------- Q_POLAR 队列容量 (LiDAR + 火焰传感器) ---------- */

#define Q_POLAR_DEPTH           (LIDAR_SECTORS + FLAME_SENSOR_COUNT)  /* 36 + 5 = 41 */

/* ---------- DS18B20 温度传感器 ---------- */

#define DS18B20_GPIO_PIN      9       /* DQ 数据线 (1-Wire) */
#define DS18B20_RES_BITS      10      /* 10-bit 精度 (0.25°C, 188ms 转换), 支持 4Hz */
#define DS18B20_TASK_FREQ     4       /* 采样频率 (Hz) */

/* ---------- APF 人工势场参数 ---------- */

#define APF_SAFE_RANGE_MM      6000.0f  /* 安全感知范围 (6m), 超出视为噪声 */
#define APF_DANGER_RANGE_MM    1000.0f  /* 危险区阈值 (2m) */
#define APF_PERCEPTION_MIN_MM  100.0f   /* 最小感知距离 (避免自身/地面对 1/r² 的无穷大) */
#define APF_MAX_FORCE_MM       6000.0f  /* 合力输出幅值上限 (±6m) */
#define APF_MIN_FORCE_MM       100.0f   /* 合力输出死区, 小于此值归零 */

#define APF_ATTRACT_GAIN       1500.0f  /* 引力增益 (K_att), 产生向前的恒定拉力 */
#define APF_REPULSE_GAIN       8000.0f  /* 斥力增益 (K_rep) */
#define APF_DANGER_REPULSE_WT  2.5f     /* 危险区斥力权重倍率 (0-2m) */
#define APF_SAFE_REPULSE_WT    1.0f     /* 感知区斥力权重倍率 (2-6m) */

/* ---------- 电机驱动硬件配置 ---------- */

#define MOTOR_LEFT_IN1_GPIO    4        /* 左侧 IN1 (AIN1), PWM */
#define MOTOR_LEFT_IN2_GPIO    5        /* 左侧 IN2 (AIN2), Level */
#define MOTOR_RIGHT_IN1_GPIO   6        /* 右侧 IN1 (BIN1), PWM */
#define MOTOR_RIGHT_IN2_GPIO   7        /* 右侧 IN2 (BIN2), Level */

#define MOTOR_PWM_FREQ         20000    /* PWM 频率 20kHz (高于人耳听觉范围) */
#define MOTOR_PWM_RES_BITS     10       /* 10-bit 分辨率 (0-1023) */
#define MOTOR_MAX_DUTY         (((1U << MOTOR_PWM_RES_BITS) - 1) * 80U / 100U) /* 80% = 818 */

#define MOTOR_CTL_HZ            8       /* 电机控制任务运行频率 (Hz) */
#define MOTOR_CTL_PERIOD_MS     (1000 / MOTOR_CTL_HZ)  /* 125ms */

/**
 * EMA 时间常数 τ = 300ms, 控制平滑收敛速度
 * α = 1 - exp(-dt/τ) = 1 - exp(-125/300) ≈ 0.34
 * 若改为 16Hz (dt=62.5ms): α = 1 - exp(-62.5/300) ≈ 0.19
 */
#define MOTOR_EMA_TC_MS         150.0f
#define MOTOR_EMA_ALPHA         0.34f   /* α = 1 - exp(-MOTOR_CTL_PERIOD_MS / MOTOR_EMA_TC_MS) */

#define MOTOR_INPUT_TIMEOUT_MS  1000    /* 上游断流超过 1s → 目标归零, 自然停车 */

#define MOTOR_DEADZONE_MM       50.0f   /* 笛卡尔死区 (mm), 小于此值视为零指令 */
#define MOTOR_TURN_THRESHOLD    0.08f   /* |angular/linear| 超过此阈值进入转向状态 */
#define MOTOR_TURN_RATIO        0.7f    /* 差速转向中角分量的灵敏度权重 */
#define MOTOR_DEADTIME_MS       20      /* 换向死区制动保持时长 (ms), 防止电流冲击 */
#define MOTOR_CMD_QUEUE_DEPTH   1       /* 笛卡尔指令队列深度 (xQueueOverwrite 要求=1) */

// #define DEBUG
