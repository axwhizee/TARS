#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * all_defs.h — 项目公共定义
 */

/* ---------- LD14P 数据协议 ---------- */

#define LD14P_POINTS_PER_PACK 12
#define LD14P_POINTS_PER_REV  360
#define LIDAR_SECTORS         36       /* 降采样: 每个扇区 10°, 共 36 个输出点 */

/* ---------- 任务间同步 ---------- */

#define BIT_LIDAR_READY       (1 << 0) /* APF 任务的 lidar 数据就绪位 */

/*
 * vector_polar_t — 带显式角度的输出结构 (公用数据类型)
 * angle_deg: 传感器正前方为 0°, 顺时针递增 (LD14P 原生坐标系)
 */
typedef struct {
    float distance_mm;
    float angle_deg;
} vector_polar_t;

/* ---------- UART 硬件配置 ---------- */

#define LD14P_UART_NUM       UART_NUM_1
#define LD14P_UART_BAUD      115200
#define LD14P_UART_TX_PIN    17
#define LD14P_UART_RX_PIN    18
#define LD14P_UART_RX_BUF    2048

/* ---------- 日志上传配置 ---------- */

#define UPLOAD_HEADER        0xAA

/* ---------- 板级硬件配置 ---------- */

#define LED_PIN              48
