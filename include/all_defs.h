#pragma once
#include "esp_err.h"
#include <stdbool.h>

/* ============================================================
 *  LD14P 激光雷达 公共宏定义与数据结构
 * ============================================================ */

/* ---------- 点云数据结构 ---------- */

#define LD14P_POINTS_PER_REV  360

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
