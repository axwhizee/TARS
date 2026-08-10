/**
 * @file drv8833_port_def.h
 * @brief 移植目标硬件资源定义 — 连接驱动库与平台 HAL 驱动定义
 *
 * 注意：本文件包含平台 HAL 头文件，仅供移植层 (port/) 引用，
 *       核心层禁止包含本文件（避免 HAL 污染核心层）。
 */
#pragma once
#include "driver/ledc.h"

/* LEDC 资源（ESP32-S3） */
#define DRV8833_LEDC_MODE       LEDC_LOW_SPEED_MODE  // ESP32-S3 无 HIGH_SPEED_MODE
#define DRV8833_LEDC_CLK_CFG    LEDC_USE_APB_CLK     // 显式 80MHz APB 时钟
#define DRV8833_TIMER_A         LEDC_TIMER_0         // 左电机定时器
#define DRV8833_TIMER_B         LEDC_TIMER_1         // 右电机定时器
#define DRV8833_CH_AIN1         LEDC_CHANNEL_0
#define DRV8833_CH_AIN2         LEDC_CHANNEL_1
#define DRV8833_CH_BIN1         LEDC_CHANNEL_2
#define DRV8833_CH_BIN2         LEDC_CHANNEL_3

/* 引脚映射（当前临时飞线方案；原始 PCB 用 GPIO 11,12,13,14，见 apf_common.h 注释） */
#define DRV8833_AIN1_PIN        5   // AIN1
#define DRV8833_AIN2_PIN        6   // AIN2
#define DRV8833_BIN1_PIN        7   // BIN1
#define DRV8833_BIN2_PIN        15  // BIN2
