/**
 * @file ld14p_port_def.h
 * @brief 移植目标硬件资源定义 — 连接驱动库与平台 HAL 驱动定义
 *
 * 注意：本文件包含平台 HAL 头文件，仅供移植层 (port/) 引用，
 *       核心层禁止包含本文件（避免 HAL 污染核心层）。
 */
#pragma once
#include "driver/uart.h"

/* UART 资源（ESP32-S3） */
#define LD14P_UART_NUM      UART_NUM_1      // 通信串口
#define LD14P_UART_BAUD     115200          // 波特率（实测模组为 115200，非手册 230400）
#define LD14P_UART_TX_PIN   17              // UTX，接 LD14P-RX（频率命令）
#define LD14P_UART_RX_PIN   18              // URX，接 LD14P-TX（数据接收）
#define LD14P_UART_RX_BUF   2048            // 接收缓冲区大小

/* 接线指南：1(Red)<->UTX, 2(Black)<->GND, 3(Yellow)<->URX, 4(Green)<->VCC(5V) */
