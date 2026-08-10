/**
 * @file ld14p_config.h
 * @brief LD14P 驱动库配置 — 协议编码公式、参数范围、RTOS 适配开关
 *
 * 核心层与移植层均可见，移植时按目标平台调整本文件。
 */
#pragma once

/* ---- RTOS 适配开关 ----
 * 1: 移植层使用 RTOS 版本 delay/tick（本项目为 FreeRTOS）
 * 0: 移植层使用裸机版本（esp_timer / esp_rom_delay_us）
 */
#define LD14P_RTOS_ACTIVE   1

/* ---- 协议编码公式 ---- */
#define LD14P_FRAME_HEADER      0x54    // 帧头固定字节
#define LD14P_VER_LEN           0x2C    // 版本+长度字段（帧类型=1，点数=12）
#define LD14P_CMD_SPEED         0xA2    // 转速控制命令字节
#define LD14P_FRAME_LEN         47      // 完整数据帧字节数
#define LD14P_POINTS_FRAME      12      // 每帧采样点数
#define LD14P_POINTS_ALL        360     // 一整圈点云点数
#define LD14P_ANGLE_RES         100     // 角度精度 0.01°/LSB → ° 换算系数
#define LD14P_REV_DEBOUNCE_MS   150     // 圈检测防抖时间（ms）
#define LD14P_LASER_TAN         0.11923f  // tan(6.8°)，激光器固定夹角（SlTransform 用）

/* ---- 默认参数范围 ---- */
#define LD14P_FREQ_HZ_MIN       2       // 目标扫描频率下限（Hz）
#define LD14P_FREQ_HZ_MAX       8       // 目标扫描频率上限（Hz）
#define LD14P_FREQ_HZ_DEF       4       // 默认目标扫描频率（Hz）
#define LD14P_TIMEOUT_MS_DEF    100     // 默认通信超时（ms）
