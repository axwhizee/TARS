/**
 * @file drv8833_config.h
 * @brief DRV8833 驱动库配置 — 底盘选择、一阶低通滤波、PWM/控制参数
 *
 * 核心层与移植层均可见，移植时按目标平台调整本文件。
 * 引脚/通道等硬件资源见 port/drv8833_port_def.h（本文件不含平台 HAL）。
 */
#pragma once

/* ---- RTOS 适配开关 ----
 * 1: 移植层使用 RTOS 版本 delay/tick（本项目为 FreeRTOS）
 * 0: 移植层使用裸机版本（esp_timer / esp_rom_delay_us）
 */
#define DRV8833_RTOS_ACTIVE   1

/* ---- 底盘类型选择 ----
 * 决定 drv8833_update() 使用哪套差速/运动学算法（见 internal/）：
 *   DRV8833_CHASSIS_DIFF_4WD  四轮差速（同侧并联，当前实现）
 *   DRV8833_CHASSIS_DIFF_2WD  两轮差速（预留，未实现）
 *   DRV8833_CHASSIS_MECANUM   麦克纳姆轮（预留，未实现）
 *   DRV8833_CHASSIS_TRACK     履带（预留，未实现）
 * 新增底盘：在 drv8833_types.h 扩展枚举、internal/ 新增控制文件、
 *           并在 internal/drv8833_chassis.c 注册即可，无需改动核心 API。
 */
#define DRV8833_CHASSIS_SELECT DRV8833_CHASSIS_DIFF_4WD

/* ---- 一阶低通滤波 (LPF) ----
 * 编译期总开关：0 时滤波代码不编译，update() 直通目标。
 * 运行期可用 drv8833_set_lpf() 或 cfg.lpf_enabled 单独启停（编译开关打开时）。
 */
#define DRV8833_LPF_ENABLE     1
#define DRV8833_LPF_ALPHA_DEF  0.90f   // 默认 α，越大响应越快，取 1 无平滑

/* ---- PWM 参数 ---- */
#define DRV8833_PWM_FREQ_HZ    20000   // PWM 频率 20kHz
#define DRV8833_PWM_RES_BITS   10      // 定时器计数器位数（分辨率）
#define DRV8833_MAX_DUTY       ((1U << DRV8833_PWM_RES_BITS) - 1)    // 计数器满值 (1023)
#define DRV8833_MIN_DUTY_PCT   75      // 能克服电机静摩擦的最低有效占空比 (%)
#define DRV8833_MIN_DUTY       (DRV8833_MAX_DUTY * DRV8833_MIN_DUTY_PCT / 100U)

/* ---- 控制参数默认值（cfg 为 0/负 时使用） ---- */
#define DRV8833_TIMEOUT_MS_DEF 500     // 指令超时阈值（ms），超时目标归零停车
#define DRV8833_VEL_MAX_MM_DEF 4000.0f // 速度幅值归一化上限（mm），应与 APF_RANGE_MAX 对齐

/* ---- DRV8833 逻辑输入通道索引（供核心层与底盘算法使用） ---- */
#define DRV8833_IN_AIN1  0
#define DRV8833_IN_AIN2  1
#define DRV8833_IN_BIN1  2
#define DRV8833_IN_BIN2  3
