/**
 * @file drv8833_types.h
 * @brief DRV8833 公共类型定义：错误码、底盘类型、数据结构、实例句柄
 *
 * 核心层与移植层均可见，用户可直接调用。
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "drv8833_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 错误码 */
typedef enum {
  DRV8833_OK = 0U,
  DRV8833_ERR_PARAM,        /* 参数非法 */
  DRV8833_ERR_INIT,         /* 初始化失败 */
  DRV8833_ERR_NOT_SUPPORTED /* 底盘类型未实现 */
} drv8833_err_t;

/* 底盘类型（运动学算法选择，见 drv8833_config.h 的 DRV8833_CHASSIS_SELECT） */
typedef enum {
  DRV8833_CHASSIS_DIFF_4WD = 0, /* 四轮差速（同侧并联，当前实现） */
  DRV8833_CHASSIS_DIFF_2WD,     /* 两轮差速（预留） */
  DRV8833_CHASSIS_MECANUM,      /* 麦克纳姆轮（预留） */
  DRV8833_CHASSIS_TRACK,        /* 履带（预留） */
} drv8833_chassis_type_t;

/* 目标速度指令（笛卡尔，mm） */
typedef struct {
  float dx; /* 前向分量 */
  float dy; /* 侧向分量 */
} drv8833_vel_t;

/* 初始化配置；cfg 为 NULL 时全部使用 drv8833_config.h 默认值 */
typedef struct {
  drv8833_chassis_type_t chassis; /* 底盘类型，0=DRV8833_CHASSIS_SELECT */
  int8_t lpf_enabled;             /* LPF 开关：-1=默认(config), 0=关, 1=开 */
  float  lpf_alpha;               /* LPF α，0=默认 */
  uint32_t timeout_ms;            /* 指令超时(ms)，0=默认 */
  float    vel_max_mm;            /* 速度归一化上限(mm)，0=默认 */
} drv8833_cfg_t;

/* 实例句柄：支持多实例，由调用方静态分配 */
typedef struct {
  void *bus; /* 平台外设句柄（移植层填充，LEDC 无句柄用哨兵） */
  bool  initialized;

  drv8833_chassis_type_t chassis; /* 当前底盘类型 */
  bool  lpf_enabled;              /* LPF 运行期开关 */
  float lpf_alpha;                /* LPF 系数 */
  uint32_t timeout_ms;            /* 指令超时 */
  float    vel_max_mm;            /* 速度归一化上限 */

  /* 控制状态 */
  drv8833_vel_t target;     /* 目标速度（set_velocity 写入） */
  drv8833_vel_t filtered;   /* LPF 输出 */
  uint32_t last_cmd_ms;     /* 最近指令时间戳（超时检测） */
} drv8833_handle_t;

#ifdef __cplusplus
}
#endif
