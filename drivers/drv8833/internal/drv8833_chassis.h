/**
 * @file drv8833_chassis.h
 * @brief 底盘算法内部接口 — 各底盘运动学实现经此注册
 *
 * 本文件仅供核心层 (drv8833.c) 与底盘实现文件使用，不对外暴露。
 * 新增底盘流程（以麦克纳姆轮为例）：
 *   1. drv8833_types.h 扩展 drv8833_chassis_type_t 枚举
 *   2. 新建 internal/drv8833_chassis_mecanum.c 实现 control()
 *   3. 在本文件声明、drv8833_chassis.c 注册
 *   4. drv8833_config.h 设置 DRV8833_CHASSIS_SELECT
 */
#pragma once
#include "drv8833_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 底盘控制操作集 */
typedef struct {
  /**
   * @brief 底盘运动学：将滤波后的目标速度映射为左右轮转速并输出
   *
   * @param h    实例句柄
   * @param vel  滤波后目标速度（dx 前向 / dy 侧向，mm）
   * @return DRV8833_OK 成功；其他值失败
   */
  drv8833_err_t (*control)(drv8833_handle_t *h, const drv8833_vel_t *vel);
  const char *name;   /* 底盘名称（日志用） */
} drv8833_chassis_ops_t;

/**
 * @brief 获取指定底盘类型的控制操作集
 *
 * @param type 底盘类型
 * @return 操作集指针；类型未实现时返回 NULL
 */
const drv8833_chassis_ops_t *drv8833_chassis_ops(drv8833_chassis_type_t type);

#ifdef __cplusplus
}
#endif
