/**
 * @file drv8833_chassis.c
 * @brief 底盘算法注册表 — 按底盘类型分发给对应实现
 */
#include "internal/drv8833_chassis.h"

/* 各底盘控制操作集（internal/ 下实现） */
extern const drv8833_chassis_ops_t drv8833_chassis_diff4wd_ops;

const drv8833_chassis_ops_t *drv8833_chassis_ops(drv8833_chassis_type_t type) {
  switch (type) {
    case DRV8833_CHASSIS_DIFF_4WD:
      return &drv8833_chassis_diff4wd_ops;
    /* 预留底盘（2WD / 麦克纳姆 / 履带）未实现 → NULL，init/update 返回 NOT_SUPPORTED */
    default:
      return NULL;
  }
}
