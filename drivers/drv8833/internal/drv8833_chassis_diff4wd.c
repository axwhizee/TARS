/**
 * @file drv8833_chassis_diff4wd.c
 * @brief 四轮差速底盘算法 — dx/dy 直接差速映射（同侧电机并联）
 *
 * 算法（与原 motor_task.c diff_control 一致）：
 *   propulsion = dx_u × scale(|dy_u|)     // |dy| 越大前进越慢，避免转向过快致雷达畸变
 *   steering   = dy_u × STEER_GAIN        // 转向差速与 dy 成正比
 *   left/right = propulsion ± steering
 *
 * 优势：无角度分解的非线性跳跃，转向比例始终可预测，调参直观。
 */
#include "internal/drv8833_chassis.h"
#include "drv8833.h"
#include <math.h>

/* 四轮差速控制参数 */
#define DIFF4WD_DX_COUPLING   0.20f   // 转向减速系数，值越高小车转向时速度越慢
#define DIFF4WD_STEER_GAIN    0.50f   // 差速转向增益
#define DIFF4WD_SPIN_GAIN     0.10f   // 原地旋转系数
#define DIFF4WD_REV_GAIN      0.40f   // 小车后退时的系数修正
#define DIFF4WD_DEAD_ZONE     0.01f   // 死区阈值 (归一化 [-1,1])

static drv8833_err_t drv8833_diff4wd_control(drv8833_handle_t *h,
                                             const drv8833_vel_t *vel) {
  /* 归一化到 [-1, 1] */
  float dx_u = vel->dx / h->vel_max_mm;
  float dy_u = vel->dy / h->vel_max_mm;

  /* |dy| 越大，前进分量衰减越多 */
  float scale = 1.0f - fabsf(dy_u) * DIFF4WD_DX_COUPLING;
  if (scale < 0.0f) scale = 0.0f;

  float propulsion = dx_u * scale;         /* 动力：被 dy 抑制的前进量 */
  float steering   = dy_u * DIFF4WD_STEER_GAIN;  /* 转向：直接反映 dy */

  /* 小车后退时减速 */
  if (propulsion < DIFF4WD_DEAD_ZONE) {
    propulsion *= DIFF4WD_REV_GAIN;
  }

  float left  = propulsion - steering;
  float right = propulsion + steering;

  /* 差速控制：x 分量小于 y 分量时进入原地旋转；旋转时削弱转速防抖 */
  if (left * right < 0) {
    left  *= DIFF4WD_SPIN_GAIN;
    right *= DIFF4WD_SPIN_GAIN;
  }

  /* 比例限幅（保持左右转向比） */
  float m = fmaxf(fabsf(left), fabsf(right));
  if (m > 1.0f) {
    left  /= m;
    right /= m;
  }

  /* 死区 */
  if (fabsf(left) < DIFF4WD_DEAD_ZONE && fabsf(right) < DIFF4WD_DEAD_ZONE) {
    return drv8833_set_speed(h, 0, 0);
  }

  return drv8833_set_speed(h, (int8_t)roundf(left * 100.0f),
                           (int8_t)roundf(right * 100.0f));
}

const drv8833_chassis_ops_t drv8833_chassis_diff4wd_ops = {
  .control = drv8833_diff4wd_control,
  .name = "diff_4wd",
};
