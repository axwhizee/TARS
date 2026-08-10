/**
 * @file drv8833.c
 * @brief DRV8833 核心层 — IN/IN 控制逻辑、占空比映射、一阶低通滤波、底盘分发
 *
 * 本文件不包含任何平台 HAL 头文件，平台相关操作全部经 port/ 抽象。
 * 底盘运动学算法按 DRV8833_CHASSIS_SELECT 分发至 internal/ 实现。
 */
#include "drv8833.h"
#include "drv8833_config.h"
#include "drv8833_types.h"
#include "port/drv8833_port.h"
#include "internal/drv8833_chassis.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "DRV8833";

/**
 * @brief 设置单侧电机转速比例（-100~100），已修正克服静摩擦力的占空比下限
 *
 * DRV8833 IN/IN 控制：
 *   pct>0: IN1=PWM, IN2=0 → 正转
 *   pct<0: IN1=0,   IN2=PWM → 反转
 *   pct=0: IN1=IN2=0 → 滑行
 *
 * @param in1_ch IN1 逻辑通道
 * @param in2_ch IN2 逻辑通道
 * @param pct    转速百分比 [-100,100]
 */
static void drv8833_set_side(drv8833_handle_t *h, uint8_t in1_ch, uint8_t in2_ch,
                             int8_t pct) {
  int8_t clamped = pct;
  if (clamped > 100) clamped = 100;
  if (clamped < -100) clamped = -100;

  if (clamped == 0) {
    drv8833_port_set_duty(h, in1_ch, 0);
    drv8833_port_set_duty(h, in2_ch, 0);
    return;
  }

  /* [1,100] 线性映射到 [MIN_DUTY, MAX_DUTY]（统一牵引力比例） */
  uint8_t abs_pct = (uint8_t)(clamped < 0 ? -clamped : clamped);
  uint32_t duty = DRV8833_MIN_DUTY
    + ((uint32_t)(abs_pct - 1) * (DRV8833_MAX_DUTY - DRV8833_MIN_DUTY)) / 99U;

  if (clamped > 0) {
    drv8833_port_set_duty(h, in1_ch, duty);
    drv8833_port_set_duty(h, in2_ch, 0);
  } else {
    drv8833_port_set_duty(h, in1_ch, 0);
    drv8833_port_set_duty(h, in2_ch, duty);
  }
}

/* 一阶低通滤波：y[n] = α·x[n] + (1-α)·y[n-1] */
static void drv8833_lpf_step(drv8833_handle_t *h) {
#if DRV8833_LPF_ENABLE
  const float alpha = h->lpf_alpha;
  const float alpha_inv = 1.0f - alpha;
  h->filtered.dx = alpha * h->target.dx + alpha_inv * h->filtered.dx;
  h->filtered.dy = alpha * h->target.dy + alpha_inv * h->filtered.dy;
#else
  h->filtered = h->target;   /* LPF 编译关闭 → 直通 */
#endif
}

drv8833_err_t drv8833_init(drv8833_handle_t *h, const drv8833_cfg_t *cfg) {
  if (h == NULL) {
    return DRV8833_ERR_PARAM;
  }
  if (h->initialized) {
    return DRV8833_ERR_PARAM;   /* 重复初始化 */
  }

  memset(h, 0, sizeof(*h));

  /* 应用配置（默认值填充） */
  h->chassis = DRV8833_CHASSIS_SELECT;
  h->lpf_alpha = DRV8833_LPF_ALPHA_DEF;
  h->timeout_ms = DRV8833_TIMEOUT_MS_DEF;
  h->vel_max_mm = DRV8833_VEL_MAX_MM_DEF;
#if DRV8833_LPF_ENABLE
  h->lpf_enabled = true;
#else
  h->lpf_enabled = false;
#endif
  if (cfg != NULL) {
    if (cfg->chassis != 0) h->chassis = cfg->chassis;
    if (cfg->lpf_enabled >= 0) h->lpf_enabled = (cfg->lpf_enabled == 1);
    if (cfg->lpf_alpha > 0.0f) h->lpf_alpha = cfg->lpf_alpha;
    if (cfg->timeout_ms > 0) h->timeout_ms = cfg->timeout_ms;
    if (cfg->vel_max_mm > 0.0f) h->vel_max_mm = cfg->vel_max_mm;
  }

  /* 底盘类型有效性（预留类型返回 NOT_SUPPORTED） */
  if (drv8833_chassis_ops(h->chassis) == NULL) {
    return DRV8833_ERR_NOT_SUPPORTED;
  }

  /* 移植层：配置 LEDC 定时器 + 通道 */
  drv8833_err_t err = drv8833_port_pwm_init(h);
  if (err != DRV8833_OK) {
    return err;
  }

  h->initialized = true;

  /* 初始化为滑行状态 */
  drv8833_coast(h);
  ESP_LOGI(TAG, "DRV8833 initialized (chassis=%d, lpf=%d, %dHz, %d-bit, min_duty=%d)",
           (int)h->chassis, (int)h->lpf_enabled, DRV8833_PWM_FREQ_HZ,
           DRV8833_PWM_RES_BITS, DRV8833_MIN_DUTY);
  return DRV8833_OK;
}

drv8833_err_t drv8833_deinit(drv8833_handle_t *h) {
  if (h == NULL) {
    return DRV8833_ERR_PARAM;
  }
  drv8833_coast(h);
  drv8833_err_t err = drv8833_port_pwm_deinit(h);
  h->initialized = false;
  return err;
}

drv8833_err_t drv8833_set_speed(drv8833_handle_t *h, int8_t left, int8_t right) {
  if (h == NULL || !h->initialized) {
    return DRV8833_ERR_PARAM;
  }
  drv8833_set_side(h, DRV8833_IN_AIN1, DRV8833_IN_AIN2, left);
  drv8833_set_side(h, DRV8833_IN_BIN1, DRV8833_IN_BIN2, right);
  return DRV8833_OK;
}

drv8833_err_t drv8833_brake(drv8833_handle_t *h) {
  if (h == NULL || !h->initialized) {
    return DRV8833_ERR_PARAM;
  }
  /* IN1=IN2=HIGH → 两路低侧 FET 导通 → 电机绕组短接 */
  const uint32_t full = DRV8833_MAX_DUTY;
  drv8833_port_set_duty(h, DRV8833_IN_AIN1, full);
  drv8833_port_set_duty(h, DRV8833_IN_AIN2, full);
  drv8833_port_set_duty(h, DRV8833_IN_BIN1, full);
  drv8833_port_set_duty(h, DRV8833_IN_BIN2, full);
  return DRV8833_OK;
}

drv8833_err_t drv8833_coast(drv8833_handle_t *h) {
  if (h == NULL || !h->initialized) {
    return DRV8833_ERR_PARAM;
  }
  /* 全部通道占空比 0 → Hi-Z */
  drv8833_port_set_duty(h, DRV8833_IN_AIN1, 0);
  drv8833_port_set_duty(h, DRV8833_IN_AIN2, 0);
  drv8833_port_set_duty(h, DRV8833_IN_BIN1, 0);
  drv8833_port_set_duty(h, DRV8833_IN_BIN2, 0);
  return DRV8833_OK;
}

drv8833_err_t drv8833_set_velocity(drv8833_handle_t *h, float dx, float dy) {
  if (h == NULL || !h->initialized) {
    return DRV8833_ERR_PARAM;
  }
  h->target.dx = dx;
  h->target.dy = dy;
  h->last_cmd_ms = drv8833_port_get_tick_ms();
  return DRV8833_OK;
}

drv8833_err_t drv8833_update(drv8833_handle_t *h) {
  if (h == NULL || !h->initialized) {
    return DRV8833_ERR_PARAM;
  }

  /* 超时保护 → 目标归零（自然停车） */
  uint32_t now = drv8833_port_get_tick_ms();
  if ((now - h->last_cmd_ms) > h->timeout_ms) {
    h->target.dx = 0.0f;
    h->target.dy = 0.0f;
  }

  /* 一阶低通滤波（可运行期开关） */
  if (h->lpf_enabled) {
    drv8833_lpf_step(h);
  } else {
    h->filtered = h->target;   /* 直通 */
  }

  /* 底盘算法分发 */
  const drv8833_chassis_ops_t *ops = drv8833_chassis_ops(h->chassis);
  if (ops == NULL) {
    return DRV8833_ERR_NOT_SUPPORTED;
  }
  return ops->control(h, &h->filtered);
}

drv8833_err_t drv8833_set_lpf(drv8833_handle_t *h, bool enable) {
#if DRV8833_LPF_ENABLE
  if (h == NULL || !h->initialized) {
    return DRV8833_ERR_PARAM;
  }
  h->lpf_enabled = enable;
  if (!enable) {
    h->filtered = h->target;   /* 关闭时直通，避免残留滤波值拖尾 */
  }
  return DRV8833_OK;
#else
  (void)h;
  (void)enable;
  return DRV8833_ERR_NOT_SUPPORTED;
#endif
}
