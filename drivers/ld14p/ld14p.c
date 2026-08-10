/**
 * @file ld14p.c
 * @brief LD14P 激光雷达核心层 — 协议状态机、角度插值、圈检测、频率命令
 *
 * 本文件不包含任何平台 HAL 头文件，平台相关操作全部经 port/ 抽象。
 * 点云 cloud[LD14P_POINTS_ALL] 持久化于句柄，每圈自然覆盖不主动清零，
 * dst==0 标记无效点；圈检测用帧末点角度（借鉴官方 SDK），配合防抖时间。
 */
#include "ld14p.h"
#include "ld14p_config.h"
#include "ld14p_types.h"
#include "port/ld14p_port.h"
#include "internal/ld14p_utils.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "LD14P ";

ld14p_err_t ld14p_init(ld14p_handle_t *h, const ld14p_cfg_t *cfg) {
  if (h == NULL) {
    return LD14P_ERR_PARAM;
  }
  if (h->initialized) {
    return LD14P_ERR_PARAM;   /* 重复初始化 */
  }

  memset(h, 0, sizeof(*h));

  /* 应用配置（0 = 使用默认值） */
  h->timeout_ms = LD14P_TIMEOUT_MS_DEF;
  uint16_t freq = LD14P_FREQ_HZ_DEF;
  if (cfg != NULL) {
    if (cfg->target_freq_hz) {
      if (cfg->target_freq_hz < LD14P_FREQ_HZ_MIN || cfg->target_freq_hz > LD14P_FREQ_HZ_MAX) {
        return LD14P_ERR_PARAM;
      }
      freq = cfg->target_freq_hz;
    }
    if (cfg->timeout_ms) {
      h->timeout_ms = cfg->timeout_ms;
    }
  }

  /* 移植层：安装/配置 UART */
  ld14p_err_t err = ld14p_port_uart_init(h);
  if (err != LD14P_OK) {
    return err;
  }

  /* 等电机上电稳定 → 发频率命令 → 等电机响应 */
  ld14p_port_delay_ms(100);
  err = ld14p_set_freq(h, freq);
  if (err != LD14P_OK) {
    return err;
  }
  ld14p_port_delay_ms(200);

  h->initialized = true;
  ESP_LOGI(TAG, "LD14P initialized @ %d Hz", freq);
  return LD14P_OK;
}

ld14p_err_t ld14p_deinit(ld14p_handle_t *h) {
  if (h == NULL) {
    return LD14P_ERR_PARAM;
  }
  ld14p_err_t err = ld14p_port_uart_deinit(h);
  h->initialized = false;
  return err;
}

ld14p_err_t ld14p_set_freq(ld14p_handle_t *h, uint16_t freq_hz) {
  if (h == NULL) {
    return LD14P_ERR_PARAM;
  }
  if (freq_hz < LD14P_FREQ_HZ_MIN || freq_hz > LD14P_FREQ_HZ_MAX) {
    return LD14P_ERR_PARAM;
  }

  /* 0xA2 转速控制命令：speed = freq_hz * 360 (°/s) */
  uint16_t speed = (uint16_t)freq_hz * 360;
  uint8_t cmd[8] = {
    LD14P_FRAME_HEADER, LD14P_CMD_SPEED, 4,
    (uint8_t)(speed & 0xFF), (uint8_t)(speed >> 8),
    0x00, 0x00, 0x00
  };
  cmd[7] = ld14p_crc8(cmd, 7);

  ld14p_err_t err = ld14p_port_write(h, cmd, sizeof(cmd));
  if (err != LD14P_OK) {
    return err;
  }

  h->freq_hz = freq_hz;
  ESP_LOGI(TAG, "LD14P set freq @ %d Hz (%d deg/s)", freq_hz, speed);
  return LD14P_OK;
}

ld14p_err_t ld14p_feed_byte(ld14p_handle_t *h, uint8_t byte, const ld14p_frame_t **out_frame) {
  if (h == NULL || out_frame == NULL) {
    return LD14P_ERR_PARAM;
  }
  *out_frame = NULL;

  if (h->parse_state == 0) {          /* S_IDLE */
    if (byte == LD14P_FRAME_HEADER) {
      h->parse_buf[0] = LD14P_FRAME_HEADER;
      h->parse_idx = 1;
      h->parse_state = 1;             /* S_FRAME */
    }
    return LD14P_ERR_NOT_READY;
  }

  h->parse_buf[h->parse_idx++] = byte;
  if (h->parse_idx < LD14P_FRAME_LEN) {
    return LD14P_ERR_NOT_READY;
  }

  /* 帧拼满，复位状态机 */
  h->parse_state = 0;
  h->parse_idx = 0;

  if (h->parse_buf[1] != LD14P_VER_LEN) {
    return LD14P_ERR_FRAME;   /* 帧长标识不匹配 → 丢弃 */
  }
  if (ld14p_crc8(h->parse_buf, LD14P_FRAME_LEN - 1) != h->parse_buf[LD14P_FRAME_LEN - 1]) {
    return LD14P_ERR_CRC;   /* CRC 校验失败 → 丢弃 */
  }

  *out_frame = (const ld14p_frame_t *)h->parse_buf;
  return LD14P_OK;
}

ld14p_err_t ld14p_collect(ld14p_handle_t *h, const ld14p_frame_t *frm, const ld14p_polar_t **out_scan) {
  if (h == NULL || frm == NULL || out_scan == NULL) {
    return LD14P_ERR_PARAM;
  }
  *out_scan = NULL;

  /* 12 点在 start_angle~end_angle 间等间隔插值 */
  int diff = (int)frm->end_angle - (int)frm->start_angle;
  if (diff < 0) {
    diff += 360 * LD14P_ANGLE_RES;    /* 跨 0° 补偿 */
  }
  float step_raw = (float)diff / (LD14P_POINTS_FRAME - 1);

  float last_deg = 0.0f;
  for (int i = 0; i < LD14P_POINTS_FRAME; i++) {
    int raw_angle = (int)frm->start_angle + (int)(i * step_raw);
    int deg = (raw_angle / LD14P_ANGLE_RES) % LD14P_POINTS_ALL;
    float angle_f = (float)raw_angle / LD14P_ANGLE_RES;

    h->cloud[deg].ang = angle_f;
    h->cloud[deg].dst = (float)frm->points[i].distance;
    if (i == LD14P_POINTS_FRAME - 1) {
      last_deg = angle_f;
    }
  }

  /* 圈检测：末点角度跨过 0° 线（prev>340 且 current<20）+ 防抖 */
  uint32_t now = ld14p_port_get_tick_ms();
  if (h->prev_last_deg > 340.0f && last_deg < 20.0f
      && (now - h->rev_last_tick_ms) > LD14P_REV_DEBOUNCE_MS) {
    h->rev_ready = true;
    h->rev_last_tick_ms = now;
  }
  h->prev_last_deg = last_deg;

  if (h->rev_ready) {
    h->rev_ready = false;             /* 单次消费，阻止同一圈重复返回 */
    *out_scan = h->cloud;
    return LD14P_OK;
  }
  return LD14P_ERR_NOT_READY;
}

ld14p_err_t ld14p_process(ld14p_handle_t *h, const ld14p_polar_t **out_scan) {
  if (h == NULL || out_scan == NULL) {
    return LD14P_ERR_PARAM;
  }
  *out_scan = NULL;

  uint8_t buf[256];
  size_t got = 0;
  ld14p_err_t err = ld14p_port_read(h, buf, sizeof(buf), &got, 0);
  if (err != LD14P_OK) {
    return err;
  }
  if (got == 0) {
    return LD14P_ERR_NOT_READY;
  }

  for (size_t i = 0; i < got; i++) {
    const ld14p_frame_t *frm = NULL;
    if (ld14p_feed_byte(h, buf[i], &frm) != LD14P_OK) {
      continue;
    }

    const ld14p_polar_t *scan = NULL;
    if (ld14p_collect(h, frm, &scan) == LD14P_OK) {
      *out_scan = scan;
      return LD14P_OK;
    }
  }
  return LD14P_ERR_NOT_READY;
}

ld14p_err_t ld14p_calibrate(ld14p_polar_t *points, uint16_t count,
                            float offset_x, float offset_y) {
  if (points == NULL || count == 0) {
    return LD14P_ERR_PARAM;
  }

  /*
   * SlTransform（参照官方 SDK）:
   *   - 激光器偏离旋转中心 (offset_x, offset_y)，且发光方向有固定夹角
   *   - 公式: x = dist + offset_x
   *          y = dist * LASER_TAN + offset_y
   *          shift = atan2(y, x) * 180/π
   *          angle_corrected = angle_raw - shift（左手系）
   */
  float last_shift = 0.0f;

  for (uint16_t i = 0; i < count; i++) {
    float dist = points[i].dst;
    float angle = points[i].ang;
    float shift;

    if (dist > 0.0f) {
      float x = dist + offset_x;
      float y = dist * LD14P_LASER_TAN + offset_y;
      shift = atan2f(y, x) * 180.0f / 3.14159f;
      last_shift = shift;
    } else {
      shift = last_shift;             /* 无效点沿用上次有效 shift，避免角度跳变 */
    }

    angle -= shift;
    if (angle > 360.0f) {             /* 归一化到 [0, 360) */
      angle -= 360.0f;
    }
    if (angle < 0.0f) {
      angle += 360.0f;
    }
    points[i].ang = angle;
  }
  return LD14P_OK;
}

const ld14p_polar_t *ld14p_get_cloud(ld14p_handle_t *h) {
  if (h == NULL) {
    return NULL;
  }
  return h->cloud;
}
