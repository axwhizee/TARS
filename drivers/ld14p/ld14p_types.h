/**
 * @file ld14p_types.h
 * @brief LD14P 公共类型定义：错误码、数据结构、实例句柄
 *
 * 核心层与移植层均可见，用户可直接调用。
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ld14p_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 错误码 */
typedef enum {
  LD14P_OK = 0U,
  LD14P_ERR_PARAM,      /* 参数非法 */
  LD14P_ERR_NOT_READY,  /* 数据未就绪（帧不完整 / 圈未完成 / 无数据） */
  LD14P_ERR_FRAME,      /* 帧结构错误（VerLen 不匹配） */
  LD14P_ERR_CRC,        /* CRC8 校验失败 */
  LD14P_ERR_TIMEOUT,    /* 通信超时 */
  LD14P_ERR_UART,       /* UART 传输错误 */
  LD14P_ERR_INIT,       /* 初始化失败 */
} ld14p_err_t;

/* 驱动自有极坐标点：dst=距离(mm)，ang=角度(°)，0°=正前方顺时针递增 */
typedef struct {
  float dst;
  float ang;
} ld14p_polar_t;

/* 单点原始数据，distance==0 视为无效点 */
typedef struct __attribute__((packed)) {
  uint16_t distance;    /* 距离 (mm)，LSB 在前 */
  uint8_t  intensity;   /* 反射强度 0~255 */
} ld14p_point_t;

/* 完整数据帧 47B，直接映射 LD14P 线协议（ESP32 LE = LSB-first） */
typedef struct __attribute__((packed)) {
  uint8_t       header;      /* [0]   0x54 */
  uint8_t       ver_len;     /* [1]   0x2C */
  uint16_t      speed;       /* [2..3]   转速 (°/s) */
  uint16_t      start_angle; /* [4..5]   起始角度 (×0.01°) */
  ld14p_point_t points[LD14P_POINTS_FRAME]; /* [6..41] 12 采样点 */
  uint16_t      end_angle;   /* [42..43]  结束角度 */
  uint16_t      timestamp;   /* [44..45]  时间戳 (ms) */
  uint8_t       crc8;        /* [46]     CRC8 */
} ld14p_frame_t;
_Static_assert(sizeof(ld14p_frame_t) == LD14P_FRAME_LEN, "ld14p_frame_t must be exactly 47 bytes");

/* 初始化配置；字段为 0 时使用 ld14p_config.h 中的默认值 */
typedef struct {
  uint16_t target_freq_hz;  /* 目标扫描频率 (2~8 Hz)，0=默认 */
  uint32_t timeout_ms;      /* 通信超时 (ms)，0=默认 */
} ld14p_cfg_t;

/* 实例句柄：支持多实例，由调用方静态分配 */
typedef struct {
  void *bus;  /* 平台外设句柄（UART 端口号，由移植层填充） */
  bool  initialized;
  uint32_t timeout_ms;

  uint8_t parse_state;  /* 帧解析状态机：0=IDLE，1=拼帧中 */
  uint8_t parse_buf[LD14P_FRAME_LEN];
  uint8_t parse_idx;

  ld14p_polar_t cloud[LD14P_POINTS_ALL]; /* 点云累积（一整圈） */
  float         prev_last_deg;    /* 上一帧末点角度，跨零检测用 */
  uint32_t      rev_last_tick_ms; /* 最近一次圈完成时间戳 */
  bool          rev_ready;        /* 一圈完成待消费标志 */
  uint16_t      freq_hz;          /* 当前目标扫描频率 */
} ld14p_handle_t;

#ifdef __cplusplus
}
#endif
