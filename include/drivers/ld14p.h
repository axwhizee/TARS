/**
 * @file ld14p.h
 * @brief LD14P 激光雷达驱动
 *
 * 调用链: init → parse → collect → [calibrate]
 */
#pragma once
#include "apf_common.h"

#define FRAME_LEN        47

// 单点数据, distance==0 视为无效
typedef struct __attribute__((packed)) {
    uint16_t distance;       // 距离 (mm), LSB 在前
    uint8_t  intensity;      // 反射强度 0~255
} ld14p_point_t;

// 完整数据帧 47B, 直接映射 LD14P 线协议 (ESP32 LE = LSB-first)
typedef struct __attribute__((packed)) {
    uint8_t       header;                            /* [0]  0x54 */
    uint8_t       ver_len;                           /* [1]  0x2C */
    uint16_t      speed;                             /* [2..3] 转速 (°/s) */
    uint16_t      start_angle;                       /* [4..5] 起始角度 (×0.01°) */
    ld14p_point_t points[LD14P_POINTS_FRAME];     /* [6..41] 12 采样点 */
    uint16_t      end_angle;                         /* [42..43] 结束角度 */
    uint16_t      timestamp;                         /* [44..45] 时间戳 (ms) */
    uint8_t       crc8;                              /* [46] CRC8 */
} ld14p_frame_t;
_Static_assert(sizeof(ld14p_frame_t) == FRAME_LEN, "ld14p_frame_t must be exactly 47 bytes");

/**
 * @brief 初始化 UART1, 发送频率命令, 清零点云
 * @return ESP_OK | ESP_FAIL
 */
esp_err_t ld14p_init();

/**
 * @brief 字节级状态机 — 逐个字节喂入, 搜帧头 0x54 拼 47B, VerLen + CRC8 双校
 * @param byte UART 读取的一个字节
 * @return 通过验证的完整帧指针, 或 NULL
 */
const ld14p_frame_t *ld14p_parse(uint8_t byte);

/**
 * @brief 角度插值写入点云 + 跨零点圈检测, 一圈完成时返回 cloud_360 指针
 * @param frm ld14p_parse() 返回的有效帧
 * @return 完成一圈时返回 cloud_360[] 指针 (调用者应立即快照消费), 否则 NULL
 */
const vector_polar_t *ld14p_collect(const ld14p_frame_t *frm);

/**
 * @brief SlTransform 校准: 激光器相对旋转中心的几何偏移角度修正 (in-place)
 * @param points  点云数组, 修改 ang 字段
 * @param count   点数
 * @param offset_x  X 轴偏移 (mm), 官方默认 5.9
 * @param offset_y  Y 轴偏移 (mm), 官方默认 -18.976
 */
void ld14p_calibrate(vector_polar_t *points,  float offset_x, float offset_y);
