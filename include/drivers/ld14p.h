/**
 * @file ld14p.h
 * @brief LD14P 激光雷达驱动
 *
 * 内部类型 (调用者不需要直接操作):
 *   ld14p_point_t — 单个采样点 (距离 + 强度)
 *   ld14p_frame_t — 一个 47 字节的线帧 (packed, 直接映射协议)
 *
 * 公开 API:
 *   1. ld14p_init(4)             — 初始化 UART1 + 发送 0xA2 频率命令
 *   2. ld14p_feed_byte(byte)     — 喂入一个字节, 拼出完整帧后 CRC 验证,
 *                                  返回帧指针 (NULL = 尚未完成或 CRC 失败)
 *   3. ld14p_process_frame(frm)  — 解析帧: 角度插值 → 写入 cloud_360[],
 *                                  检测完整一圈 → 返回 true
 *   4. ld14p_get_cloud(out)      — 快照 cloud_360[] → vector_polar_t[360],
 *                                  返回有效点数
 */
#pragma once
#include "all_defs.h"


/* ────────── 内部数据类型 ────────── */

typedef struct __attribute__((packed)) {
    uint16_t distance;       /* 距离值 (毫米), LSB 在前, 0xFFFF 表示未填充 */
    uint8_t  intensity;      /* 反射强度 (0~255) */
} ld14p_point_t;

typedef struct __attribute__((packed)) {
    uint8_t       header;                            /* [0]  0x54 */
    uint8_t       ver_len;                           /* [1]  0x2C */
    uint16_t      speed;                             /* [2..3] 转速 (°/s) */
    uint16_t      start_angle;                       /* [4..5] 起始角度 (×0.01°) */
    ld14p_point_t points[LD14P_POINTS_PER_PACK];     /* [6..41] 12 个采样点 */
    uint16_t      end_angle;                         /* [42..43] 结束角度 */
    uint16_t      timestamp;                         /* [44..45] 时间戳 (ms) */
    uint8_t       crc8;                              /* [46] CRC8 */
} ld14p_frame_t;

/* ────────── 公开 API ────────── */

esp_err_t ld14p_init();

/*
 * ld14p_feed_byte — 喂入一个字节
 *
 * 内部: 状态机搜帧头 → 拼 47B → VerLen 检查 → CRC8 验证.
 * 返回: 通过验证的帧指针, 或 NULL (尚未拼完 / 校验失败).
 * 指针有效期: 至下一次本函数调用.
 */
const ld14p_frame_t *ld14p_feed_byte(uint8_t byte);

/*
 * ld14p_process_frame — 解析一帧, 更新点云
 *
 * 角度插值 → 更新 cloud_360[] → 检测 360° 圈完成.
 * 返回 true 表示 cloud_360[] 刚完成一整圈数据.
 */
bool ld14p_process_frame(const ld14p_frame_t *frm);

/*
 * ld14p_get_cloud — 获取当前 360° 点云快照
 *
 * 将内部 cloud_360[] 转为 vector_polar_t[360], 返回有效点数 (< 60000 mm).
 * out[i].angle_deg = i, out[i].distance_mm = 距离或 65535 (无效).
 */
uint32_t ld14p_get_cloud(vector_polar_t out[LD14P_POINTS_PER_REV]);
