/**
 * @file ld14p.h
 * @brief LD14P 激光雷达驱动
 */
#pragma once
#include "all_defs.h"

#define FRAME_LEN        47

typedef struct __attribute__((packed)) {
    uint16_t distance;       // 距离值 (毫米), LSB 在前, 0xFFFF 表示未填充
    uint8_t  intensity;      // 反射强度 (0~255)
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
// 编译期断言：判断大小是否严格为47Byte
_Static_assert(sizeof(ld14p_frame_t) == FRAME_LEN, "ld14p_frame_t must be exactly 47 bytes");

/**
 * @brief 初始化函数
 * @return ESP_OK | ESP_FAIL
 */
esp_err_t ld14p_init();

/**
 * @brief 字节级状态机：喂入一个字节，状态机搜帧头 → 拼 47B → VerLen 检查 → CRC8 验证
 * @param byte UART读取的一帧
 * @return 通过验证的帧指针, 或 NULL (尚未拼完 / 校验失败).
 */
const ld14p_frame_t *ld14p_feed_byte(uint8_t byte);

/**
 * @brief 通过角度插值 + 圈检测，解析一帧, 更新点云
 * @param frm 读取的完整 LD14P 数据帧
 * @return true，表示完成完整一圈（上层应取点云）；false，表示解析完毕但未完成一圈
 */
bool ld14p_process_frame(const ld14p_frame_t *frm);

/**
 * @brief 获取当前 360° 点云快照，并返回有效点数
 * @param out 完整的 360 点雷达点云
 * @return 有效点数
 */
uint16_t ld14p_get_cloud(vector_polar_t (*out)[LD14P_POINTS_PER_REV]);
