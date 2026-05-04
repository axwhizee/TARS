#pragma once
#include "esp_err.h"
#include <stdbool.h>

#define LD14P_POINTS_PER_REV  360

typedef struct {
    float distance_mm;
    float angle_deg;
} vector_polar_t;

/**
 * @brief 初始化LD14P: UART1 230400 (TX=17, RX=16) + 设置扫描频率
 * @param freq_hz 2~8 Hz
 */
esp_err_t ld14p_init(uint8_t freq_hz);

/**
 * @brief 向协议状态机喂一个字节, 状态机搜帧头→拼包→CRC→解析→插值→更新cloud_360[]
 */
void ld14p_feed_byte(uint8_t byte);

/**
 * @brief 检测新一圈数据是否完整可用
 * @return true 刚完成一整圈扫描 (每次调用只返回一次true)
 */
bool ld14p_scan_ready(void);

/**
 * @brief 获取当前360°点云快照 (正前方0°, 顺时针递增)
 * @param out   输出缓冲区(≥360元素)
 * @param count 输出: 360
 */
esp_err_t ld14p_get_scan(vector_polar_t *out, uint16_t *count);
