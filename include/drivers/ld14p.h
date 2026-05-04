#pragma once
#include "all_defs.h"

/* ============================================================
 *  LD14P 激光雷达驱动 — 公开API
 *  底层UART初始化、协议状态机、360° 点云获取
 *
 *  使用流程:
 *    1. ld14p_init(4)          — 初始化UART + 设置频率
 *    2. ld14p_feed_byte() x N  — 将UART字节喂入状态机
 *    3. ld14p_scan_ready()     — 轮询检测一圈是否完成
 *    4. ld14p_get_scan()       — 获取360点快照
 * ============================================================ */

/**
 * @brief 初始化LD14P激光雷达
 *        - 安装UART1驱动 (115200, 8N1, TX=17, RX=18)
 *        - 清零点云缓冲区
 *        - 通过0xA2命令设置目标扫描频率
 *
 * @param freq_hz 扫描频率 (2~8 Hz). 经实测频率以4Hz最稳定
 * @return esp_err_t
 *   - ESP_OK          成功
 *   - ESP_ERR_INVALID_ARG  freq_hz超出范围
 *   - ESP_ERR_NO_MEM   互斥锁创建失败
 *   - 其他 UART驱动安装/配置错误码
 *
 * @note 波特率经实测为115200 (与手册标注的230400不同)
 */
esp_err_t ld14p_init(uint8_t freq_hz);

/**
 * @brief 向协议状态机喂入一个字节
 *
 * 内部状态机自动完成:
 *   搜索帧头0x54 → 拼装47字节包 → CRC8校验(多项式0x4D)
 *   → 解析12个测量点 → 角度插值 → 更新cloud_360[]
 *
 * 此函数可反复调用, 无需了解LD14P协议细节.
 * 通常由传感器任务从UART读取字节后调用此函数.
 *
 * @param byte 从UART1 RX线收到的原始字节
 */
void ld14p_feed_byte(uint8_t byte);

/**
 * @brief 检查一整圈360°扫描是否已完整接收
 *
 * 内部通过监测start_angle从>30000翻转到<6000来判断一圈完成.
 * 每次调用返回true后会**清除标志**, 下次调用重新返回false直到下一圈完成.
 *
 * @return true  刚完成一整圈扫描 (cloud_360[]已包含所有360°数据)
 * @return false 尚未完成一整圈或已被消费
 *
 * @note 应在传感器任务中轮询调用, 返回true后配合ld14p_get_scan()读取数据
 */
bool ld14p_scan_ready(void);

/**
 * @brief 获取当前360°点云快照
 *
 * 拷贝cloud_360[]快照到用户缓冲区, 输出以传感器正前方为0°、
 * 顺时针递增的360个距离-角度点.
 * 不做坐标系转换(保持LD14P原生坐标系).
 *
 * @param[out] out   输出缓冲区, 需至少LD14P_POINTS_PER_REV (360) 个元素
 * @param[out] count 固定输出 360
 * @return esp_err_t
 *   - ESP_OK           成功
 *   - ESP_ERR_INVALID_ARG   out或count为NULL
 *
 * @code
 *   vector_polar_t scan[360];
 *   uint16_t n;
 *   if (ld14p_get_scan(scan, &n) == ESP_OK) {
 *       // scan[i].angle_deg   = i          (0~359°)
 *       // scan[i].distance_mm = 距离(毫米)
 *   }
 * @endcode
 */
esp_err_t ld14p_get_scan(vector_polar_t *out, uint16_t *count);
