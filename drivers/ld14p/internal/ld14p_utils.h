/**
 * @file ld14p_utils.h
 * @brief 内部辅助工具（仅核心层引用，不对外暴露，移植时无需修改）
 */
#pragma once
#include <stdint.h>

/**
 * @brief CRC8 校验（多项式 0x4D，init=0x00，无最终 XOR）
 *
 * @param data 待校验数据
 * @param len  数据长度
 * @return 计算出的 CRC8 值
 */
uint8_t ld14p_crc8(const uint8_t *data, uint8_t len);
