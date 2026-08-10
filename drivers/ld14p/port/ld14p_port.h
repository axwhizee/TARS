/**
 * @file ld14p_port.h
 * @brief 移植层接口声明 — 移植到新平台时只需实现本文件声明的全部函数
 *
 * 注意：本文件不包含任何平台 HAL 头文件或宏，相关定义见 ld14p_port_def.h。
 */
#pragma once
#include "ld14p_types.h"

/**
 * @brief 初始化 UART 外设（安装驱动 / 配置参数 / 设置引脚）
 *
 * 硬件资源（串口号、波特率、引脚、缓冲区）见 ld14p_port_def.h；
 * 成功后应将平台外设句柄填入 h->bus。
 *
 * @param h 实例句柄
 * @return LD14P_OK 成功；LD14P_ERR_INIT 初始化失败
 */
ld14p_err_t ld14p_port_uart_init(ld14p_handle_t *h);

/**
 * @brief 反初始化 UART 外设，释放资源
 *
 * @param h 实例句柄
 * @return LD14P_OK 成功；LD14P_ERR_UART 释放失败
 */
ld14p_err_t ld14p_port_uart_deinit(ld14p_handle_t *h);

/**
 * @brief 发送数据（核心层发送控制命令用）
 *
 * @param h    实例句柄
 * @param data 数据缓冲
 * @param len  数据长度
 * @return LD14P_OK 成功；LD14P_ERR_UART 发送失败
 */
ld14p_err_t ld14p_port_write(ld14p_handle_t *h, const uint8_t *data, size_t len);

/**
 * @brief 接收数据
 *
 * @param h          实例句柄
 * @param buf        接收缓冲
 * @param len        缓冲大小
 * @param out_len    [出参] 实际读取字节数
 * @param timeout_ms 超时（ms）；0 = 非阻塞
 * @return LD14P_OK 成功；LD14P_ERR_UART 接收错误
 */
ld14p_err_t ld14p_port_read(ld14p_handle_t *h, uint8_t *buf, size_t len,
                            size_t *out_len, uint32_t timeout_ms);

/**
 * @brief 毫秒级延时
 *
 * @note RTOS / 裸机实现由 ld14p_config.h 中的 LD14P_RTOS_ACTIVE 控制
 * @param ms 延时毫秒数
 */
void ld14p_port_delay_ms(uint32_t ms);

/**
 * @brief 获取单调递增的毫秒时间戳（圈检测防抖用）
 *
 * @return 当前毫秒时间戳
 */
uint32_t ld14p_port_get_tick_ms(void);
