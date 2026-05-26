/**
 * @file ds18b20.h
 * @brief DS18B20 数字温度计 1-Wire 驱动
 */
#pragma once
#include "all_defs.h"

/**
 * @brief 初始化 DS18B20 驱动
 * 
 * 配置 DQ 引脚为开漏输出 + 内部上拉, 设置指定分辨率的配置寄存器,
 * 并写入 EEPROM 持久化.
 * @return ESP_OK 成功, ESP_ERR_TIMEOUT 总线无器件响应
 */
esp_err_t ds18b20_init(void);

/**
 * @brief 发起一次温度转换 [44h]
 * 
 * 非阻塞, 发送 Skip ROM + Convert T 后立即返回.
 * 后续需轮询 ds18b20_poll() 或延迟 Tconv (见表) 后读取.
 * @return ESP_OK 成功, ESP_ERR_TIMEOUT 器件无响应
 */
esp_err_t ds18b20_start_conversion(void);

/**
 * @brief 轮询转换完成状态
 * 
 * 在 ds18b20_start_conversion() 之后调用, 每次调用产生一个读时隙,
 * DS18B20 在转换中返回 0, 完成后返回 1.
 * @return true 转换完成, false 仍在转换中
 */
bool ds18b20_poll(void);

/**
 * @brief 读取温度值
 * 
 * 发送 Read Scratchpad [BEh], 读取 9 字节暂存器, 验证 CRC8.
 * 输出已补偿符号位, 分辨率无关 (统一除以 16).
 * @return 温度值 (°C), NAN 表示 CRC 校验失败
 */
float ds18b20_read_temp(void);
