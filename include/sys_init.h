/**
 * @file sys_init.h
 * @brief ESP32 系统级初始化 — NVS / SPIFFS / Wi-Fi AP
 */
#pragma once
#include "apf_common.h"

esp_err_t sys_nvs_init(void);
esp_err_t sys_spiffs_init(void);
esp_err_t sys_wifi_init(void);
