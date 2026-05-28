/**
 * @file sys_init.c
 * @brief ESP32 系统级初始化实现 — NVS Flash / SPIFFS 挂载 / Wi-Fi AP 启动
 *
 *   1. NVS    — Wi-Fi 与 SPIFFS 需要 NVS 存储分区
 *   2. SPIFFS — 网页文件 (data/) 存于 SPIFFS 分区, 挂载到 /spiffs
 *   3. Wi-Fi  — AP 模式 (无加密), 静态 IP 192.168.1.1/24, 所有后续网络服务
 *       (HTTP + WebSocket) 均基于此
 */
#include "sys_init.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

static const char *TAG = "SYS_INIT";

esp_err_t sys_nvs_init(void) {
    ESP_LOGI(TAG, "NVS init...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition corrupt, erasing...");
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
    else               ESP_LOGI(TAG, "NVS ready");
    return ret;
}

esp_err_t sys_spiffs_init(void) {
    ESP_LOGI(TAG, "SPIFFS mount /spiffs...");
    esp_vfs_spiffs_conf_t conf = {
        .base_path              = "/spiffs",
        .partition_label        = NULL,
        .max_files              = 5,
        .format_if_mount_failed = true,
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
        return ret;
    }
    size_t total = 0, used = 0;
    esp_spiffs_info(NULL, &total, &used);
    ESP_LOGI(TAG, "SPIFFS ready: %d/%d KB", (int)(used / 1024), (int)(total / 1024));
    return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    (void)arg; (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_START)
        ESP_LOGI(TAG, "WiFi AP started");
}

esp_err_t sys_wifi_init(void) {
    ESP_LOGI(TAG, "WiFi AP init: SSID=%s...", WIFI_SSID);
    esp_netif_init();
    esp_event_loop_create_default();

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    esp_netif_ip_info_t ip;
    esp_netif_str_to_ip4("192.168.1.1", &ip.ip);
    esp_netif_str_to_ip4("192.168.1.1", &ip.gw);
    esp_netif_str_to_ip4("255.255.255.0", &ip.netmask);
    esp_netif_dhcps_stop(ap_netif);
    esp_netif_set_ip_info(ap_netif, &ip);
    esp_netif_dhcps_start(ap_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        wifi_event_handler, NULL, NULL);
    esp_wifi_set_mode(WIFI_MODE_AP);
    wifi_config_t wifi_cfg = {
        .ap = {
            .ssid            = WIFI_SSID,
            .ssid_len        = 0,
            .max_connection  = 4,
            .authmode        = WIFI_AUTH_OPEN,
        },
    };
    esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
    esp_err_t ret = esp_wifi_start();

    ESP_LOGI(TAG, "AP ready: 192.168.1.1, ws://192.168.1.1:%d/ws", WEBSOCKET_PORT);
    return ret;
}
