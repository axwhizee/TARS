/**
 * @file main.c
 * @brief 主入口：所有硬件初始化 + 任务创建
 */
#include "all_defs.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "drivers/ld14p.h"
#include "drivers/ds18b20.h"
#include "drivers/drv8833.h"
#include "tasks/lidar_task.h"
#include "tasks/flame_task.h"
#include "tasks/temp_task.h"
#include "tasks/apf_task.h"
#include "tasks/motor_task.h"
// #include "tasks/mqtt_task.h"
#include "tasks/websocket_task.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

// #define DEBUG

static const char *TAG = "MAIN";

// 全局 RTOS 句柄
QueueHandle_t        q_polar;    // LD14P 数据队列
QueueHandle_t        q_cart;     // APF 计算结果队列
QueueHandle_t        q_temp;     // DS18B20 温度数据队列
QueueHandle_t        q_log;      // 日志上传数据队列 (透传 vector_polar_t)
EventGroupHandle_t   eg_sync;    // 传感器同步事件组

// 心跳 LED 任务
static void vLedTask(void *pvParameters) {
    (void)pvParameters;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    while (1) {
        gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// Wi-Fi 事件处理（AP 模式）
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "WiFi AP started");
    }
}

// Wi-Fi 初始化（AP 模式, 无加密, 静态 IP 192.168.1.1/24）
static esp_err_t wifi_init_ap(void) {
    ESP_LOGI(TAG, "Starting WiFi AP: %s", WIFI_SSID);
    esp_netif_init();
    esp_event_loop_create_default();

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    esp_netif_ip_info_t ip_info;
    esp_netif_str_to_ip4("192.168.1.1", &ip_info.ip);
    esp_netif_str_to_ip4("192.168.1.1", &ip_info.gw);
    esp_netif_str_to_ip4("255.255.255.0", &ip_info.netmask);
    esp_netif_dhcps_stop(ap_netif);
    esp_netif_set_ip_info(ap_netif, &ip_info);
    esp_netif_dhcps_start(ap_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL);

    esp_wifi_set_mode(WIFI_MODE_AP);
    wifi_config_t wifi_cfg = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = 0,
            .max_connection = 4,
            .authmode = WIFI_AUTH_OPEN,
        },
    };
    esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
    esp_err_t re = esp_wifi_start();

    ESP_LOGI(TAG, "AP ready: 192.168.1.1, WebSocket: ws://192.168.1.1:%d/", WEBSOCKET_PORT);
    return re;
}

#ifdef DEBUG
/**
 * @brief 小车转向能力测试
 */
static void vTestVectorTask(void *pvParameters) {
    (void)pvParameters;
    const int step_ms = 250;
    int pct_a = 100, pct_b = 100;

    ESP_LOGI(TAG, "=== Percent Ramp: 0→100%%, step=%dms ===", step_ms);

    while (1) {
        while (pct_b != -100) {
            pct_b -= 10;
            motor_set((int8_t)pct_a, (int8_t)pct_b);
            ESP_LOGI(TAG, "motor set: A %d %%, B %d %%", pct_a, pct_b);
            vTaskDelay(pdMS_TO_TICKS(step_ms));
        }
        pct_b = 100;
        pct_a -= 10;
        if (pct_a == -100)
            pct_a = 100;
    }
}
#endif

void app_main(void) {
    ESP_LOGI(TAG, "System Init");

    // 0. NVS (WiFi 需要)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // 1. 硬件驱动初始化

    // Wi-Fi (AP 模式)
    if (wifi_init_ap() != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi init failed");
        return;
    }
    // LiDAR 传感器驱动
    if (ld14p_init() != ESP_OK) {
        ESP_LOGE(TAG, "LD14P init failed");
        return;
    }
    // 电机 PWM 驱动
    if (motor_init() != ESP_OK) {
        ESP_LOGE(TAG, "Motor init failed");
        return;
    }
    // DS18B20 温度传感器
    if (ds18b20_init() != ESP_OK) {
        ESP_LOGW(TAG, "DS18B20 init failed, temperature task will retry");
    }
    // 火焰传感器 GPIO 初始化
    if (flame_sensor_init() != ESP_OK) {
        ESP_LOGW(TAG, "Flame sensor GPIO init failed");
    }

    // 3. 创建任务间通信对象

    // q_polar: LD14P (36) + 火焰传感器 (5) 极坐标数据
    q_polar = xQueueCreate(Q_POLAR_DEPTH, sizeof(vector_polar_t));
    if (q_polar == NULL) {
        ESP_LOGE(TAG, "q_polar creation failed");
        return;
    }
    // q_cart: 用于APF算法计算的笛卡尔坐标数据
    q_cart = xQueueCreate(1, sizeof(vector_cart_t));
    if (q_cart == NULL) {
        ESP_LOGE(TAG, "q_cart creation failed");
        return;
    }
    // q_temp: DS18B20 温度数据队列 (depth=4, 1s 缓冲)
    q_temp = xQueueCreate(4, sizeof(float));
    if (q_temp == NULL) {
        ESP_LOGE(TAG, "q_temp creation failed");
        return;
    }
    // q_log: 日志上传数据队列 (depth=41, 与 q_polar 一致)
    q_log = xQueueCreate(Q_POLAR_DEPTH, sizeof(vector_polar_t));
    if (q_log == NULL) {
        ESP_LOGE(TAG, "q_log creation failed");
        return;
    }
    // eg_sync: 用于确保传感器就绪的事件组
    eg_sync = xEventGroupCreate();
    if (eg_sync == NULL) {
        ESP_LOGE(TAG, "eg_sync creation failed");
        return;
    }

    // 4. 创建任务

#ifndef DEBUG
    motor_set(0, 0);    // 确保电机不动
    // 心跳 LED，优先级最低
    xTaskCreate(vLedTask, "LedTask", 2048, NULL, 1, NULL);
    // LIDAR 传感器任务，较复杂，依赖UART缓冲区
    xTaskCreate(ld14p_task, "ld14p_sensor", 8192, NULL, 5, NULL);
    // 火焰传感器任务，简单，快速完成
    xTaskCreate(flame_task, "flame_sensor", 2048, NULL, 7, NULL);
    // 温度传感器任务，有时序要求
    xTaskCreate(temp_task, "temp_sensor", 4096, NULL, 6, NULL);
    // 日志上传任务 (WebSocket 服务端)，低优先级，与数据流解耦
    // xTaskCreate(mqtt_task, "mqtt_log", 8192, NULL, 3, NULL);
    xTaskCreate(websocket_task, "websocket_log", 8192, NULL, 3, NULL);
    vTaskDelay(pdMS_TO_TICKS(1000)); 
    /* APF 避障任务，较复杂，依赖雷达、温度传感器数据，读取队列后即释放事件组 */
    xTaskCreate(apf_task, "apf_task", 4096, NULL, 4, NULL);
    /* 电机控制任务，较复杂，强时序要求 */
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 8, NULL);
#endif
#ifdef DEBUG
    /* 测试地底盘功能 */
    if (xTaskCreate(vTestVectorTask, "TestVector", 2048, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "TestVector task creation failed");
        return;
    }
#endif
    ESP_LOGI(TAG, "\n----------Leaving app_main, scheduler to be started----------\n\n");
}
