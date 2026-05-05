/*
 * main.c — ESP32_Template 主入口
 *
 * 启动流程:
 *   1. ld14p_init(4)        — 初始化 UART1 + 发送 0xA2 频率命令
 *   2. 创建 eg_sync + q_polar — 事件组 + 36 深度队列 (vector_polar_t)
 *   3. vLedTask  (prio 1)   — GPIO48 心跳灯
 *   4. ld14p_sensor (prio 3) — 数据读取 → 降采样 → 推送队列
 */

#include "all_defs.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "drivers/ld14p.h"
#include "tasks/lidar_task.h"

static const char *TAG = "MAIN";

static void vLedTask(void *pvParameters)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    for (;;) {
        gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "System Init");

    if (ld14p_init(4) != ESP_OK) {
        ESP_LOGE(TAG, "LD14P init failed");
        return;
    }

    /* 传感器任务参数: 输出队列 + 同步事件组 */
    static lidar_sensor_params_t params;
    params.q_polar = xQueueCreate(LIDAR_SECTORS, sizeof(vector_polar_t));
    params.eg_sync = xEventGroupCreate();

    xTaskCreate(vLedTask,          "LedTask",       2048, NULL,    1, NULL);
    xTaskCreate(ld14p_sensor_task, "ld14p_sensor",  8192, &params, 3, NULL);
}
