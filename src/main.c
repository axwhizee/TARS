/* main.c */
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define LED_PIN 48
static const char *TAG = "MAIN";

void vLedTask(void *pvParameters) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN), // 注意：64 位掩码
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf); // 1. 配置 GPIO

    ESP_LOGI(TAG, "Task Started on Core %d", xPortGetCoreID()); // 2. 专用日志

    for (;;) {
        gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) { // 3. 入口函数名固定为 app_main
    ESP_LOGI(TAG, "System Init");
    // 4. 堆栈单位是字 (Word)，不是字节，且最小堆栈要求更高
    xTaskCreate(vLedTask, "LedTask", 2048, NULL, 5, NULL); 
    // ESP-IDF 调度器在 app_main 返回前自动启动，无需 vTaskStartScheduler
}
