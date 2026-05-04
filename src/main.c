/* main.c */
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "drivers/ld14p.h"
#include "tasks/lidar_task.h"
#include "tasks/logger.h"

#define LED_PIN 48
static const char *TAG = "MAIN";

void vLedTask(void *pvParameters) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    for (;;) {
        gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "System Init");

    ld14p_init(4);

    QueueHandle_t scan_queue = xQueueCreate(360, sizeof(vector_polar_t));

    xTaskCreate(vLedTask,           "LedTask",       2048, NULL,                  1, NULL);
    xTaskCreate(ld14p_sensor_task,  "ld14p_sensor",  8192, (void *)scan_queue,    5, NULL);
    xTaskCreate(logger_task,        "logger",        4096, (void *)scan_queue,    4, NULL);
}
