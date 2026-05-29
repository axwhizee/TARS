/**
 * @file flame_task.c
 * @brief 火焰传感器任务 — SENSOR_FREQ Hz GPIO 采样, 非阻塞跳过模式
 */
#include "tasks/flame_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "FLAME_TASK";

static const struct {
    gpio_num_t gpio;
    float      angle_deg;
} flame_sensors[FLAME_SENSOR_COUNT] = {
    {11, 300.0f},   /* 左侧 */
    {12, 330.0f},   /* 左前方 */
    {16,   0.0f},   /* 正前方 (中心) — 暂未飞线, 上拉保持 HIGH → 无火 */
    {13,  30.0f},   /* 右前方 */
    {14,  60.0f},   /* 右侧 */
};

esp_err_t flame_sensor_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = FLAME_GPIO_MASK,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,      /* 上拉: 无火=HIGH, 有火=LOW */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&io_conf);
}

void flame_task(void *pvParameters) {
    (void)pvParameters;

    const TickType_t period = pdMS_TO_TICKS(1000 / SENSOR_FREQ);    // 工作周期
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t skip_count = 0;
    ESP_LOGI(TAG, "Flame task started @%dHz", SENSOR_FREQ);

    while (1) {
        vTaskDelayUntil(&last_wake, period);    // 任务周期性运行

        if (!(xEventGroupGetBits(eg_sync) & BIT_FLAME_Q_READY)) {
            vector_polar_t data[FLAME_SENSOR_COUNT];
            for (int i = 0; i < FLAME_SENSOR_COUNT; i++) {
                data[i].angle = flame_sensors[i].angle_deg;
                bool flame_detected = (gpio_get_level(flame_sensors[i].gpio) == 0);
                data[i].distance = flame_detected ? FLAME_DETECT_MM : 0.0f;
            }

            for (int i = 0; i < FLAME_SENSOR_COUNT; i++) {
                xQueueSend(q_polar, &data[i], 0);
            }
            xEventGroupSetBits(eg_sync, BIT_FLAME_Q_READY);
            skip_count = 0;
        } else {
            if ((skip_count++ & 0xF) == 0) {
                ESP_LOGW(TAG, "Flame skipped: q_polar occupied (skip #%lu)", skip_count);
            }
        }
    }
}
