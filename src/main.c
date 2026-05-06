/**
 * @file main.c
 * @brief ESP32_Template 主入口 — 所有硬件初始化 + 任务创建
 *
 * 启动流程:
 *   1. ld14p_init(4)           — 初始化 UART1 + 发送 0xA2 频率命令 (待实现)
 *   2. motor_init()            — 初始化 LEDC PWM (GPIO4-7, 20kHz, 10-bit)
 *   3. 创建 q_polar + eg_sync   — LiDAR→APF 的数据管道
 *   4. 创建 q_cart             — APF→Motor 的数据管道
 *   5. vLedTask     (prio 1)   — GPIO48 心跳灯
 *   6. ld14p_sensor (prio 3)   — 数据读取 → 降采样 → 推送 q_polar (待实现)
 *   7. apf_task     (prio 3)   — 极坐标→APF→笛卡尔合力→推送 q_cart
 *   8. motor_task   (prio 2)   — EMA 滤波 → 差速转换 → PWM 驱动
 *
 * 数据流:
 *   LiDAR → q_polar → APF → q_cart → Motor → DRV8833
 */

#include "all_defs.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "drivers/motor.h"
#include "tasks/motor_task.h"
#include "tasks/apf_task.h"
// #include "drivers/ld14p.h"
// #include "tasks/lidar_task.h"

static const char *TAG = "MAIN";

/* ---------------------------------------------------------- */
/* 心跳 LED 任务                                                */
/* ---------------------------------------------------------- */
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

/* ---------------------------------------------------------- */
/* 主入口                                                       */
/* ---------------------------------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "System Init");

    /* ============================================ */
    /* 1. 硬件驱动初始化                             */
    /* ============================================ */

    /* LiDAR 传感器 (待实现) */
    // if (ld14p_init(4) != ESP_OK) {
    //     ESP_LOGE(TAG, "LD14P init failed");
    //     return;
    // }

    /* 电机 PWM 驱动 */
    if (motor_init() != ESP_OK) {
        ESP_LOGE(TAG, "Motor driver init failed");
        return;
    }

    /* ============================================ */
    /* 2. 创建任务间通信对象                          */
    /* ============================================ */

    /* q_polar: LiDAR 传感器 → APF 的极坐标数据管道 */
    QueueHandle_t q_polar = xQueueCreate(LIDAR_SECTORS, sizeof(vector_polar_t));
    if (q_polar == NULL) {
        ESP_LOGE(TAG, "q_polar creation failed");
        return;
    }

    /* eg_sync: LiDAR → APF 的同步事件组 (BIT_LIDAR_READY) */
    EventGroupHandle_t eg_sync = xEventGroupCreate();
    if (eg_sync == NULL) {
        ESP_LOGE(TAG, "eg_sync creation failed");
        return;
    }

    /* q_cart: APF → Motor 的笛卡尔指令管道 */
    QueueHandle_t q_cart = xQueueCreate(MOTOR_CMD_QUEUE_DEPTH, sizeof(vector_cart_t));
    if (q_cart == NULL) {
        ESP_LOGE(TAG, "q_cart creation failed");
        return;
    }

    /* ============================================ */
    /* 3. 创建任务                                   */
    /* ============================================ */

    /* 心跳 LED */
    xTaskCreate(vLedTask, "LedTask", 2048, NULL, 1, NULL);

    /* LiDAR 传感器任务 (待实现) */
    // static lidar_sensor_params_t lidar_params;
    // lidar_params.q_polar = q_polar;
    // lidar_params.eg_sync = eg_sync;
    // xTaskCreate(ld14p_sensor_task, "ld14p_sensor", 8192, &lidar_params, 3, NULL);

    /* APF 避障任务 (prio 3, 与 LiDAR 同级) */
    static apf_task_params_t apf_params;
    apf_params.q_polar = q_polar;
    apf_params.q_cart  = q_cart;
    apf_params.eg_sync = eg_sync;

    if (xTaskCreate(apf_task, "apf_task", APF_TASK_STACK,
                    &apf_params, APF_TASK_PRIO, NULL) != pdPASS) {
        ESP_LOGE(TAG, "APF task creation failed");
        return;
    }

    /* 电机控制任务 (prio 2) */
    static motor_task_params_t motor_params;
    motor_params.q_cart = q_cart;

    if (xTaskCreate(motor_task, "motor_task", MOTOR_TASK_STACK,
                    &motor_params, MOTOR_TASK_PRIO, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Motor task creation failed");
        return;
    }

    ESP_LOGI(TAG, "System started: LedTask + APFTask + MotorTask ready");
}
