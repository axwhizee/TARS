/**
 * @file main.c
 * @brief ESP32_Template 主入口 — 所有硬件初始化 + 任务创建
 *
 * 启动流程:
 *   1. ld14p_init(4)           — 初始化 UART1 + 发送 0xA2 频率命令 (待实现)
 *   2. motor_init()            — 初始化 LEDC PWM (GPIO4-7, 20kHz, 10-bit)
 *   3. 创建 g_q_polar, g_q_cart, g_eg_sync (句柄定义见 all_defs.h)
 *   4. vLedTask     (prio 1)   — GPIO48 心跳灯
 *   5. ld14p_sensor (prio 3)   — 数据读取 → 降采样 → 推送 g_q_polar (待实现)
 *   6. vTestVector  (prio 3)   — 测试用: 生成前进/转向/后退测试向量 → g_q_cart
 *   7. motor_task   (prio 2)   — EMA 滤波 → 差速转换 → PWM 驱动
 *
 * 数据流:
 *   vTestVector → g_q_cart → motor_task → DRV8833
 *   未来: LiDAR → g_q_polar → APF → g_q_cart → motor_task → DRV8833
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
// #include "tasks/apf_task.h"
// #include "drivers/ld14p.h"
// #include "tasks/lidar_task.h"

static const char *TAG = "MAIN";

/* RTOS 全局句柄 (extern 声明在 all_defs.h, 此处定义) */
QueueHandle_t      g_q_polar = NULL;
QueueHandle_t      g_q_cart  = NULL;
EventGroupHandle_t g_eg_sync = NULL;

/* 心跳 LED 任务 */
static void vLedTask(void *pvParameters)
{
    (void)pvParameters;
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

/**
 * @brief 测试阶段定义（临时, 用于验证电机驱动）
 *
 * 每个阶段持续 TEST_PHASE_DURATION_MS, 循环发送 vector_cart_t 到 g_q_cart
 * EMA 滤波会使电机平滑过渡
 */
#define TEST_TICK_MS      250   /* 每个测试周期发送一次指令 */
#define TEST_PHASE_TICKS  (3000 / TEST_TICK_MS)  /* 每阶段持续 3s = 15 次 */

typedef struct {
    float x;     /* 前进/后退分量 (mm) */
    float y;     /* 转向分量 (mm) */
    const char *desc;
} test_phase_t;

static const test_phase_t test_seq[] = {
    { 3000.0f,     0.0f, "Forward"           },
    { 2000.0f,   800.0f, "Forward-Right"     },
    { 3000.0f,     0.0f, "Forward"           },
    {    0.0f,     0.0f, "Stop"              },
    {-2000.0f,     0.0f, "Reverse"           },
    {-2000.0f,  -800.0f, "Reverse-Left"      },
    {    0.0f,     0.0f, "Stop (pre-loop)"   },
};

#define TEST_PHASE_COUNT (sizeof(test_seq) / sizeof(test_seq[0]))

static void vTestVectorTask(void *pvParameters)
{
    (void)pvParameters;
    vector_cart_t cmd;
    int phase = 0;
    int tick = TEST_PHASE_TICKS;  /* 立即切换到第一个阶段 */

    ESP_LOGI(TAG, "TestVector: started, %d phases", TEST_PHASE_COUNT);

    while (1) {
        /* 阶段切换: 在新阶段开始时打印标识 */
        if (tick >= TEST_PHASE_TICKS) {
            tick = 0;
            ESP_LOGI(TAG, "TestVector: phase %d/%d → %s (x=%.0f y=%.0f)",
                     phase + 1, TEST_PHASE_COUNT,
                     test_seq[phase].desc,
                     (double)test_seq[phase].x,
                     (double)test_seq[phase].y);
        }

        /* 发送当前阶段的测试向量 */
        cmd.x = test_seq[phase].x;
        cmd.y = test_seq[phase].y;
        xQueueOverwrite(g_q_cart, &cmd);

        vTaskDelay(pdMS_TO_TICKS(TEST_TICK_MS));
        tick++;

        /* 阶段完成 → 推进到下一阶段 (在发送之后) */
        if (tick >= TEST_PHASE_TICKS) {
            phase = (phase + 1) % TEST_PHASE_COUNT;
        }
    }
}

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
    /* 2. 创建任务间通信对象 (句柄定义在文件顶部)      */
    /* ============================================ */

    /* g_q_polar: LiDAR 传感器 → APF 的极坐标数据管道 */
    g_q_polar = xQueueCreate(LIDAR_SECTORS, sizeof(vector_polar_t));
    if (g_q_polar == NULL) {
        ESP_LOGE(TAG, "g_q_polar creation failed");
        return;
    }

    /* g_eg_sync: LiDAR → APF 的同步事件组 (BIT_LIDAR_READY) */
    g_eg_sync = xEventGroupCreate();
    if (g_eg_sync == NULL) {
        ESP_LOGE(TAG, "g_eg_sync creation failed");
        return;
    }

    /* g_q_cart: 上游 (测试/APF) → Motor 的笛卡尔指令管道 */
    g_q_cart = xQueueCreate(MOTOR_CMD_QUEUE_DEPTH, sizeof(vector_cart_t));
    if (g_q_cart == NULL) {
        ESP_LOGE(TAG, "g_q_cart creation failed");
        return;
    }

    /* ============================================ */
    /* 3. 创建任务                                   */
    /* ============================================ */

    /* 心跳 LED */
    xTaskCreate(vLedTask, "LedTask", 2048, NULL, 1, NULL);

    /* LiDAR 传感器任务 (待实现) */
    // xTaskCreate(ld14p_sensor_task, "ld14p_sensor", 8192, NULL, 3, NULL);

    /* APF 避障任务 (待 LiDAR 就绪后启用) */
    // if (xTaskCreate(apf_task, "apf_task", APF_TASK_STACK, NULL,
    //                 APF_TASK_PRIO, NULL) != pdPASS) {
    //     ESP_LOGE(TAG, "APF task creation failed");
    //     return;
    // }

    /* 测试向量生成任务 (验证电机驱动) */
    if (xTaskCreate(vTestVectorTask, "TestVector", 2048, NULL,
                    3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "TestVector task creation failed");
        return;
    }

    /* 电机控制任务 (prio 2) */
    if (xTaskCreate(motor_task, "motor_task", MOTOR_TASK_STACK, NULL,
                    MOTOR_TASK_PRIO, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Motor task creation failed");
        return;
    }

    ESP_LOGI(TAG, "System started: LedTask + TestVector + MotorTask ready");
}
