/*
 * ESP32_Template — 主入口
 *
 * 当前精简启动流程 (调试阶段):
 *   1. 初始化 LD14P 激光雷达 (UART1 @ 115200, 4Hz)
 *   2. 启动两个任务:
 *      - vLedTask        (prio 1) 系统指示灯   (1秒闪烁, 确认调度器活着)
 *      - ld14p_sensor    (prio 3) LD14P 数据读取 + 日志输出
 *
 *   logger_task 已暂时禁用, sensor 任务独立运行便于排查问题.
 */
#include "all_defs.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "drivers/ld14p.h"
#include "tasks/lidar_task.h"

static const char *TAG = "MAIN";
static QueueHandle_t scan_queue = NULL;

static void vLedTask(void *pvParameters)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = 0,
        .pull_down_en = 0,
        .intr_type    = GPIO_INTR_DISABLE,
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

    /*
     * 第 1 步: ld14p_init(4)
     *   - 安装 UART1 驱动 (115200-8N1, RX=GPIO18, TX=GPIO17)
     *   - 清空 cloud_360[360] (所有距离=0xFFFF 标记未填充)
     *   - 发送 0xA2 速率命令 (4Hz → 1440°/s)
     *   - 等待 300ms 让电机稳定, 其间 UART RX 环形缓冲区开始积压数据
     *   - 返回 ESP_OK 意味着 UART 已就绪, 可以开始喂字节
     */
    esp_err_t err = ld14p_init(4);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LD14P init failed: %s", esp_err_to_name(err));
        return;
    }

    /*
     * 第 2 步: 启动两个 FreeRTOS 任务
     *   - LedTask: 最低优先级 1, 仅用于确认调度器正常工作 (心跳指示灯)
     *   - ld14p_sensor: 优先级 3, 独占消费 UART1 数据流
     *
     * 队列 (scan_queue) 已创建但不使用 — 当前 sensor 任务直接读 cloud_360[]
     * 并在本地统计有效点数, 不推送到队列.
     */
    xTaskCreate(vLedTask,          "LedTask",       2048, NULL, 1, NULL);
    xTaskCreate(ld14p_sensor_task, "ld14p_sensor",  8192, NULL, 3, NULL);

    /*
     * app_main() 返回后, ESP-IDF 的 main_task 会自行销毁.
     * FreeRTOS 调度器继续运行 — 只要还有任务没退出, 系统就不会停止.
     */
}
