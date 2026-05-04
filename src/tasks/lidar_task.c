#include "tasks/lidar_task.h"
#include "drivers/ld14p.h"
#include "all_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "LIDAR_TASK";

/**
 * @brief LD14P 传感器任务
 *
 * 【职责】
 *   连续轮询 UART1 读取 LD14P 原始字节 → 喂入协议状态机
 *   → 检测到一圈扫描完成 → 获取 360° 点云快照 → 推入队列
 *
 * 【数据流】
 *   UART1 RX ─→ ld14p_feed_byte() → 状态机 → cloud_360[]
 *                                              │
 *   ld14p_scan_ready() ←────────────────────────┘ (360°跨越检测)
 *          │ true
 *          ▼
 *   ld14p_get_scan()  →  xQueueReset(queue)
 *                       →  xQueueSend ×360  (每个 vector_polar_t 一个元素)
 *
 * 【时序】
 *   当UART无数据时, 每次循环 vTaskDelay(1) ≈ 10ms 让步CPU.
 *   当LD14P持续发送数据时(≈5640字节/秒@4Hz), 内层while紧耦合读取.
 */
void ld14p_sensor_task(void *arg) {
    QueueHandle_t queue = (QueueHandle_t)arg;
    ESP_LOGI(TAG, "Sensor task started");

    /* scan_buf 用于接收 ld14p_get_scan() 的360点快照,
     * 约 360×8=2880 字节, 需保证任务栈 ≥8192 words */
    vector_polar_t scan_buf[LD14P_POINTS_PER_REV];
    uint16_t count = 0;

    while (1) {
        /* ---- Phase 1: 耗尽UART RX 缓冲区内所有可用字节 ---- */
        uint8_t byte;
        while (uart_read_bytes(LD14P_UART_NUM, &byte, 1, 0) > 0)
            ld14p_feed_byte(byte);

        /* ---- Phase 2: 检测整圈扫描是否完成 ---- */
        if (ld14p_scan_ready()) {
            if (ld14p_get_scan(scan_buf, &count) == ESP_OK) {
                /* 清空旧数据 → 推送最新一圈360点 */
                xQueueReset(queue);
                for (int i = 0; i < count; i++)
                    xQueueSend(queue, &scan_buf[i], 0);
            }
        } else {
            /* 无数据也无新一圈, 让步CPU让低优先级任务运行 */
            vTaskDelay(1);
        }
    }
}
