/*
 * lidar_task.c — LD14P 传感器任务 (调试精简版)
 *
 * 职责:
 *   1. 从 UART1 环形缓冲区连续 drain (非阻塞读取)
 *   2. 逐字节喂入 ld14p 协议状态机
 *   3. 检测完整一圈 360° 扫描 → 从 cloud_360[] 快照数据 → 统计有效点数
 *   4. 每 2 秒输出吞吐量统计 (fed total, rx_buf backlog)
 *
 * 核心设计原则 (经过 Cr ash 调试确定的):
 *   - uart_read_bytes timeout=0 (非阻塞) → 避免 StoreProhibited @0x08 崩溃
 *   - 批量 drain (最多 50 批 × 256 字节) → 保持 ring buffer 不积压
 *   - vTaskDelay(2 ticks = 20ms) 固定间隔 → 兼顾 CPU 让步与数据吞吐
 */

#include "tasks/lidar_task.h"
#include "drivers/ld14p.h"
#include "all_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "LIDAR";

void ld14p_sensor_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "=== Sensor task started (fast drain mode) ===");

    /*
     * scan[360]: 本地 360 点快照缓冲区.
     *   大小 = 360 × 8 字节 = 2880 字节.
     *   任务栈 8192 words = 32768 字节, 足够容纳.
     *   ld14p_get_scan() 将 cloud_360[] 的当前状态拷贝到这里.
     */
    vector_polar_t scan[LD14P_POINTS_PER_REV];
    uint16_t       count;
    int64_t        last_stat = esp_timer_get_time(); /* us 级时间戳, 用于 2s 日志节流 */
    bool           dumped    = false;                /* 首包 hex dump 只打一次 */

    while (1) {
        /*
         * ═══════════════════════════════════════════════════════════
         *  第 1 阶段: drain UART1 环形缓冲区
         * ═══════════════════════════════════════════════════════════
         *
         * buf[256]: 单批最大读入量.
         *   256 字节 ≈ 5.4 个 LD14P 数据包 (每包 47 字节).
         *
         * 内层 while:
         *   - uart_read_bytes(..., timeout=0) 非阻塞.
         *     有数据 → 返回实际字节数; 无数据 → 返回 0, 立即退出内层 while.
         *   - 最多 50 批 × 256 字节 = 12.8KB ≈ 1.1 秒数据量 (115200 bps).
         *     这个上限防止: (a) 任务看门狗超时 (b) 长时间不 yield 饿死其他任务.
         *   - 每批数据立刻逐字节喂给 ld14p_feed_byte().
         *
         * 首包 hex dump:
         *   当累计喂入 ≥ 47 字节后, 打印正在读的那批数据的前 16 字节.
         *   目的: 肉眼确认 0x54 0x2C 帧头存在, 验证波特率和协议正确.
         */
        uint8_t buf[256];
        int len, batch = 0;

        while (batch < 50 && (len = uart_read_bytes(LD14P_UART_NUM, buf, sizeof(buf), 0)) > 0) {
            for (int i = 0; i < len; i++)
                ld14p_feed_byte(buf[i]);
            batch++;

            if (!dumped && ld14p_get_total_bytes() >= 47) {
                dumped = true;
                ESP_LOGI(TAG, "First packet: %02x %02x %02x %02x %02x %02x %02x %02x",
                         buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
                ESP_LOGI(TAG, "             %02x %02x %02x %02x %02x %02x %02x %02x",
                         buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
            }
        }

        /*
         * ═══════════════════════════════════════════════════════════
         *  第 2 阶段: 消费完整的 360° 扫描结果
         * ═══════════════════════════════════════════════════════════
         *
         * ld14p_scan_ready():
         *   检查 revolution_flag, 若为 true → 说明刚刚完成一整圈 360° 扫描.
         *   返回 true 的同时会清除该标志 (消费一次).
         *   这个标志由 ld14p 状态机在检测到角度从 >300° 翻转到 <60° 时置位.
         *
         * ld14p_get_scan():
         *   把驱动内部的 cloud_360[360] 拷贝到本地 scan[] 缓冲区.
         *   cloud_360[] 是持久化的 — 不被清空, 每圈数据自然覆盖刷新对应角度.
         *
         * 有效点判定:
         *   distance_mm < 60000 → 有效测量值 (LD14P 最大量程 8m = 8000mm).
         *   初始化为 0xFFFF (65535) 的角度尚未收到数据, 视为无效.
         */
        if (ld14p_scan_ready()) {
            if (ld14p_get_scan(scan, &count) == ESP_OK) {
                uint32_t valid = 0;
                for (int i = 0; i < count; i++)
                    if (scan[i].distance_mm < 60000) valid++;
                ESP_LOGI(TAG, "REV: %lu / %d valid pts", valid, count);
            }
        }

        /*
         * ═══════════════════════════════════════════════════════════
         *  第 3 阶段: 每 2 秒输出吞吐量统计
         * ═══════════════════════════════════════════════════════════
         *
         * 使用 esp_timer_get_time() (微秒级) 而非 xTaskGetTickCount().
         * 原因: 2 秒间隔用 tick 也够, 但 us 级更精确, 且不受 tick 频率影响.
         *
         * 关键指标:
         *   fed      — 累计喂入协议状态机的字节数 (应持续增长)
         *   rx_buf   — UART 硬件环形缓冲区中的积压字节数 (0 表示无积压, 理想状态)
         *   batch    — 本循环内实际 drain 的批次数 (0 表示无数据可读)
         */
        int64_t now = esp_timer_get_time();
        if (now - last_stat >= 2000000) {
            size_t rx = 0;
            uart_get_buffered_data_len(LD14P_UART_NUM, &rx);
            ESP_LOGI(TAG, "STAT: fed=%lu | rx_buf=%u | batch=%d",
                     ld14p_get_total_bytes(), (unsigned)rx, batch);
            last_stat = now;
        }

        /*
         * ═══════════════════════════════════════════════════════════
         *  第 4 阶段: 固定 2-tick (20ms) 让步
         * ═══════════════════════════════════════════════════════════
         *
         * 为什么是 2 ticks 而不是 1 tick?
         *   - 实测 vTaskDelay(1) 在高优先级 + ESP-IDF v5.5.3 下不可靠,
         *     表现为任务似乎冻结, STAT 日志永不出现.
         *   - vTaskDelay(2) = 20ms 稳定工作 (经 24s+ 连续运行验证).
         *
         * 20ms 期间 LD14P 约产生 11520 × 0.02 = 230 字节 → 约 5 个数据包.
         * 下一次循环 drain 256 字节即可清空这些新数据, 不会积压.
         */
        vTaskDelay(2);
    }
}
