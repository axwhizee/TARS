/**
 * @file sysmon_task.c
 * @brief 系统监控: WS2812 CPU负载颜色指示 (1Hz) + 每5s vTaskGetRunTimeStats()
 *
 * WS2812 颜色分级:
 *   绿 (0-33%)  → 黄 (33-66%) → 红 (66-100%)
 *
 * 前置条件 (sdkconfig.defaults):
 *   CONFIG_FREERTOS_USE_TRACE_FACILITY=y
 *   CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y
 *   CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS=y
 */
#include "tasks/sysmon_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "SYSMON";

#define STATS_PERIOD_MS   5000
#define STATS_BUF_SIZE    1024
#define RMT_RESOLUTION_HZ (10 * 1000 * 1000)  // 0.1μs per tick

// ── WS2812 RMT driver ──────────────────────────────────────────

static rmt_channel_handle_t rmt_chan;
static rmt_encoder_handle_t rmt_encoder;

static void ws2812_init(void) {
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = WS2812_PIN,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &rmt_chan));
    ESP_ERROR_CHECK(rmt_enable(rmt_chan));

    rmt_copy_encoder_config_t enc_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &rmt_encoder));
}

static void ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    // WS2812 GRB order, MSB first
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;

    rmt_symbol_word_t symbols[25];   // 24 bits + 1 reset
    for (int i = 0; i < 24; i++) {
        if ((grb >> (23 - i)) & 1) {
            // T1H=0.8μs(8 ticks)  T1L=0.45μs(5 ticks)
            symbols[i] = (rmt_symbol_word_t){
                .duration0 = 8, .level0 = 1,
                .duration1 = 5, .level1 = 0,
            };
        } else {
            // T0H=0.4μs(4 ticks)  T0L=0.85μs(8 ticks)
            symbols[i] = (rmt_symbol_word_t){
                .duration0 = 4, .level0 = 1,
                .duration1 = 8, .level1 = 0,
            };
        }
    }
    // Reset: >50μs low → 600 ticks = 60μs
    symbols[24] = (rmt_symbol_word_t){ .duration0 = 0, .level0 = 0,
                                       .duration1 = 600, .level1 = 0 };

    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    rmt_transmit(rmt_chan, rmt_encoder, symbols, sizeof(symbols), &tx_cfg);
    rmt_tx_wait_all_done(rmt_chan, portMAX_DELAY);
}

// ── CPU 使用率计算 (双核 SMP 校正) ──────────────────────────────
//
// uxTaskGetSystemState 的 pulTotalRunTime 是墙钟计数器(单核视角),
// 而两个 IDLE 任务在各自核上并行累加, 所以 dt_idle 会 > dt_total.
// 正确做法: 分核算 CPU% = 1 - idle_dt / wall_dt, 最后取均值.

static float cpu_usage_pct(void) {
    static uint32_t prev_total = 0, prev_idle0 = 0, prev_idle1 = 0;

    UBaseType_t n = uxTaskGetNumberOfTasks();
    TaskStatus_t *tasks = malloc(n * sizeof(TaskStatus_t));
    if (!tasks) return 0.0f;

    uint32_t total = 0, idle0 = 0, idle1 = 0;
    n = uxTaskGetSystemState(tasks, n, &total);
    for (UBaseType_t i = 0; i < n; i++) {
        if (strcmp(tasks[i].pcTaskName, "IDLE0") == 0)
            idle0 = tasks[i].ulRunTimeCounter;
        else if (strcmp(tasks[i].pcTaskName, "IDLE1") == 0)
            idle1 = tasks[i].ulRunTimeCounter;
    }
    free(tasks);

    float pct = 0.0f;
    uint32_t dt_wall = total - prev_total;
    if (dt_wall > 0) {
        float dt0 = (float)(idle0 - prev_idle0);
        float dt1 = (float)(idle1 - prev_idle1);
        float cpu0 = 100.0f * (1.0f - dt0 / dt_wall);
        float cpu1 = 100.0f * (1.0f - dt1 / dt_wall);
        pct = (cpu0 + cpu1) * 0.5f;
        if (pct < 0.0f) pct = 0.0f;
    }

    prev_total = total;
    prev_idle0 = idle0;
    prev_idle1 = idle1;
    return pct;
}

// 百分比->颜色映射 (纯整数, 绿→黄→红热力图)
static void cpu_to_color(float pct, uint8_t *r, uint8_t *g, uint8_t *b) {
    int p = (int)pct;
    if (p < 0) p = 0;
    if (p > 100) p = 100;

    if (p <= 50) {
        // 0%→50%: 绿 → 黄 (R 从 0 升到 255)
        *r = (uint8_t)(p * 255 / 50);
        *g = 255;
        *b = 0;
    } else {
        // 50%→100%: 黄 → 红 (G 从 255 降到 0)
        *r = 255;
        *g = (uint8_t)((100 - p) * 255 / 50);
        *b = 0;
    }
}

// ── 任务入口 ───────────────────────────────────────────────────

void sysmon_task(void *pvParameters) {
    (void)pvParameters;
    ws2812_init();

    TickType_t last_stats = xTaskGetTickCount();
    char buf[STATS_BUF_SIZE];

    ESP_LOGI(TAG, "Sysmon started (WS2812 GPIO%d, stats every %ds)",
        WS2812_PIN, STATS_PERIOD_MS / 1000);

    while (1) {
        float cpu = cpu_usage_pct();

        uint8_t r, g, b;
        cpu_to_color(cpu, &r, &g, &b);
        ws2812_set_rgb(r, g, b);

        TickType_t now = xTaskGetTickCount();
        if ((now - last_stats) >= pdMS_TO_TICKS(STATS_PERIOD_MS)) {
            last_stats = now;
            vTaskGetRunTimeStats(buf);
            ESP_LOGI(TAG, "CPU %.1f%%\n%s", cpu, buf);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // 1Hz
    }
}
