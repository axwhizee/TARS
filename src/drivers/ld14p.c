/**
 * @file ld14p.c
 * @brief LD14P 激光雷达底层驱动
 *
 * cloud_360[360] 持久化, 每圈自然覆盖不主动清零, distance==0 标记无效点.
 * 圈检测用帧末点角度 (借鉴官方 SDK), 配合 150ms 防抖.
 */
#include "drivers/ld14p.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

#define HEADER           0x54
#define VER_LEN          0x2C
#define ANGLE_RES        100               // 0.01° → ° 换算系数
#define CMD_SPEED        0xA2
#define CRC_COVER        (FRAME_LEN - 1)   // CRC 覆盖除最后一个字节外的全部
#define LASER_TAN        0.11923f          // tan(6.8°), 固定激光夹角
#define REV_DEBOUNCE_MS  150               // 圈检测防抖

static const char *TAG = "LD14P ";

static vector_polar_t cloud_360[LD14P_POINTS_PER_REV];
static float prev_last_deg;       // 上一帧末点角度, 用于跨零检测
static uint32_t rev_last_tick;
static bool rev_ready;

// CRC8 查表 (多项式 0x4D)
static const uint8_t CRC_TABLE[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
    0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
    0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
    0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
    0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
    0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
    0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
    0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
    0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
    0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
    0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
    0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
    0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
    0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
    0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
    0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
    0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
    0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
    0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
    0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
    0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
    0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

// CRC8 校验, 多项式 0x4D, init=0x00, 无最终 XOR
static uint8_t crc8_calc(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++)
        crc = CRC_TABLE[(crc ^ data[i]) & 0xFF];
    return crc;
}

// 发送 0xA2 转速控制命令, speed = SENSOR_FREQ * 360 (°/s)
static void send_freq_command() {
    uint16_t speed = (uint16_t)SENSOR_FREQ * 360;
    uint8_t cmd[8] = { HEADER, CMD_SPEED, 4, (uint8_t)(speed & 0xFF),
        (uint8_t)(speed >> 8), 0x00, 0x00, 0x00 };
    cmd[7] = crc8_calc(cmd, 7);
    uart_write_bytes(LD14P_UART_NUM, cmd, sizeof(cmd));
    ESP_LOGI(TAG, "LD14P set freq @ %d Hz (%d deg/s)", SENSOR_FREQ, speed);
}

const ld14p_frame_t *ld14p_parse(uint8_t byte) {
    typedef enum { S_IDLE, S_FRAME } state_t;
    static state_t state = S_IDLE;
    static uint8_t buf[FRAME_LEN];
    static uint8_t idx;

    if (state == S_IDLE) {
        if (byte == HEADER) {
            buf[0] = HEADER;
            idx = 1;
            state = S_FRAME;
        }
        return NULL;
    }

    buf[idx++] = byte;
    if (idx >= FRAME_LEN) {
        state = S_IDLE;
        idx = 0;
        // 帧长标识不匹配 → 丢弃
        if (buf[1] != VER_LEN)       return NULL;
        // CRC 校验失败 → 丢弃
        if (crc8_calc(buf, CRC_COVER) != buf[CRC_COVER]) return NULL;
        return (const ld14p_frame_t *)buf;
    }
    return NULL;
}

const vector_polar_t *ld14p_collect(const ld14p_frame_t *frm) {
    // 12 点在 start_angle~end_angle 间等间隔插值
    int diff = (int)frm->end_angle - (int)frm->start_angle;
    if (diff < 0) diff += 360 * ANGLE_RES;  // 跨 0° 补偿
    float step_raw = (float)diff / (LD14P_POINTS_PER_PACK - 1);

    float last_deg = 0.0f;
    for (int i = 0; i < LD14P_POINTS_PER_PACK; i++) {
        int raw_angle = (int)frm->start_angle + (int)(i * step_raw);
        int deg = (raw_angle / ANGLE_RES) % LD14P_POINTS_PER_REV;
        float angle_f = (float)raw_angle / ANGLE_RES;

        cloud_360[deg].angle = angle_f;
        cloud_360[deg].distance = (float)frm->points[i].distance;
        if (i == LD14P_POINTS_PER_PACK - 1) last_deg = angle_f;
    }

    // 圈检测: 末点角度跨过 0° 线 (prev>340 且 current<20)
    uint32_t now = xTaskGetTickCount();
    if (prev_last_deg > 340.0f && last_deg < 20.0f
        && (now - rev_last_tick) > pdMS_TO_TICKS(REV_DEBOUNCE_MS)) {
        rev_ready = true;
        rev_last_tick = now;
    }
    prev_last_deg = last_deg;

    if (rev_ready) {
        rev_ready = false;  // 单次消费, 阻止同一圈重复返回
        return cloud_360;
    }
    return NULL;
}

void ld14p_calibrate(vector_polar_t *points, float offset_x, float offset_y) {
    /*
     * SlTransform (参照官方 SDK):
     *   - 激光器偏离旋转中心 (offset_x, offset_y), 且发光方向有固定夹角
     *   - 公式: x = dist + offset_x
     *          y = dist * LASER_TAN + offset_y
     *          shift = atan2(y, x) * 180/π
     *          angle_corrected = angle_raw - shift (左手系)
     */
    static float last_shift = 0.0f;

    for (uint16_t i = 0; i < LD14P_POINTS_PER_REV; i++) {
        float dist = points[i].distance;
        float angle = points[i].angle;
        float shift;

        if (dist > 0.0f) {
            float x = dist + offset_x;
            float y = dist * LASER_TAN + offset_y;
            shift = atan2f(y, x) * 180.0f / 3.14159f;
            last_shift = shift;
        } else {
            shift = last_shift;     // 无效点沿用上次有效 shift, 避免角度跳变
        }

        angle -= shift;
        // 归一化到 [0, 360)
        if (angle > 360.0f) angle -= 360.0f;
        if (angle < 0.0f)   angle += 360.0f;
        points[i].angle = angle;
    }
}

esp_err_t ld14p_init() {
    if (SENSOR_FREQ < 2 || SENSOR_FREQ > 8) return ESP_ERR_INVALID_ARG;

    memset(cloud_360, 0, sizeof(cloud_360));
    prev_last_deg = 0.0f;
    rev_ready = false;
    rev_last_tick = 0;

    uart_config_t cfg = {
        .baud_rate = LD14P_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(LD14P_UART_NUM, &cfg);
    if (err != ESP_OK) return err;

    err = uart_set_pin(LD14P_UART_NUM, LD14P_UTX_PIN, LD14P_URX_PIN,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    // event_queue=NULL → 上层必须使用 timeout=0 非阻塞读取
    err = uart_driver_install(LD14P_UART_NUM, LD14P_UART_RX_BUF * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) return err;

    // 等电机上电稳定 → 发频率命令 → 等电机响应
    vTaskDelay(pdMS_TO_TICKS(100));
    send_freq_command();
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "LD14P initialized @ %d Hz", SENSOR_FREQ);
    return ESP_OK;
}
