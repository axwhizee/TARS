#include "drivers/ld14p.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "LD14P";

/* 协议内部宏 (不暴露给外部) */
#define LD14P_HEADER         0x54
#define LD14P_PACKET_LEN     47
#define LD14P_POINT_PER_PACK 12
#define LD14P_ANGLE_RES      100
#define LD14P_CMD_SPEED      0xA2
#define LD14P_CMD_LEN        4

/* UART配置来自 all_defs.h (通过 ld14p.h 间接包含) */

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

static uint8_t ld14p_crc8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++)
        crc = CRC_TABLE[(crc ^ data[i]) & 0xFF];
    return crc;
}

typedef struct {
    uint16_t distance;
    uint8_t  intensity;
} ld14p_point_t;

static ld14p_point_t cloud_360[LD14P_POINTS_PER_REV];
static uint16_t prev_start_angle = 0;
static volatile bool revolution_flag = false;

static uint32_t crc_ok_count = 0;
static uint32_t crc_fail_count = 0;

static void ld14p_update_cloud(uint8_t *buf) {
    uint8_t crc = ld14p_crc8(buf, LD14P_PACKET_LEN - 1);
    if (crc != buf[LD14P_PACKET_LEN - 1]) {
        crc_fail_count++;
        if (crc_fail_count <= 3)
            ESP_LOGW(TAG, "CRC fail #%lu calc=%02x recv=%02x", crc_fail_count, crc, buf[LD14P_PACKET_LEN - 1]);
        return;
    }
    crc_ok_count++;

    uint16_t start_angle = (buf[5] << 8) | buf[4];
    uint16_t end_angle   = (buf[43] << 8) | buf[42];

    if (prev_start_angle > 30000 && start_angle < 6000) {
        revolution_flag = true;
        ESP_LOGI(TAG, "Revolution complete! pkts_ok=%lu pkts_fail=%lu start=%u end=%u",
                 crc_ok_count, crc_fail_count, start_angle, end_angle);
    }
    prev_start_angle = start_angle;

    int16_t diff = end_angle - start_angle;
    if (diff < 0) diff += 360 * LD14P_ANGLE_RES;
    float step = (float)diff / (LD14P_POINT_PER_PACK - 1);

    for (int i = 0; i < LD14P_POINT_PER_PACK; i++) {
        int raw  = start_angle + (int)(i * step);
        int deg  = (raw / LD14P_ANGLE_RES) % LD14P_POINTS_PER_REV;
        cloud_360[deg].distance  = (buf[3 * i + 7] << 8) | buf[3 * i + 6];
        cloud_360[deg].intensity = buf[3 * i + 8];
    }
}

void ld14p_feed_byte(uint8_t byte) {
    static uint8_t state = 0;
    static uint8_t buf[LD14P_PACKET_LEN];
    static uint8_t idx = 0;

    if (state == 0) {
        if (byte == LD14P_HEADER) {
            buf[0] = LD14P_HEADER;
            idx = 1;
            state = 1;
        }
    } else {
        buf[idx++] = byte;
        if (idx >= LD14P_PACKET_LEN) {
            state = 0;
            idx = 0;
            ld14p_update_cloud(buf);
        }
    }
}

bool ld14p_scan_ready(void) {
    bool ret = revolution_flag;
    if (ret) revolution_flag = false;
    return ret;
}

esp_err_t ld14p_get_scan(vector_polar_t *out, uint16_t *count) {
    if (!out || !count) return ESP_ERR_INVALID_ARG;

    *count = LD14P_POINTS_PER_REV;
    for (int i = 0; i < LD14P_POINTS_PER_REV; i++) {
        out[i].angle_deg   = (float)i;
        out[i].distance_mm = (float)cloud_360[i].distance;
    }
    return ESP_OK;
}

static void ld14p_set_freq(uint8_t freq_hz) {
    uint16_t speed = (uint16_t)freq_hz * 360;
    uint8_t cmd[8] = { 0x54, LD14P_CMD_SPEED, LD14P_CMD_LEN,
                       (uint8_t)(speed & 0xFF), (uint8_t)(speed >> 8), 0x00, 0x00, 0x00 };
    cmd[7] = ld14p_crc8(cmd, 7);
    uart_write_bytes(LD14P_UART_NUM, cmd, sizeof(cmd));
    ESP_LOGI(TAG, "Set freq %d Hz (%d deg/s)", freq_hz, speed);
}

esp_err_t ld14p_init(uint8_t freq_hz) {
    if (freq_hz < 2 || freq_hz > 8) return ESP_ERR_INVALID_ARG;

    uart_config_t cfg = {
        .baud_rate  = LD14P_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(LD14P_UART_NUM, LD14P_UART_RX_BUF, 0, 0, NULL, 0);
    if (err != ESP_OK) return err;
    err = uart_param_config(LD14P_UART_NUM, &cfg);
    if (err != ESP_OK) return err;
    err = uart_set_pin(LD14P_UART_NUM, LD14P_UART_TX_PIN, LD14P_UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    memset(cloud_360, 0, sizeof(cloud_360));
    vTaskDelay(pdMS_TO_TICKS(100));  // let LD14P stabilize before command
    ld14p_set_freq(freq_hz);

    ESP_LOGI(TAG, "LD14P ready on UART1 (TX:17 RX:18) @ %d Hz", freq_hz);
    return ESP_OK;
}
