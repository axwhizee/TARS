/**
 * @file ds18b20.c
 * @brief DS18B20 数字温度计 1-Wire 底层驱动实现
 *
 * 关键设计:
 *   - GPIO 开漏模式 (OUTPUT_OD) 模拟 1-Wire 双向总线
 *   - 所有 1-Wire 位操作在关中断保护下执行, 保证 μs 级时序
 *   - Dallas CRC8 (多项式 X^8+X^5+X^4+1) 验证暂存器数据完整性
 *   - 外部供电模式: VDD 接 3.3V, DQ 外接 4.7KΩ 上拉
 *
 * ROM 指令: 本驱动仅支持单器件, 始终使用 Skip ROM [CCh]
 */
#include "drivers/ds18b20.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include <math.h>

static const char *TAG = "DS18B20";

/* ────────── 1-Wire ROM 指令码 ────────── */
#define OW_SKIP_ROM         0xCC    /* 跳过 ROM 匹配, 仅单器件 */

/* ────────── DS18B20 功能指令码 ────────── */
#define DS18_CONVERT_T      0x44    /* 启动温度转换 */
#define DS18_WRITE_SCRATCH  0x4E    /* 写暂存器 (TH, TL, config) */
#define DS18_READ_SCRATCH   0xBE    /* 读暂存器 (9 字节) */
#define DS18_COPY_SCRATCH   0x48    /* 暂存器 → EEPROM 保存 */

/* ────────── 1-Wire 时序常量 (μs) ────────── */
#define OW_RST_LOW          480     /* 复位低电平持续时间 */
#define OW_RST_WAIT         70      /* 释放后等待存在脉冲 */
#define OW_RST_TAIL         410     /* 存在脉冲尾部等待 */
#define OW_SLOT_START       2       /* 时隙起始低电平 */
#define OW_SLOT_SAMPLE      12      /* 读时隙采样点 (从下降沿起) */
#define OW_SLOT_TOTAL       60      /* 时隙总时长 (最小 60μs) */
#define OW_RECOVERY         1       /* 时隙间恢复 */

/* ────────── Dallas 1-Wire CRC8 查表 ────────── */
static const uint8_t CRC_TABLE[256] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
    0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
    0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
    0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
    0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
    0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
    0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
    0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
    0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
    0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
    0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
};

static uint8_t dallas_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++)
        crc = CRC_TABLE[crc ^ data[i]];
    return crc;
}

/* ────────── 1-Wire 位操作 (关中断保护 μs 级时序) ────────── */

static inline void ow_low(gpio_num_t pin) {
    gpio_set_level(pin, 0);
}

static inline void ow_release(gpio_num_t pin) {
    gpio_set_level(pin, 1);   /* 开漏模式: 1 = 高阻, 由上拉电阻拉高 */
}

static inline int ow_read(gpio_num_t pin) {
    return gpio_get_level(pin);
}

/**
 * @brief 1-Wire 总线复位 + 检测存在脉冲
 * @return true 存在脉冲收到 (器件在线), false 无器件
 */
static bool ow_reset(gpio_num_t pin) {
    portDISABLE_INTERRUPTS();

    ow_low(pin);
    esp_rom_delay_us(OW_RST_LOW);       /* 480μs 复位脉冲 */
    ow_release(pin);
    esp_rom_delay_us(OW_RST_WAIT);      /* 等待 70μs (DS18B20 在 15-60μs 后拉低) */
    bool present = (ow_read(pin) == 0); /* 采样存在脉冲 */
    esp_rom_delay_us(OW_RST_TAIL);      /* 等待存在脉冲结束 (60-240μs) */

    portENABLE_INTERRUPTS();
    return present;
}

/**
 * @brief 写 1 位到 1-Wire 总线
 *
 * 写 0: 拉低并保持 60μs → 释放
 * 写 1: 拉低 2μs → 释放 (DS18B20 在 15-60μs 窗口采样为高)
 */
static void ow_write_bit(gpio_num_t pin, bool bit) {
    portDISABLE_INTERRUPTS();

    ow_low(pin);                          /* 开始时隙 */
    esp_rom_delay_us(OW_SLOT_START);      /* 2μs */
    if (bit) ow_release(pin);             /* 写 1: 立即释放 */
    esp_rom_delay_us(OW_SLOT_TOTAL - OW_SLOT_START); /* 到 60μs */
    ow_release(pin);                      /* 保证释放 */
    esp_rom_delay_us(OW_RECOVERY);        /* 1μs 恢复 */

    portENABLE_INTERRUPTS();
}

/**
 * @brief 从 1-Wire 总线读 1 位
 *
 * 主机拉低 2μs → 释放 → 等待 10μs → 采样 (DS18B20 在 15μs 内输出有效数据)
 */
static bool ow_read_bit(gpio_num_t pin) {
    portDISABLE_INTERRUPTS();

    ow_low(pin);                          /* 开始读时隙 */
    esp_rom_delay_us(OW_SLOT_START);      /* 2μs */
    ow_release(pin);                      /* 释放总线 */
    esp_rom_delay_us(OW_SLOT_SAMPLE - OW_SLOT_START); /* 再等 10μs, 总共 12μs */
    bool bit = (ow_read(pin) == 1);       /* 采样 */
    esp_rom_delay_us(OW_SLOT_TOTAL - OW_SLOT_SAMPLE); /* 到 60μs */
    esp_rom_delay_us(OW_RECOVERY);        /* 1μs 恢复 */

    portENABLE_INTERRUPTS();
    return bit;
}

static void ow_write_byte(gpio_num_t pin, uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        ow_write_bit(pin, byte & 0x01);
        byte >>= 1;
    }
}

static uint8_t ow_read_byte(gpio_num_t pin) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        if (ow_read_bit(pin)) byte |= (1 << i);
    }
    return byte;
}

/* ────────── DS18B20 配置寄存器值 (TH=75°C, TL=70°C, 不同分辨率) ────────── */

static uint8_t res_to_config(uint8_t bits) {
    switch (bits) {
        case  9: return 0x1F;
        case 10: return 0x3F;
        case 11: return 0x5F;
        default: return 0x7F;   /* 12-bit */
    }
}

static uint32_t res_to_conv_ms(uint8_t bits) {
    switch (bits) {
        case  9: return 94;
        case 10: return 188;
        case 11: return 375;
        default: return 750;
    }
}

/* ────────── DS18B20 公开 API ────────── */

esp_err_t ds18b20_init(void) {
    gpio_num_t pin = (gpio_num_t)DS18B20_GPIO_PIN;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode         = GPIO_MODE_OUTPUT_OD,      /* 开漏: 可拉低或释放 */
        .pull_up_en   = GPIO_PULLUP_ENABLE,       /* 内部上拉 (仍需外接 4.7KΩ) */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    ow_release(pin);    /* 总线空闲 = 高电平 */

    /* 检测总线是否存在器件 */
    if (!ow_reset(pin)) {
        ESP_LOGE(TAG, "No DS18B20 detected on GPIO%d", DS18B20_GPIO_PIN);
        return ESP_ERR_TIMEOUT;
    }

    /* 写配置: Skip ROM → Write Scratchpad → TH, TL, config → 复位中止 */
    uint8_t cfg = res_to_config(DS18B20_RES_BITS);
    ow_write_byte(pin, OW_SKIP_ROM);
    ow_write_byte(pin, DS18_WRITE_SCRATCH);
    ow_write_byte(pin, 0x4B);   /* TH = 75°C */
    ow_write_byte(pin, 0x46);   /* TL = 70°C */
    ow_write_byte(pin, cfg);    /* 分辨率配置 */
    ow_reset(pin);              /* 中止写入 */

    /* 保存到 EEPROM: Skip ROM → Copy Scratchpad */
    ow_write_byte(pin, OW_SKIP_ROM);
    ow_write_byte(pin, DS18_COPY_SCRATCH);
    vTaskDelay(pdMS_TO_TICKS(15));  /* EEPROM 写入需 ≥10ms */

    ESP_LOGI(TAG, "Ready on GPIO%d, %d-bit resolution (%.2f°C, %lums conv)",
             DS18B20_GPIO_PIN, DS18B20_RES_BITS,
             (double)(1.0f / (1 << (DS18B20_RES_BITS - 8))),
             (unsigned long)res_to_conv_ms(DS18B20_RES_BITS));
    return ESP_OK;
}

esp_err_t ds18b20_start_conversion(void) {
    gpio_num_t pin = (gpio_num_t)DS18B20_GPIO_PIN;

    if (!ow_reset(pin)) return ESP_ERR_TIMEOUT;
    ow_write_byte(pin, OW_SKIP_ROM);
    ow_write_byte(pin, DS18_CONVERT_T);
    return ESP_OK;
}

bool ds18b20_poll(void) {
    /* 读时隙: DS18B20 输出 0=转换中, 1=完成 */
    return ow_read_bit((gpio_num_t)DS18B20_GPIO_PIN);
}

float ds18b20_read_temp(void) {
    gpio_num_t pin = (gpio_num_t)DS18B20_GPIO_PIN;

    if (!ow_reset(pin)) {
        ESP_LOGW(TAG, "Bus reset failed during read");
        return NAN;
    }

    ow_write_byte(pin, OW_SKIP_ROM);
    ow_write_byte(pin, DS18_READ_SCRATCH);

    uint8_t buf[9];
    for (int i = 0; i < 9; i++) buf[i] = ow_read_byte(pin);

    if (dallas_crc8(buf, 8) != buf[8]) {
        ESP_LOGW(TAG, "CRC mismatch");
        return NAN;
    }

    int16_t raw = (int16_t)(buf[1] << 8) | buf[0];
    return raw / 16.0f;
}
