/**
 * @file motor.c
 * @brief 电机驱动底层实现 — DRV8833 + LEDC PWM
 */
#include "drivers/drv8833.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "MOTOR ";

// LEDC 通道映射
#define AIN1_CHANNEL    LEDC_CHANNEL_0
#define AIN2_CHANNEL    LEDC_CHANNEL_1
#define BIN1_CHANNEL    LEDC_CHANNEL_2
#define BIN2_CHANNEL    LEDC_CHANNEL_3
#define A_TIMER     LEDC_TIMER_0
#define B_TIMER     LEDC_TIMER_1

/**
 * @brief 设置单个 LEDC 通道占空比
 */
static inline void ledc_duty_apply(ledc_channel_t channel, uint32_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

/**
 * @brief 控制一侧电机的转速比例（-100~100），已修正克服静摩擦力的占空比下限
 */
static void motor_set_side(ledc_channel_t in1_ch, ledc_channel_t in2_ch, int8_t percent) {
    uint8_t  abs_pct;
    uint32_t duty;

    // 钳位
    if (percent > 100)  percent = 100;
    if (percent < -100) percent = -100;

    if (percent == 0) {
        ledc_duty_apply(in1_ch, 0);
        ledc_duty_apply(in2_ch, 0);
        return;
    }

    abs_pct = (uint8_t)(percent < 0 ? -percent : percent);
    // [MIN_EFF, MAX] 线性映射，转换为统一的 [1, 100] 牵引力比例
    duty = MOTOR_MIN_COUNT
         + ((uint32_t)(abs_pct - 1) * (MOTOR_MAX_COUNT - MOTOR_MIN_COUNT)) / 99U;

    if (percent > 0) {
        ledc_duty_apply(in1_ch, duty);
        ledc_duty_apply(in2_ch, 0);
    } else {
        ledc_duty_apply(in1_ch, 0);
        ledc_duty_apply(in2_ch, duty);
    }
}

esp_err_t motor_init(void) {

    // 配置 LEDC 定时器
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = A_TIMER,
        .freq_hz = MOTOR_PWM_FREQ,
        .clk_cfg = LEDC_USE_APB_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "Left timer config failed");

    timer_cfg.timer_num = B_TIMER;
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "Right timer config failed");

    // 配置 4 路 LEDC 通道
    ledc_channel_config_t ch_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty = 0,
        .hpoint = 0,
    };

    // AIN1：ch0, timer0
    ch_cfg.gpio_num = MOTOR_AIN1_PIN;
    ch_cfg.channel = AIN1_CHANNEL;
    ch_cfg.timer_sel = A_TIMER;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "Left IN1 channel failed");

    // AIN2：ch1, timer0
    ch_cfg.gpio_num = MOTOR_AIN2_PIN;
    ch_cfg.channel = AIN2_CHANNEL;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "Left IN2 channel failed");

    // BIN1：ch2, timer1
    ch_cfg.gpio_num = MOTOR_BIN1_PIN;
    ch_cfg.channel = BIN1_CHANNEL;
    ch_cfg.timer_sel = B_TIMER;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "Right IN1 channel failed");

    // BIN2：ch3, timer1
    ch_cfg.gpio_num = MOTOR_BIN2_PIN;
    ch_cfg.channel = BIN2_CHANNEL;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "Right IN2 channel failed");

    // 初始化为滑行状态 (所有通道占空比 0)
    motor_coast();
    ESP_LOGI(TAG, "Motor initialized (20kHz, 10-bit, max_eff=%d, min_eff=%d)", MOTOR_MAX_COUNT, MOTOR_MIN_COUNT);
    return ESP_OK;
}

// 设置速度
esp_err_t motor_set(int8_t left, int8_t right) {
    motor_set_side(AIN1_CHANNEL, AIN2_CHANNEL, left);
    motor_set_side(BIN1_CHANNEL, BIN2_CHANNEL, right);

    return ESP_OK;
}

// 制动
void motor_brake(void) {
    // IN1=HIGH, IN2=HIGH → 两路低侧 FET 导通 → 电机绕组短接
    const uint32_t full = MOTOR_MAX_COUNT;

    ledc_duty_apply(AIN1_CHANNEL, full);
    ledc_duty_apply(AIN2_CHANNEL, full);
    ledc_duty_apply(BIN1_CHANNEL, full);
    ledc_duty_apply(BIN2_CHANNEL, full);
}

// 滑行
void motor_coast(void) {
    // 全部通道占空比 0 → Hi-Z
    ledc_duty_apply(AIN1_CHANNEL, 0);
    ledc_duty_apply(AIN2_CHANNEL, 0);
    ledc_duty_apply(BIN1_CHANNEL, 0);
    ledc_duty_apply(BIN2_CHANNEL, 0);
}
