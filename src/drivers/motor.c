/**
 * @file motor.c
 * @brief 电机驱动底层实现 — DRV8833 + LEDC PWM
 *
 * 控制策略:
 *   - 采用 IN/IN + Fast Decay 模式 (IN1=PWM, IN2=0 为正转)
 *   - 4 路 LEDC 通道: ch0=GPIO4(AIN1), ch1=GPIO5(AIN2),
 *                     ch2=GPIO6(BIN1), ch3=GPIO7(BIN2)
 *   - Timer 0 供左侧 2 通道共用, Timer 1 供右侧 2 通道共用
 *   - 10-bit 分辨率 (0-1023), 20kHz 载波频率
 *   - 最大占空比限制 80% (=819) 防止堵转过流
 */

#include "drivers/motor.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include <string.h>

static const char *TAG = "MOTOR";

/* ---------------------------------------------------------- */
/* LEDC 通道映射                                                */
/* ---------------------------------------------------------- */
#define LEFT_IN1_CHANNEL  LEDC_CHANNEL_0
#define LEFT_IN2_CHANNEL  LEDC_CHANNEL_1
#define RIGHT_IN1_CHANNEL LEDC_CHANNEL_2
#define RIGHT_IN2_CHANNEL LEDC_CHANNEL_3

#define LEFT_TIMER   LEDC_TIMER_0
#define RIGHT_TIMER  LEDC_TIMER_1

static bool initialized = false;

esp_err_t motor_init(void)
{
    if (initialized) return ESP_OK;

    /* 安装 LEDC 渐变服务 (必须先调用) */
    ledc_fade_func_install(0);

    /* ---------- 配置 LEDC 定时器 ---------- */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LEFT_TIMER,
        .freq_hz         = MOTOR_PWM_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "Left timer config failed");

    timer_cfg.timer_num = RIGHT_TIMER;
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "Right timer config failed");

    /* ---------- 配置 4 路 LEDC 通道 ---------- */
    ledc_channel_config_t ch_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty       = 0,
        .hpoint     = 0,
    };

    /* 左侧 IN1 — GPIO4, ch0, timer0 */
    ch_cfg.gpio_num   = MOTOR_LEFT_IN1_GPIO;
    ch_cfg.channel    = LEFT_IN1_CHANNEL;
    ch_cfg.timer_sel  = LEFT_TIMER;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "Left IN1 channel failed");

    /* 左侧 IN2 — GPIO5, ch1, timer0 */
    ch_cfg.gpio_num   = MOTOR_LEFT_IN2_GPIO;
    ch_cfg.channel    = LEFT_IN2_CHANNEL;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "Left IN2 channel failed");

    /* 右侧 IN1 — GPIO6, ch2, timer1 */
    ch_cfg.gpio_num   = MOTOR_RIGHT_IN1_GPIO;
    ch_cfg.channel    = RIGHT_IN1_CHANNEL;
    ch_cfg.timer_sel  = RIGHT_TIMER;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "Right IN1 channel failed");

    /* 右侧 IN2 — GPIO7, ch3, timer1 */
    ch_cfg.gpio_num   = MOTOR_RIGHT_IN2_GPIO;
    ch_cfg.channel    = RIGHT_IN2_CHANNEL;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "Right IN2 channel failed");

    /* 初始化为滑行状态 (所有通道占空比 0) */
    motor_coast();

    initialized = true;
    ESP_LOGI(TAG, "Motor driver initialized (20kHz, 10-bit, max duty=%d)", MOTOR_MAX_DUTY);
    return ESP_OK;
}

/* ---------------------------------------------------------- */
/* 单侧电机控制: IN/IN + Fast Decay                            */
/* ---------------------------------------------------------- */

/**
 * @brief 设置单个 LEDC 通道占空比并立即生效
 */
static inline void ledc_duty_apply(ledc_channel_t channel, uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

/**
 * @brief 控制单侧电机的 IN1/IN2 信号
 *
 * @param in1_ch  IN1 对应的 LEDC 通道
 * @param in2_ch  IN2 对应的 LEDC 通道
 * @param speed   有符号速度，正数=前进，负数=后退，零=滑行
 *                绝对值范围 [0, MOTOR_MAX_DUTY]
 *
 * IN/IN 真值表 (Fast Decay):
 * | speed | IN1  | IN2  | 效果         |
 * | >0    | PWM  | 0    | 正转 (前进)  |
 * | <0    | 0    | PWM  | 反转 (后退)  |
 * | =0    | 0    | 0    | 滑行 (Coast) |
 */
static void motor_set_side(ledc_channel_t in1_ch, ledc_channel_t in2_ch,
                           int16_t speed)
{
    uint32_t duty;

    /* 钳位到允许范围 */
    if (speed > (int16_t)MOTOR_MAX_DUTY)  speed = (int16_t)MOTOR_MAX_DUTY;
    if (speed < -(int16_t)MOTOR_MAX_DUTY) speed = -(int16_t)MOTOR_MAX_DUTY;

    duty = (uint32_t)(speed < 0 ? -speed : speed);

    if (speed > 0) {
        /* 正转: IN1 = PWM, IN2 = 0 */
        ledc_duty_apply(in1_ch, duty);
        ledc_duty_apply(in2_ch, 0);
    } else if (speed < 0) {
        /* 反转: IN1 = 0, IN2 = PWM */
        ledc_duty_apply(in1_ch, 0);
        ledc_duty_apply(in2_ch, duty);
    } else {
        /* 滑行: IN1 = 0, IN2 = 0 */
        ledc_duty_apply(in1_ch, 0);
        ledc_duty_apply(in2_ch, 0);
    }
}

esp_err_t motor_set(int16_t left, int16_t right)
{
    if (!initialized) {
        ESP_LOGE(TAG, "Motor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    motor_set_side(LEFT_IN1_CHANNEL,  LEFT_IN2_CHANNEL,  left);
    motor_set_side(RIGHT_IN1_CHANNEL, RIGHT_IN2_CHANNEL, right);

    return ESP_OK;
}

/* ---------------------------------------------------------- */
/* 制动 & 滑行                                                   */
/* ---------------------------------------------------------- */

void motor_brake(void)
{
    if (!initialized) return;

    /* 制动: IN1=HIGH, IN2=HIGH → 两路低侧 FET 导通 → 电机绕组短接
     *       通过设置 100% 占空比实现持续 HIGH 电平 */
    const uint32_t full = (1U << MOTOR_PWM_RES_BITS) - 1; /* 1023 */

    ledc_duty_apply(LEFT_IN1_CHANNEL,  full);
    ledc_duty_apply(LEFT_IN2_CHANNEL,  full);
    ledc_duty_apply(RIGHT_IN1_CHANNEL, full);
    ledc_duty_apply(RIGHT_IN2_CHANNEL, full);
}

void motor_coast(void)
{
    if (!initialized) return;

    /* 滑行: 全部通道占空比 0 → Hi-Z */
    ledc_duty_apply(LEFT_IN1_CHANNEL,  0);
    ledc_duty_apply(LEFT_IN2_CHANNEL,  0);
    ledc_duty_apply(RIGHT_IN1_CHANNEL, 0);
    ledc_duty_apply(RIGHT_IN2_CHANNEL, 0);
}

void motor_deinit(void)
{
    if (!initialized) return;

    motor_coast();

    ledc_stop(LEDC_LOW_SPEED_MODE, LEFT_IN1_CHANNEL,  0);
    ledc_stop(LEDC_LOW_SPEED_MODE, LEFT_IN2_CHANNEL,  0);
    ledc_stop(LEDC_LOW_SPEED_MODE, RIGHT_IN1_CHANNEL, 0);
    ledc_stop(LEDC_LOW_SPEED_MODE, RIGHT_IN2_CHANNEL, 0);

    initialized = false;
}
