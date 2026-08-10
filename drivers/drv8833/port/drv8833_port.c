/**
 * @file drv8833_port.c
 * @brief 移植层实现 — ESP32-S3 LEDC PWM 驱动
 *
 * 本文件是唯一允许包含平台 HAL 头文件的部分。
 * 移植到其他平台时，重写本文件并调整 drv8833_port_def.h / drv8833_config.h 即可。
 */
#include "drv8833_port.h"
#include "drv8833_port_def.h"
#include "drv8833_config.h"
#include "driver/ledc.h"
#include "esp_log.h"

#if DRV8833_RTOS_ACTIVE
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#else
#include "esp_timer.h"
#include "esp_rom_sys.h"
#endif

static const char *TAG = "DRV8833_PORT";

/* 逻辑通道号 → LEDC 通道 / 定时器 / GPIO 映射 */
static const struct {
  ledc_channel_t ch;
  ledc_timer_t   timer;
  gpio_num_t     pin;
} ch_map[4] = {
  [DRV8833_IN_AIN1] = { DRV8833_CH_AIN1, DRV8833_TIMER_A, DRV8833_AIN1_PIN },
  [DRV8833_IN_AIN2] = { DRV8833_CH_AIN2, DRV8833_TIMER_A, DRV8833_AIN2_PIN },
  [DRV8833_IN_BIN1] = { DRV8833_CH_BIN1, DRV8833_TIMER_B, DRV8833_BIN1_PIN },
  [DRV8833_IN_BIN2] = { DRV8833_CH_BIN2, DRV8833_TIMER_B, DRV8833_BIN2_PIN },
};

/* 设置单个 LEDC 通道占空比 */
static void ledc_duty_apply(ledc_channel_t channel, uint32_t duty) {
  ledc_set_duty(DRV8833_LEDC_MODE, channel, duty);
  ledc_update_duty(DRV8833_LEDC_MODE, channel);
}

drv8833_err_t drv8833_port_pwm_init(drv8833_handle_t *h) {
  if (h == NULL) {
    return DRV8833_ERR_PARAM;
  }
  h->bus = (void *)(uintptr_t)1;   /* LEDC 无句柄，哨兵标记已初始化 */

  /* 配置 LEDC 定时器 */
  ledc_timer_config_t timer_cfg = {
    .speed_mode = DRV8833_LEDC_MODE,
    .duty_resolution = (ledc_timer_bit_t)DRV8833_PWM_RES_BITS,
    .timer_num = DRV8833_TIMER_A,
    .freq_hz = DRV8833_PWM_FREQ_HZ,
    .clk_cfg = DRV8833_LEDC_CLK_CFG,
  };
  if (ledc_timer_config(&timer_cfg) != ESP_OK) {
    ESP_LOGE(TAG, "Timer A config failed");
    return DRV8833_ERR_INIT;
  }

  timer_cfg.timer_num = DRV8833_TIMER_B;
  if (ledc_timer_config(&timer_cfg) != ESP_OK) {
    ESP_LOGE(TAG, "Timer B config failed");
    return DRV8833_ERR_INIT;
  }

  /* 配置 4 路 LEDC 通道 */
  ledc_channel_config_t ch_cfg = {
    .speed_mode = DRV8833_LEDC_MODE,
    .duty = 0,
    .hpoint = 0,
  };
  for (int i = 0; i < 4; i++) {
    ch_cfg.gpio_num = ch_map[i].pin;
    ch_cfg.channel = ch_map[i].ch;
    ch_cfg.timer_sel = ch_map[i].timer;
    if (ledc_channel_config(&ch_cfg) != ESP_OK) {
      ESP_LOGE(TAG, "Channel %d config failed", i);
      return DRV8833_ERR_INIT;
    }
  }
  return DRV8833_OK;
}

drv8833_err_t drv8833_port_pwm_deinit(drv8833_handle_t *h) {
  if (h == NULL) {
    return DRV8833_ERR_PARAM;
  }
  if (h->bus == NULL) {
    return DRV8833_OK;
  }
  for (int i = 0; i < 4; i++) {
    ledc_duty_apply(ch_map[i].ch, 0);
  }
  h->bus = NULL;
  return DRV8833_OK;
}

drv8833_err_t drv8833_port_set_duty(drv8833_handle_t *h, uint8_t ch, uint32_t duty) {
  if (h == NULL || h->bus == NULL || ch > 3) {
    return DRV8833_ERR_PARAM;
  }
  if (duty > DRV8833_MAX_DUTY) {
    duty = DRV8833_MAX_DUTY;
  }
  ledc_duty_apply(ch_map[ch].ch, duty);
  return DRV8833_OK;
}

void drv8833_port_delay_ms(uint32_t ms) {
#if DRV8833_RTOS_ACTIVE
  vTaskDelay(pdMS_TO_TICKS(ms));
#else
  for (uint32_t i = 0; i < ms; i++) {
    esp_rom_delay_us(1000);   /* esp_rom_delay_us 单次上限较短，按毫秒分片 */
  }
#endif
}

uint32_t drv8833_port_get_tick_ms(void) {
#if DRV8833_RTOS_ACTIVE
  return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
#else
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
#endif
}
