/**
 * @file ld14p_port.c
 * @brief 移植层实现 — ESP32-S3 UART 驱动
 *
 * 本文件是唯一允许包含平台 HAL 头文件的部分。
 * 移植到其他平台时，重写本文件并调整 ld14p_port_def.h / ld14p_config.h 即可。
 */
#include "ld14p_port.h"
#include "ld14p_port_def.h"
#include "ld14p_config.h"
#include "driver/uart.h"
#include "esp_log.h"

#if LD14P_RTOS_ACTIVE
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#else
#include "esp_timer.h"
#include "esp_rom_sys.h"
#endif

static const char *TAG = "LD14P_PORT";

ld14p_err_t ld14p_port_uart_init(ld14p_handle_t *h) {
  if (h == NULL) {
    return LD14P_ERR_PARAM;
  }

  h->bus = (void *)(uintptr_t)LD14P_UART_NUM;

  uart_config_t cfg = {
    .baud_rate = LD14P_UART_BAUD,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };

  if (uart_param_config(LD14P_UART_NUM, &cfg) != ESP_OK) {
    ESP_LOGE(TAG, "uart_param_config failed");
    return LD14P_ERR_INIT;
  }
  if (uart_set_pin(LD14P_UART_NUM, LD14P_UART_TX_PIN, LD14P_UART_RX_PIN,
                   UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
    ESP_LOGE(TAG, "uart_set_pin failed");
    return LD14P_ERR_INIT;
  }
  /* event_queue=NULL → 上层必须使用 timeout=0 非阻塞读取 */
  if (uart_driver_install(LD14P_UART_NUM, LD14P_UART_RX_BUF * 2, 0, 0, NULL, 0) != ESP_OK) {
    ESP_LOGE(TAG, "uart_driver_install failed");
    return LD14P_ERR_INIT;
  }
  return LD14P_OK;
}

ld14p_err_t ld14p_port_uart_deinit(ld14p_handle_t *h) {
  if (h == NULL) {
    return LD14P_ERR_PARAM;
  }
  if (h->bus == NULL) {
    return LD14P_OK;
  }
  if (uart_driver_delete((uart_port_t)(uintptr_t)h->bus) != ESP_OK) {
    return LD14P_ERR_UART;
  }
  h->bus = NULL;
  return LD14P_OK;
}

ld14p_err_t ld14p_port_write(ld14p_handle_t *h, const uint8_t *data, size_t len) {
  if (h == NULL || data == NULL || h->bus == NULL) {
    return LD14P_ERR_PARAM;
  }
  int n = uart_write_bytes((uart_port_t)(uintptr_t)h->bus, data, (int)len);
  return (n == (int)len) ? LD14P_OK : LD14P_ERR_UART;
}

ld14p_err_t ld14p_port_read(ld14p_handle_t *h, uint8_t *buf, size_t len,
                            size_t *out_len, uint32_t timeout_ms) {
  if (h == NULL || buf == NULL || out_len == NULL || h->bus == NULL) {
    return LD14P_ERR_PARAM;
  }
  *out_len = 0;
  int n = uart_read_bytes((uart_port_t)(uintptr_t)h->bus, buf, (int)len, timeout_ms);
  if (n < 0) {
    return LD14P_ERR_UART;
  }
  *out_len = (size_t)n;
  return LD14P_OK;
}

void ld14p_port_delay_ms(uint32_t ms) {
#if LD14P_RTOS_ACTIVE
  vTaskDelay(pdMS_TO_TICKS(ms));
#else
  for (uint32_t i = 0; i < ms; i++) {
    esp_rom_delay_us(1000);   /* esp_rom_delay_us 单次上限较短，按毫秒分片 */
  }
#endif
}

uint32_t ld14p_port_get_tick_ms(void) {
#if LD14P_RTOS_ACTIVE
  return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
#else
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
#endif
}
