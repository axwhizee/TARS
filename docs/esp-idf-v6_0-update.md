# ESP-IDF v6.0 变更指南 (自 v5.x 迁移)

> 目标板: **ESP32-S3** (N16R8)。本文只记录 **v6.0 相对 v5.x 的变化**,环境搭建 / 构建 / 调试流程见《开发环境搭建》与 `AGENTS.md`。

---

## 1. 组件体系 (最重要)

- **`driver` 组件不再自动带出子驱动依赖**。用哪个外设,就在 `main/CMakeLists.txt` 的 `REQUIRES` 里**显式加对应 `esp_driver_xxx` 组件**,否则报 `Failed to resolve component 'esp_driver_xxx'` 或找不到头文件。
- **头文件路径不变** —— `include` 写法与 v5 相同,只是缺了组件声明会报错。

| include 的头文件 | REQUIRES 添加 |
|---|---|
| `driver/spi_master.h` / `spi_slave.h` | `esp_driver_spi` |
| `driver/uart.h` | `esp_driver_uart` |
| `driver/gpio.h` | `esp_driver_gpio` |
| `driver/ledc.h` | `esp_driver_ledc` |
| `driver/rmt_tx.h` / `rmt_rx.h` | `esp_driver_rmt` |
| `driver/temperature_sensor.h` | `esp_driver_tsens` |
| `driver/gptimer.h` | `esp_driver_gptimer` |
| `driver/i2c_master.h` / `i2c_slave.h` | `esp_driver_i2c` |
| `driver/i2s_std.h` / `i2s_tdm.h` | `esp_driver_i2s` |
| `driver/pulse_cnt.h` | `esp_driver_pcnt` |
| `driver/mcpwm_prelude.h` | `esp_driver_mcpwm` |

**新外设三步**: ① `REQUIRES` 加组件 → ② `#include` 照旧 → ③ 用到 FreeRTOS 队列/信号量时显式 `#include "freertos/queue.h"`、`"freertos/semphr.h"`(v6.0 驱动头不再隐式包含 FreeRTOS)。

> **clangd 局限**: `--check` 不抓组件缺失(只看头文件在不在);**`idf.py build` 才报依赖缺失**,加新外设后务必 build 验证。

## 2. 内置组件 → IDF Component Manager (托管组件)

以下组件已移出 IDF 内置目录,需在 `main/idf_component.yml` 声明或 `idf.py add-dependency`:

| 组件 | 声明 |
|---|---|
| cJSON (原 `json`) | `espressif/cjson: "^1.7.19"` |
| ESP-MQTT | `espressif/mqtt` (API 不变,头 `mqtt_client.h`) |
| USB / touch_element / NT35510 LCD / gcov / SystemView | `espressif/usb` / `espressif/touch_element` / `espressif/esp_lcd_nt35510` 等 |

> 本仓库已把 cJSON **vender 进 `libs/cJSON/`**(见 §4),不再用托管组件 —— 因此本仓库 `main/` 下**没有** `idf_component.yml`。

## 3. 已移除的 API / 驱动 (替换项)

- **彻底删除的遗留驱动**: `driver/adc.h`, `driver/mcpwm.h`, `driver/i2s.h`, `driver/pcnt.h`, `driver/rmt.h`, `driver/timer.h`, `driver/temp_sensor.h`, `driver/sigmadelta.h` → 全部换新驱动。
- **I2C**: 遗留 `driver/i2c.h` 标记 EOL(预计 v7.0 移除),新驱动 `driver/i2c_master.h`/`i2c_slave.h`;slave 改回调模式。
- **GPIO**: `gpio_iomux_in/out()` → `gpio_iomux_input/output()`;`gpio_deep_sleep_wakeup_*` → `gpio_wakeup_*_on_hp_periph_powerdown_sleep()`。
- **LEDC**: `ledc_timer_set()` 移除 → `ledc_timer_config()` / `ledc_set_freq()`;`LEDC_APB_CLK_HZ`/`LEDC_REF_CLK_HZ` 移除。
- **UART**: `UART_FIFO_LEN` → `UART_HW_FIFO_LEN`;`soc/uart_channel.h` → `soc/uart_pins.h`。
- **FreeRTOS**: `xTaskGetAffinity()` → `xTaskGetCoreID()`;`vTaskDelayUntil()` → `xTaskDelayUntil()`;`xQueueGenericReceive()` → `xQueueReceive()` 等。
- **Log**: `esp_log_buffer_hex()/char()` → `ESP_LOG_BUFFER_HEX()/CHAR()`。
- **系统**: `esp_sleep_get_wakeup_cause()` → `esp_sleep_get_wakeup_causes()`(位图);`EXT_RAM_ATTR` → `EXT_RAM_BSS_ATTR`;`esp_ota_get_app_description()` → `esp_app_get_description()`。

## 4. 构建 / 运行时行为变化

- **默认 LibC 从 Newlib 切到 Picolibc**: 更省内存;但 `stdin/stdout/stderr` 变为全局共享,不能按任务重定义。可用 `CONFIG_LIBC` 选回 Newlib。
- **默认编译警告当作错误** (`CONFIG_COMPILER_DISABLE_DEFAULT_ERRORS=n`)。
- **FreeRTOS/ringbuf 函数默认放 flash** (省 IRAM),可用 `CONFIG_FREERTOS_IN_IRAM` 恢复 IRAM。
- **链接器 orphan section 报错**: 链接失败时检查未放入 linker script 的段,或 `CONFIG_COMPILER_ORPHAN_SECTIONS` 降级。
- 最小 CMake **3.22.1**;最小 Python **3.10**;`idf.py size --format json2`(原 `json`);`efuse*` 命令必须 `--port`。
- 全局构造函数顺序改为**升序** (`__libc_init_array()`),依赖旧降序的注册逻辑需调整。

## 5. 添加组件 / 库的流程

### 5.1 IDF 内置组件 (外设驱动)

见 §1 三步 + 组件映射表。

### 5.2 三方库 (推荐: 统一放 `libs/`) ⭐

**不推荐用 `idf_component.yml` 引入三方托管库**。原因:
- 引入额外机制(组件管理器下载、`managed_components/`、版本锁)。
- 增加一处 `.yml` 需要维护的构建配置。
- `libs/` 已被 GLOB 自动编译,放进去即用,零额外步骤、零联网、版本完全自控。

**流程** (以 cJSON 为例):
1. 从 GitHub / 托管组件源码下载,放入 `libs/<lib>/`:
   ```
   libs/cJSON/cJSON.c
   libs/cJSON/cJSON.h
   ```
2. GLOB 自动编译(`main/CMakeLists.txt` 已含 `libs/*.c`),无需登记。
3. `INCLUDE_DIRS` 已含 `libs/`,代码里:
   ```c
   #include "cJSON/cJSON.h"    # 以库子目录区分,避免头文件名冲突
   ```
4. 用 cJSON 的宏 `CJSON_NESTING_LIMIT` / `CJSON_CIRCULAR_LIMIT` 用头文件内置默认值即可 (1000 / 10000)。

**版本管理**: 在 `libs/<lib>/README.md` 注明来源 URL 与版本号。

### 5.3 托管组件 (备选,慎用)

仅当三方库**无法直接放入 `libs/`** 且强烈依赖 IDF 组件生态时,才用 `main/idf_component.yml`:

```yaml
dependencies:
  espressif/cjson: "^1.7.19"
```

- REQUIRES 写组件名(不带 `espressif/` 前缀)。
- 源码自动下载到 `managed_components/`(已 gitignore),版本写入 `dependencies.lock`。
- 换机后 `idf.py build` 自动重新拉取。

---

## 6. 注意事项速查

| 主题 | 要点 |
|---|---|
| 组件缺失 | clangd 不抓,`idf.py build` 才报;加新外设务必 build |
| clangd 误报 | `drv_unknown_argument`(GCC-15/Xtensa 专属 flag)用 `.clangd` `CompileFlags.Remove` 屏蔽 |
| 警告报错 | v6.0 默认 warning-as-error,`CONFIG_COMPILER_DISABLE_DEFAULT_ERRORS` 可关 |
| 链接 orphan section | 检查未放入 linker script 的段;或 `CONFIG_COMPILER_ORPHAN_SECTIONS=place` 降级 |
