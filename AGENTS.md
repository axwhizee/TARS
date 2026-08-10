# ESP32 TARS — Native ESP-IDF (idf.py)

Target: **esp32-s3** (N16R8: 16MB flash, 8MB Octal PSRAM). Framework: **ESP-IDF v6.0.2** (native `idf.py`).
Build runs on **Windows** (VSCode tasks) — manual install, unified at `C:\Users\Richa\.espressif`: IDF source per version at `~/.espressif\v6.0.2\esp-idf`, tools at `~/.espressif\tools` (`IDF_TOOLS_PATH=~/.espressif`).
From WSL, drive the build via PowerShell + `~/.espressif/activate-idf.ps1` (see below).

## Build & Flash & Debug

Recommended: VSCode tasks (`.vscode/tasks.json`) — build / flash / monitor / menuconfig / clean / openocd. Debug via F5 (`.vscode/launch.json`, OpenOCD + GDB over board USB-JTAG).

CLI on Windows (PowerShell):
```powershell
# activate IDF environment once (unified manual install)
. 'C:\Users\Richa\.espressif\activate-idf.ps1'

idf.py set-target esp32s3     # one-time (already in sdkconfig.defaults)
idf.py build                  # build → build/TARS.bin + build/spiffs.bin
idf.py flash                  # flash firmware + SPIFFS
idf.py monitor                # serial monitor
idf.py flash monitor          # full: flash + monitor
idf.py menuconfig
idf.py fullclean
```

WSL cross-call variant (single-level, no nested `powershell -Command`):
```
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command ". 'C:/Users/Richa/.espressif/activate-idf.ps1'; Set-Location 'C:/Users/Richa/WorkSpace/MCUs/ESP32/Projects/TARS'; idf.py build"
```
> **activate-idf.ps1 rules**: ① Must be **UTF-8 with BOM** (PowerShell 5.1 mis-parses UTF-8-no-BOM Chinese comments → syntax errors). ② Must be called with **dot-source** (`.`, not `&`) — it defines a `global:idf.py` function, and `idf.py` must survive into the caller's scope. ③ It sets `IDF_PATH`/`IDF_TOOLS_PATH`/`IDF_PYTHON_ENV_PATH`, prepends venv `Scripts` to PATH (this machine's `python` is a Windows Store stub), then dot-sources IDF native `export.ps1`. EIM is **not used** — manual install. ④ **Anti-re-entry guard**: if `IDF_PATH` already equals the v6.0.2 path, the script returns immediately (~2s saved). VSCode tasks use `"presentation": { "group": "idf" }` so all `idf:` tasks share one terminal — only the first task per terminal pays full activation.
> **VSCode task gotcha**: task commands must NOT wrap in `powershell -Command "..."` — VSCode's shell task already runs in a PowerShell terminal; nesting creates a second scope where the `idf.py` function is lost. Write the bare command (`. '...'; idf.py build`).

## Source Layout

| Path | Role |
|---|---|
| `main/main.c` | Entrypoint — NVS/SPIFFS/Wi-Fi init, hardware drivers, 4 queues + 1 event group, 7 tasks |
| `main/sys_init.c` | NVS flash → SPIFFS `/spiffs` → Wi-Fi AP (`TARS`, open, `192.168.10.1/24`, DHCP off) |
| `main/apf_common.h` | Global types, RTOS handle `extern`s, event-group bits, GPIO pins, APF/motor/WiFi constants |
| `drivers/` | `ld14p/` subdriver (LD14P LiDAR, **adapt-drv v6.0 分层**: `ld14p.h/.c` 核心层 + `ld14p_config.h` + `ld14p_types.h` + `internal/` + `port/` + `README.md`); `drv8833/` subdriver (DRV8833 底盘驱动, 同分层, 含 `internal/` 底盘算法 + `port/` LEDC 移植层); `ds18b20.c` (**dead code**) — 独立 `.c/.h` 同目录 |
| `tasks/` | 7 FreeRTOS tasks (prio 1–8, see main.c:93-99): `sysmon_task`(1), `web_task`(3, Core0), `apf_task`(4), `ld14p_task`(5), `temp_task`(6), `flame_task`(7), `motor_task`(8) |
| `data/` | SPIFFS web UI — `index.html`, `script.js`, `style.css` (Canvas polar radar, WebSocket client). Built into `build/spiffs.bin` via `spiffs_create_partition_image` |
| `libs/` | Vendored third-party libs — `cJSON/cJSON.c` (v1.7.19, used by web_task) + two unused zips |
| `.vscode/` | `tasks.json` (build/flash/menuconfig/clean/openocd), `launch.json` (OpenOCD+GDB debug), `settings.json` (clangd) |
| `PCB/` | Altium project |

## Partition Table (`default_16MB.csv`)

```
nvs(16KB) → otadata(8KB) → phy_init(4KB) → factory(2MB) → ota_0(2MB) → ota_1(2MB) → spiffs(1MB)
```

## ESP-IDF v6.0 — 要点速查

> 环境搭建 / 构建 / 调试见上文与《开发环境搭建》;v6.0 变化指南见 **`docs/esp-idf-v6_0-update.md`**。

- **加新外设必做**: 在 `main/CMakeLists.txt` 的 `REQUIRES` 显式加对应 `esp_driver_xxx`(如 SPI → `esp_driver_spi`);`include` 路径不变;用到 FreeRTOS 队列/信号量时显式 `#include "freertos/queue.h"`。clangd 不抓组件缺失,**`idf.py build` 才报**。
- **cJSON 已 vendor 进 `libs/cJSON/`**(v1.7.19,`#include "cJSON/cJSON.h"`),**不再使用托管组件**;`main/` 下无 `idf_component.yml`。ESP-MQTT、USB 等如需托管组件,才用 `main/idf_component.yml` / `idf.py add-dependency`。

## Conventions

- **All includes** use `""`. `main/CMakeLists.txt` sets `INCLUDE_DIRS` to `{main/, tasks/, drivers/, drivers/ld14p/, drivers/drv8833/, libs/, 项目根}` so `"apf_common.h"`, `"drivers/ld14p/ld14p.h"`, `"drivers/drv8833/drv8833.h"`, `"drivers/*.h"`, `"tasks/*.h"` all resolve. Headers live next to their `.c` (no `include/` mirror).
- **Source discovery**: `main/CMakeLists.txt` does `FILE(GLOB_RECURSE ...)` over `main/` + `drivers/` + `libs/` — every `.c` auto-included (含 `drivers/ld14p/`、`drivers/drv8833/` 子目录).
- **Driver structure**: 外设驱动遵循 `adapt-drv` 分层模式 (见 `.opencode/skills/adapt-drv`) — 核心层 `xxx.h/.c`(无平台头) + `xxx_config.h` + `xxx_types.h` + `internal/`(核心内部) + `port/`(唯一平台 HAL, 含 `port_def.h` 硬件资源) + `README.md`。平台 HAL 头文件只允许出现在 `port/xxx_port.c`。
- **Required IDF components**: `driver`, `esp_driver_gpio`, `esp_driver_ledc`, `esp_driver_rmt`, `esp_driver_tsens`, `esp_driver_uart`, `esp_event`, `esp_http_server`, `esp_netif`, `esp_timer`, `esp_wifi`, `nvs_flash`, `spiffs`. When adding a new peripheral, see the component table in "ESP-IDF v6.0 Key Changes".
- **FreeRTOS**: `xTaskCreate` stack size in **words**. `q_cart` uses `xQueueOverwrite` → depth=1.
- **Dual-core pinning**: Core 0 = WiFi + lwIP + `web_task` (network); Core 1 = all sensor/control tasks.
- **WebSocket** at `ws://192.168.10.1:80/ws`. HTTP file server at `http://192.168.10.1:80/`.
- **Outgoing JSON is hand-built** via `snprintf` in `web_task.c` (avoids cJSON heap fragmentation). **Incoming** WS messages use `cJSON_ParseWithLength` — small temporary allocations are acceptable for the 2-field JSON parse.
- **Static files preloaded** from SPIFFS to heap at startup (`preload_static_files()`), then served from RAM — no FILE\* contention at runtime.
- **WS protocol**: All upstream messages use `action` field for unified dispatch. Periodic telemetry (4Hz) carries `v_apf` (APF repulsion), `v_vfh` (VFH attraction), and `v_polars` (77 sensor points) — combined cmd is computed client-side as `v_apf + v_vfh`. Event responses (`mode_changed`, `params`) are sent on-demand. Param validation uses a data-driven `PARAM_FIELDS[]` table.
- **LIDAR_SECTORS = 72** — 360° → 72 sectors downsampling with min-weighted average. `Q_POLAR_DEPTH = LIDAR_SECTORS + FLAME_SENSOR_COUNT = 77`.
- **LPF 一阶低通滤波**: drv8833 核心层对 dx/dy 一阶低通 (`DRV8833_LPF_ALPHA` 0.90,`DRV8833_LPF_ENABLE`/`drv8833_set_lpf` 可启停);apf_task.c 对 VFH 目标角度跨帧 EMA (`VFH_EMA_ALPHA` 0.80,环绕安全处理)。(lidar_task.c **无** 滤波)
- **Motor control**: drv8833 核心层底盘算法 (`internal/drv8833_chassis_diff4wd.c`): **dx/dy 直接差速映射**,无角度分解。`propulsion = dx × scale(|dy|)`, `steering = dy × STEER_GAIN`;自旋时 `× SPIN_GAIN`,后退 `× REV_GAIN`,超时 `DRV8833_TIMEOUT_MS` 归零。底盘类型由 `DRV8833_CHASSIS_SELECT` 宏选择 (2WD/麦克纳姆/履带预留)。

## Key Gotchas

| # | What | Why |
|---|---|---|
| 1 | `ld14p_calibrate()` is **commented out** in `lidar_task.c` | LiDAR 0° PCB reversal not software-corrected yet |
| 2 | Flame GPIO 16 (center, angle 0°) is **not wired** | Pull-up keeps HIGH → always "no fire" |
| 3 | `CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM=n` (v6.0 改名自 `CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY`) | Task stacks stay in internal RAM (~4K words max safe) |
| 4 | Motor min duty 75 (`DRV8833_MIN_DUTY_PCT` in `drv8833_config.h`) | Overcomes static friction; duty→count linear map starts from `DRV8833_MIN_DUTY` |
| 5 | Web task reads `g_apf_rep`/`g_vfh_att` globals (not queue) | `apf_compute()` writes APF repulsion and VFH attraction separately to globals; web_task snapshots them for the WS `v_apf`/`v_vfh` fields. Combined cmd is computed client-side (`cmd = v_apf + v_vfh`). Motor control uses `q_cart` only |
| 6 | `temp_task.c` uses **ESP32-S3 internal temperature sensor**, not DS18B20 | Old DS18B20 code preserved in `#if 0` block; driver at `drivers/ds18b20.c` is dead code |
| 7 | Flame sensor init failure halts boot (`return` from `app_main`) | Logged as `ESP_LOGW` not `ESP_LOGE` — visually non-obvious but still fatal |
| 8 | `ws_handler` is **callback-based, no while(1)** | httpd uses select multiplexing; blocking loops corrupt session state and cause `WS frame is not properly masked` errors |
| 9 | `cfg.close_fn = ws_close_cb` in `httpd_config_t` | NOT `httpd_sess_set_close_fn` — that's a non-public API that causes compile errors |
| 10 | `cfg.send_wait_timeout = 2` (not `recv_wait_timeout`) | `recv_wait_timeout` field was removed from the codebase; ws_handler blocks on httpd select events |
| 11 | Manual mode uses `BIT_MANUAL_MODE` event bit | apf_task always computes APF (both modes) for web visualization; only writes `q_cart` in auto mode. ws_handler writes to `q_cart` only when bit is set |
| 12 | Joystick dy reversed when dx < 0 | `sendCmdNow()` in `script.js` negates dy for backward motion (APF steering convention) |
| 13 | Web UI uses unified color palette for 3 vectors | `CLR` object in `script.js`: red=APF repulsion, blue=VFH attraction, yellow=combined cmd (joystick in manual, rep+att in auto). Background rings: red=danger, green=safe, yellow=noise |
| 14 | Joystick sends at 8Hz (`JS_SEND_MS=125`) with 5% dead zone | Matches `MOTOR_FREQ_HZ`; CSS `touch-action:none` blocks mobile gestures; `visibilitychange`/`blur` sends stop on page hide |
| 15 | APF param update/reset guarded by `BIT_MANUAL_MODE` | `update_apf_params` and `reset_apf_params` only execute in manual mode; `get_apf_params` works in any mode |
| 16 | APF param update validates each field individually | Per-field range clamping prevents illegal values (e.g. `gain_rep_x` ∈ [0, 500], `rep_nx` ∈ (0.1, 5.0)); fields missing from JSON are skipped |
| 17 | Config panel opens on header click (manual mode only) | Clicking "TARS-Console" header text triggers `get_apf_params` and shows the panel; panel is hidden by default |

## PCB & Hardware Errata

| Issue | Workaround |
|---|---|
| LiDAR 0° reversed on PCB | Software correction deferred (`ld14p_calibrate` commented out) |
| Motor/Flame pin conflict on original layout | Current mapping in `apf_common.h:57-66` (motors: GPIO 5,6,7,15; flame: 11,12,13,14,16) |
| Pins 3,46 physically cut on MCU | Do not use |

## Dead Code

| File | Status |
|---|---|
| `libs/mlx90640-library.zip` | Unused — not imported anywhere |
| `drivers/ds18b20.c` | Replaced by ESP32-S3 internal temp sensor; driver still present but not compiled into active task |

## Git

- `Paper/` is `.gitignore`d — **not** committed. (`AGENTS.md` **is** tracked and committed.)
- `build/`, `managed_components/`, `sdkconfig`, `sdkconfig.old`, `dependencies.lock`, `.opencode/` are gitignored.
- `.vscode/{tasks,settings,extensions,launch}.json` are committed; `libs/` and `docs/` are **not** gitignored.
