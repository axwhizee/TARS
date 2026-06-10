# ESP32 TARS — PlatformIO + ESP-IDF

Target: **esp32-s3** (N16R8: 16MB flash, 8MB Octal PSRAM). Framework: **espidf** (not Arduino).
Project lives on **WSL** — `pio` must come from `/mnt/c/Users/Richa/WorkSpace/MCUs/.venv/Scripts:$PATH`.
For flash/monitor across WSL→Windows, use `pio.exe` with a 18s timeout.

## Build & Flash

```bash
export PATH="/mnt/c/Users/Richa/WorkSpace/MCUs/.venv/Scripts:$PATH"

pio run                          # build
pio run -t upload                # build + flash (921600 baud)
pio run -t monitor               # serial monitor (115200 baud)
pio run -t clean
pio run --target envdump         # inspect resolved build vars
pio run --target buildfs         # build SPIFFS image from data/
pio run --target uploadfs        # upload SPIFFS image to flash
pio run -t upload -t uploadfs -t monitor     # full: firmware + fs + monitor
```

WSL cross-call variant (wraps to avoid hang):
```
timeout 18 pio.exe run --target upload --target monitor 2>&1 || true
```

## Source Layout

| Path | Role |
|---|---|
| `src/main.c` | Entrypoint — NVS/SPIFFS/Wi-Fi init, hardware drivers, 4 queues + 1 event group, 7 tasks |
| `src/sys_init.c` | NVS flash → SPIFFS `/spiffs` → Wi-Fi AP (`TARS`, open, `192.168.10.1/24`, DHCP off) |
| `include/apf_common.h` | Global types, RTOS handle `extern`s, event-group bits, GPIO pins, APF/motor/WiFi constants |
| `src/drivers/` | `ld14p.c` (LiDAR UART), `drv8833.c` (DRV8833 PWM), `ds18b20.c` (**dead code**, see below) |
| `src/tasks/` | 7 FreeRTOS tasks (prio 1–8): `sysmon_task`, `lidar_task`, `flame_task`, `temp_task`, `apf_task` (APF 斥力 + VFH 方向选择, 双模式持续解算), `motor_task`, `web_task` (HTTP/WS + 手动遥控 + APF 参数调优) |
| `data/` | SPIFFS web UI — `index.html`, `script.js`, `style.css` (Canvas polar radar, WebSocket client) |
| `PCB/` | Altium project |

## Partition Table (`default_16MB.csv`)

```
nvs(16KB) → otadata(8KB) → phy_init(4KB) → factory(2MB) → ota_0(2MB) → ota_1(2MB) → spiffs(1MB)
```

## Conventions

- **All includes** use `""`. `src/CMakeLists.txt` sets `INCLUDE_DIRS` to `include/`, includes as `"apf_common.h"`.
- **Source discovery**: `src/CMakeLists.txt` does `FILE(GLOB_RECURSE ...)` — every `.c` under `src/` auto-included.
- **Required IDF components**: `driver`, `esp_event`, `esp_http_server`, `esp_netif`, `esp_timer`, `esp_wifi`, `json`, `nvs_flash`, `spiffs`.
- **FreeRTOS**: `xTaskCreate` stack size in **words**. `q_cart` uses `xQueueOverwrite` → depth=1.
- **Dual-core pinning**: Core 0 = WiFi + lwIP + `web_task` (network); Core 1 = all sensor/control tasks.
- **WebSocket** at `ws://192.168.10.1:80/ws`. HTTP file server at `http://192.168.10.1:80/`.
- **Outgoing JSON is hand-built** via `snprintf` in `web_task.c` (avoids cJSON heap fragmentation). **Incoming** WS messages use `cJSON_ParseWithLength` — small temporary allocations are acceptable for the 2-field JSON parse.
- **Static files preloaded** from SPIFFS to heap at startup (`preload_static_files()`), then served from RAM — no FILE\* contention at runtime.
- **WS protocol**: All upstream messages use `action` field for unified dispatch. Periodic telemetry (4Hz) carries `v_apf` (APF repulsion), `v_vfh` (VFH attraction), and `v_polars` (77 sensor points) — combined cmd is computed client-side as `v_apf + v_vfh`. Event responses (`mode_changed`, `params`) are sent on-demand. Param validation uses a data-driven `PARAM_FIELDS[]` table.
- **LIDAR_SECTORS = 72** (not 60) — 360° → 72 sectors downsampling with min-weighted average. `Q_POLAR_DEPTH = LIDAR_SECTORS + FLAME_SENSOR_COUNT = 77`.
- **Inter-revolution LiDAR EMA** (`lidar_task.c`): `smoothed[i] = α × current + (1-α) × previous`. Tunable via `LIDAR_EMA_ALPHA` (default 0.6). Dampens turning-induced scan distortion; `first_rev` flag copies first scan directly to avoid blending with uninitialized zeros.
- **Motor control**: Magnitude-angle decomposition in `diff_control()`. `atan2(dy,dx)` → direction; `cos(angle)` → speed, `sin(angle) × GAIN_BASE` → turn rate. At ≥ `MOTOR_SPIN_DEG` (75°) switches to in-place spin. Turn rate is decoupled from speed, preventing proportional-clamp saturation at high forward velocity.

## Key Gotchas

| # | What | Why |
|---|---|---|
| 1 | `ld14p_calibrate()` is **commented out** in `lidar_task.c:84` | LiDAR 0° PCB reversal not software-corrected yet |
| 2 | Flame GPIO 16 (center, angle 0°) is **not wired** | Pull-up keeps HIGH → always "no fire" |
| 3 | `CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=n` | Task stacks stay in internal RAM (~4K words max safe) |
| 4 | Motor min duty 75 (`MOTOR_MIN_DUTY`) | Overcomes static friction; duty→count linear map starts from `MOTOR_MIN_COUNT` |
| 5 | Web task reads `g_cart_rep`/`g_cart_att` globals (not queue) | `apf_compute()` writes APF repulsion and VFH attraction separately to globals; web_task snapshots them for the WS `v_apf`/`v_vfh` fields. Combined cmd is computed client-side (`cmd = v_apf + v_vfh`). Motor control uses `q_cart` only |
| 6 | `temp_task.c` uses **ESP32-S3 internal temperature sensor**, not DS18B20 | Old DS18B20 code preserved in `#if 0` block; driver at `src/drivers/ds18b20.c` is dead code |
| 7 | Flame sensor init failure halts boot (`return` from `app_main`) | Logged as `ESP_LOGW` not `ESP_LOGE` — visually non-obvious but still fatal |
| 8 | `ws_handler` is **callback-based, no while(1)** | httpd uses select multiplexing; blocking loops corrupt session state and cause `WS frame is not properly masked` errors |
| 9 | `cfg.close_fn = ws_close_cb` in `httpd_config_t` | NOT `httpd_sess_set_close_fn` — that's a non-public API that causes compile errors |
| 10 | `recv_wait_timeout` is **commented out** | WS handler blocks on httpd select events; setting a timeout triggers EAGAIN loops that corrupt WS state |
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
| Motor/Flame pin conflict on original layout | Current mapping in `apf_common.h:65-96` (motors: GPIO 5,6,7,15; flame: 11,12,13,14,16) |
| Pins 3,46 physically cut on MCU | Do not use |

## Dead Code

| File | Status |
|---|---|
| `lib/mlx90640-library.zip` | Unused — not imported anywhere |
| `lib/ldlidar_sl_sdk.zip` | Vendored LD14P SDK; project uses own `src/drivers/ld14p.c` |
| `src/drivers/ds18b20.c` | Replaced by ESP32-S3 internal temp sensor; driver still present but not compiled into active task |

## Git

- `AGENTS.md` and `Paper/` are `.gitignore`d — **not** committed.
- `.pio/`, `.vscode/c_cpp_properties.json`, `.vscode/launch.json`, `.vscode/ipch` are gitignored.
- `lib/` and `docs/` are **not** gitignored.
