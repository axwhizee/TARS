# LD14P 驱动库

## 概述

### LD14P 简介

LD14P 是 LDROBOT 出品的一款低成本 360° 单线激光雷达，采用三角测距原理，测距量程 0.1~8 m，逻辑电平 3.3V TTL。内置电机带动激光头旋转，通过 UART 输出 47 字节数据帧（每帧 12 个采样点），可下发命令调节扫描频率（2~8 Hz）。

⚠️ 实测本模组通信波特率为 **115200 bps**（官方手册标注 230400 bps，用 230400 会全部乱码）。

### 使用说明

- **供电**：DC 5V±10%，工作电流 ≤300 mA，启动电流 ≤1 A（须外部供电，勿依赖 USB）。
- **接线**（LD14P 4Pin JST）：
  | LD14P 引脚 | ESP32-S3 |
  |---|---|
  | 1 PWM/RX | GPIO17 (UTX) |
  | 2 GND | GND |
  | 3 TX | GPIO18 (URX) |
  | 4 VCC | 5V |
- **坐标系**：传感器正前方为 0°，顺时针递增（原生坐标系，不做 ROS 转换）。
- 若 PWM/RX 悬空，LD14P 电机可能不转，需接 UTX 或接地。

### 目录结构

```bash
drivers/ld14p/
├── internal/              # 核心层内部辅助，不对外暴露
│   └── ld14p_utils.h(.c)  # CRC8 查表与计算
├── port/                  # 移植层，唯一包含平台 HAL 的部分
│   ├── ld14p_port_def.h   # 移植目标硬件资源定义（串口/引脚/波特率）
│   ├── ld14p_port.h(.c)   # 移植时只需实现其中的接口
├── ld14p_config.h         # 驱动配置：RTOS 开关、协议编码公式、参数范围
├── ld14p_types.h          # 公共类型定义：错误码、句柄、帧/点结构
├── ld14p.h(.c)            # 核心层公共 API，调用者唯一入口
└── README.md
```

依赖顺序：调用者 → 核心层（`ld14p.h/.c`） → 移植层（`port/`） → 平台驱动。反向依赖被禁止。

## API 参考

### 数据类型

```c
typedef enum { LD14P_OK = 0U, LD14P_ERR_PARAM, LD14P_ERR_NOT_READY,
               LD14P_ERR_FRAME, LD14P_ERR_CRC, LD14P_ERR_TIMEOUT,
               LD14P_ERR_UART, LD14P_ERR_INIT } ld14p_err_t;

typedef struct { float dst; float ang; } ld14p_polar_t;   // 距离 mm，角度 °
typedef struct { uint16_t target_freq_hz; uint32_t timeout_ms; } ld14p_cfg_t;
typedef struct { ... } ld14p_handle_t;                    // 实例句柄，静态分配
```

### 函数列表

| 函数 | 职责 |
|------|------|
| `ld14p_init(h, cfg)` | 初始化：UART 安装 → 清点云 → 发送频率命令 |
| `ld14p_deinit(h)` | 反初始化，释放 UART 资源 |
| `ld14p_set_freq(h, freq_hz)` | 发送 0xA2 转速控制命令（2~8 Hz） |
| `ld14p_feed_byte(h, byte, out_frame)` | 字节级状态机：拼帧 + VerLen/CRC8 双校 |
| `ld14p_collect(h, frm, out_scan)` | 角度插值写入点云 + 跨零点圈检测 |
| `ld14p_process(h, out_scan)` | 轮询主入口：读 UART → feed → collect |
| `ld14p_calibrate(points, count, ox, oy)` | SlTransform 几何校准（in-place） |
| `ld14p_get_cloud(h)` | 获取当前累积点云指针 |

### 调用示例

```c
#include "drivers/ld14p/ld14p.h"

static ld14p_handle_t g_ld14p;

// 初始化
static const ld14p_cfg_t cfg = { .target_freq_hz = 4 };
if (ld14p_init(&g_ld14p, &cfg) != LD14P_OK) { /* 失败处理 */ }

// 任务主循环（一圈完成才返回 LD14P_OK）
const ld14p_polar_t *scan;
if (ld14p_process(&g_ld14p, &scan) == LD14P_OK) {
  /* 消费 scan[0..359]，scan 指向驱动句柄内点云 */
}
```

### 帧格式（47 字节，LE）

```
[0]     0x54        : 帧头
[1]     0x2C        : VerLen（帧类型=1，点数=12）
[2..3]  speed       : 转速 (°/s)
[4..5]  start_angle : 起始角度 (×0.01°)
[6..41] points[12]  : 每点 3B = [distLSB][distMSB][intensity]
[42..43] end_angle  : 结束角度 (×0.01°)
[44..45] timestamp  : 时间戳 (ms)
[46]    crc8       : 多项式 0x4D，init=0，无 XOR，覆盖 [0..45]
```

## 移植指南

移植到新平台只需完成以下工作：

1. **调整 `ld14p_config.h`**：按目标平台修改协议参数、频率范围，并按环境设置 `LD14P_RTOS_ACTIVE`（1=RTOS 版本 delay/tick，0=裸机）。
2. **实现 `port/ld14p_port.h` 声明的全部接口**（硬件资源与平台 HAL 定义放 `ld14p_port_def.h`，平台头文件只允许出现在 `ld14p_port.c`）：

   | 接口 | 说明 |
   |------|------|
   | `ld14p_port_uart_init(h)` | 安装/配置 UART，成功后填入 `h->bus` |
   | `ld14p_port_uart_deinit(h)` | 释放 UART 资源 |
   | `ld14p_port_write(h, data, len)` | 发送控制命令 |
   | `ld14p_port_read(h, buf, len, out_len, timeout_ms)` | 接收数据（0=非阻塞） |
   | `ld14p_port_delay_ms(ms)` | 毫秒延时（按 `LD14P_RTOS_ACTIVE` 分支） |
   | `ld14p_port_get_tick_ms()` | 单调毫秒时间戳（圈检测防抖） |

3. **注意**：核心层 `ld14p.c` 使用 `esp_log.h` 输出日志，移植时若目标平台无该库，替换为自定义日志宏即可。

## 故障排除

| 现象 | 排查方法 |
|------|---------|
| 无数据、无 0x54 帧头 | 波特率是否为 115200（而非 230400）；RX 引脚电压 ~1.65V；共地；5V 供电 ≥4.5V |
| 0xA2 命令后数据中断 | 先不发命令验证默认 6Hz；可能是波特率不匹配致命令被误解释 |
| 任务栈溢出 / PSRAM 缓存错误 | `cloud[360]`（2.9KB）在句柄内（.bss），句柄须静态分配，勿放任务栈；`CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM=n` |
| `uart_read_bytes` 带超时崩溃 | 移植层未安装事件队列时，读取必须用 timeout=0 非阻塞 + 轮询 |
| vTaskDelay 永不返回 | 需禁用 Tickless Idle（`CONFIG_FREERTOS_USE_TICKLESS_IDLE=n`） |
