# DRV8833 电机驱动库

## 概述

### DRV8833 简介

DRV8833 是 TI 出品的一款双通道 H 桥电机驱动器，支持 2.7V~10.8V 供电，单通道 1.5A RMS / 2A 峰值。本驱动库将其抽象为**底盘驱动**：不仅封装 IN/IN + PWM 的底层输出，还内置**底盘运动学算法**（速度指令 → 各轮转速），并预留多种底盘类型的扩展接口。

### 使用说明

- **底盘类型**：本项目为 **4WD 差速**（同侧电机对置并联，`DRV8833_CHASSIS_DIFF_4WD`）。
- **PWM 配置**：20kHz / 10-bit（0~1023），`LEDC_LOW_SPEED_MODE` + APB 80MHz 时钟。
- **最小有效占空比**：75%（克服静摩擦），`[1%,100%]` 线性映射到 `[MIN_DUTY, MAX_DUTY]`。
- **坐标系**：`dx`=前向分量，`dy`=侧向分量，单位 mm，归一化上限 `vel_max_mm`。
- **一阶低通滤波 (LPF)**：`y[n] = α·x[n] + (1-α)·y[n-1]`，可编译期/运行期启停。

### 目录结构

```bash
drivers/drv8833/
├── internal/                # 底盘算法（核心层内部，不对外暴露）
│   ├── drv8833_chassis.h    # 底盘控制接口 + 注册表声明
│   ├── drv8833_chassis.c    # 底盘类型 → 操作集分发
│   └── drv8833_chassis_diff4wd.c  # 四轮差速算法（当前唯一实现）
├── port/                    # 移植层，唯一包含平台 HAL 的部分
│   ├── drv8833_port_def.h   # 移植目标硬件资源定义（引脚/LEDC 通道）
│   ├── drv8833_port.h(.c)   # 移植时只需实现其中的接口
├── drv8833_config.h         # 驱动配置：底盘选择、LPF、PWM/控制参数
├── drv8833_types.h          # 公共类型定义：错误码、底盘枚举、句柄
├── drv8833.h(.c)            # 核心层公共 API，调用者唯一入口
└── README.md
```

依赖顺序：调用者 → 核心层（`drv8833.h/.c`） → 移植层（`port/`） → 平台驱动。反向依赖被禁止。

## API 参考

### 数据类型

```c
typedef enum { DRV8833_OK = 0U, DRV8833_ERR_PARAM,
               DRV8833_ERR_INIT, DRV8833_ERR_NOT_SUPPORTED } drv8833_err_t;

typedef enum { DRV8833_CHASSIS_DIFF_4WD = 0, DRV8833_CHASSIS_DIFF_2WD,
               DRV8833_CHASSIS_MECANUM, DRV8833_CHASSIS_TRACK } drv8833_chassis_type_t;

typedef struct { float dx; float dy; } drv8833_vel_t;   // 目标速度 (mm)
typedef struct { drv8833_chassis_type_t chassis; int8_t lpf_enabled;
                 float lpf_alpha; uint32_t timeout_ms; float vel_max_mm; } drv8833_cfg_t;
typedef struct { ... } drv8833_handle_t;                // 实例句柄，静态分配
```

### 函数列表

| 函数 | 职责 |
|------|------|
| `drv8833_init(h, cfg)` | 初始化：LEDC 定时器/通道 → 滑行（底盘类型校验） |
| `drv8833_deinit(h)` | 反初始化：PWM 全 0 + 释放资源 |
| `drv8833_set_speed(h, left, right)` | 底层百分比直控 [-100,100]，含静摩擦补偿 |
| `drv8833_brake(h)` | 制动：两路 IN 全 HIGH（绕组短接） |
| `drv8833_coast(h)` | 滑行：全部通道占空比 0（Hi-Z） |
| `drv8833_set_velocity(h, dx, dy)` | 设置目标速度 (mm) |
| `drv8833_update(h)` | 周期控制入口：超时归零 → LPF → 底盘算法 → PWM |
| `drv8833_set_lpf(h, enable)` | 运行时开关一阶低通滤波 |

### 调用示例

```c
#include "drivers/drv8833/drv8833.h"

static drv8833_handle_t g_motor;

// 初始化（默认底盘，速度归一化上限对齐 APF_RANGE_MAX）
static const drv8833_cfg_t cfg = { .vel_max_mm = APF_RANGE_MAX };
if (drv8833_init(&g_motor, &cfg) != DRV8833_OK) { /* 失败处理 */ }

// 控制任务循环（8Hz）
while (1) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(125));
    if (xQueueReceive(q_cart, &cmd, 0) == pdTRUE) {
        drv8833_set_velocity(&g_motor, cmd.dx, cmd.dy);
    }
    drv8833_update(&g_motor);   // 无新指令时超时归零 + 滤波收敛
}
```

## 底盘扩展指南

新增底盘类型（2WD / 麦克纳姆 / 履带）的步骤：

1. `drv8833_types.h`：在 `drv8833_chassis_type_t` 追加枚举值。
2. 新建 `internal/drv8833_chassis_xxx.c`，实现 `control(h, vel)`（滤波后速度 → 各轮转速，内部调用 `drv8833_set_speed`）。
3. `internal/drv8833_chassis.c`：在 `drv8833_chassis_ops()` 注册该类型的操作集。
4. `drv8833_config.h`：将 `DRV8833_CHASSIS_SELECT` 指向新类型。

> 当前只有 `DRV8833_CHASSIS_DIFF_4WD` 实现；其他类型 `init()`/`update()` 返回 `DRV8833_ERR_NOT_SUPPORTED`。

## 移植指南

移植到新平台只需完成以下工作：

1. **调整 `drv8833_config.h`**：按目标平台修改 PWM 频率/分辨率/底盘选择，并按环境设置 `DRV8833_RTOS_ACTIVE`（1=RTOS 版本 delay/tick，0=裸机）。
2. **实现 `port/drv8833_port.h` 声明的全部接口**（硬件资源与平台 HAL 定义放 `drv8833_port_def.h`，平台头文件只允许出现在 `drv8833_port.c`）：

   | 接口 | 说明 |
   |------|------|
   | `drv8833_port_pwm_init(h)` | 配置定时器 + 通道 + 引脚，成功后填入 `h->bus` |
   | `drv8833_port_pwm_deinit(h)` | 释放 PWM 资源 |
   | `drv8833_port_set_duty(h, ch, duty)` | 设置单个逻辑通道占空比 [0, MAX_DUTY] |
   | `drv8833_port_delay_ms(ms)` | 毫秒延时（按 `DRV8833_RTOS_ACTIVE` 分支） |
   | `drv8833_port_get_tick_ms()` | 单调毫秒时间戳（指令超时检测） |

3. **注意**：核心层 `drv8833.c` 使用 `esp_log.h` 输出日志，移植时若目标平台无该库，替换为自定义日志宏即可。

## 故障排除

| 现象 | 排查方法 |
|------|---------|
| 电机不转 | 检查 VM/STBY 供电（STBY 须接 3.3V）；确认 `drv8833_init` 返回 `DRV8833_OK` |
| 低于 75% 占空比不动 | 静摩擦下限 `DRV8833_MIN_DUTY_PCT` 需实测校准（文档实测约 72%） |
| 前进时某侧反转 | 相序问题，检查该侧"错位并联"接线或交换驱动输出（勿改代码） |
| `update()` 返回 NOT_SUPPORTED | `DRV8833_CHASSIS_SELECT` 指向了未实现的底盘类型 |
| 换向瞬间电流冲击 | 上层换向（正↔反）前调用 `drv8833_brake()` 或 `drv8833_coast()` 插入死区 |
| 电机启动时 ESP32 重启 | VM 两端并联大电容（470µF 电解 + 0.1µF 陶瓷）去耦 |
