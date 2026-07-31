# ESP32 Fire-Fighting Robot — RTOS Sensor Fusion & APF+VFH Navigation

基于 ESP32-S3 的智能消防巡检机器人嵌入式系统。多源异构传感器（LiDAR + 红外火焰 + 温度）通过 FreeRTOS 事件组同步融合，经人工势场法 (APF) 斥力 + 360° VFH 方向选择实时解算避障指令，并通过板载 Web 页面 + WebSocket 提供雷达可视化。

## 硬件平台

| 组件 | 型号 | 接口 |
|---|---|---|
| 主控 | ESP32-S3 (16MB Flash, 8MB Octal PSRAM) | — |
| 激光雷达 | LD14P | UART1 (TX:17, RX:18), 115200bps |
| 火焰传感器 | 5× IR 红外 | GPIO 11, 12, 13, 14, 16 (注意: 15 已被电机占用) |
| 温度传感器 | ESP32-S3 内置 (原 DS18B20 已弃用) | 内部温度传感器 API |
| 电机驱动 | DRV8833 ×2 | A: GPIO5/6, B: GPIO7/15, LEDC PWM 20kHz |
| 心跳 LED | 板载 | GPIO48 |
| 通信 | Wi-Fi AP (TARS, 无密码) | IP 192.168.10.1, HTTP Port 80 |

> 小车速度大致在150mm/250ms，即600mm/s，意味着如果以250ms为控制周期，结合激光雷达的误差问题，应当将约400mm距离的障碍物视作危险障碍

## 项目结构

```bash
ESP32_Template/
├── src/
│   ├── main.c                    # 入口 — 系统/驱动/队列/任务创建编排
│   ├── sys_init.c                # ESP32 系统级初始化 (NVS / SPIFFS / Wi-Fi AP)
│   ├── CMakeLists.txt            # 源文件自动发现 + 组件依赖
│   ├── drivers/
│   │   ├── ds18b20.c             # DS18B20 驱动 (死代码, 已被内部温度传感器替代)
│   │   ├── ld14p.c               # LD14P UART 协议帧解析 + CRC8 校验
│   │   └── drv8833.c             # DRV8833 LEDC PWM 输出 + 制动
│   └── tasks/
│       ├── web_task.c            # HTTP/WS 服务器 + JSON 推送 + 手动遥控
│       ├── lidar_task.c          # LiDAR 数据采集 + 360→72 降采样
│       ├── flame_task.c          # 5 路火焰检测 + 极坐标映射
│       ├── temp_task.c           # ESP32-S3 内置温度传感器 (4Hz)
│       ├── sysmon_task.c        # WS2812 CPU 负载指示 (1Hz) + 运行时统计
│       ├── apf_task.c            # 人工势场法避障解算 + 双模式持续解算
│       └── motor_task.c          # EMA 平滑 + 差速 PWM 控制
├── include/
│   ├── apf_common.h              # 全局类型/宏定义/RTOS 句柄 extern 声明
│   ├── sys_init.h                # 系统初始化接口 (NVS / SPIFFS / Wi-Fi AP)
│   ├── drivers/                  # 驱动头文件
│   └── tasks/                    # 任务头文件
├── data/
│   ├── index.html                # 板载雷达可视化网页 (Canvas 极坐标图)
│   ├── script.js                 # WebSocket 客户端 + Canvas 渲染逻辑
│   └── style.css                 # 深色主题样式
├── platformio.ini                # PlatformIO 构建配置
├── sdkconfig.defaults            # ESP-IDF SDK 覆盖配置
├── default_16MB.csv              # 分区表 (factory + OTA×2 + SPIFFS 1MB)
└── docs/                         # 数据手册与设计文档
```

## RTOS 任务表

| 任务名 | 优先级 | 栈 (words) | 周期 | 功能 |
|---|---|---|---|---|
| `sysmon` | 1 | 4096 | 1Hz | 心跳 LED + 运行时统计 |
| `web_task` | 3 | 8192 | 事件驱动 | HTTP/WS 服务器 + JSON 推送 + 手动遥控 |
| `apf` | 4 | 4096 | 4Hz | APF 斥力 + VFH 方向选择 (双模式持续解算, 自动模式下发送 q_cart) |
| `ld14p` | 5 | 8192 | 事件驱动 | UART 读帧, CRC8, 360→72 降采样 |
| `temp` | 6 | 4096 | 4Hz | ESP32-S3 内置温度传感器 |
| `flame` | 7 | 3072 | 4Hz | 5 路红外火焰状态检测 |
| `motor` | 8 | 8192 | 8Hz | EMA 平滑 + 差速分解 + PWM 输出 |

## 任务间通信对象

### 队列

| 队列 | 深度 | 元素类型 | 生产者 → 消费者 |
|---|---|---|---|
| `q_polar` | 77 | `vector_polar_t` | LiDAR(72) + Flame(5) → APF+VFH |
| `q_cart` | 1 | `vector_cart_t` | APF(自动) / ws_handler(手动) → Motor (xQueueOverwrite) |
| `q_temp` | 4 | `float` | temp_task → web_task |
| `q_log` | 77 | `vector_polar_t` | APF → web_task |

### 事件组 `eg_sync`

| 事件位 | 置位者 | 清除者 | 含义 |
|---|---|---|---|
| `BIT_LIDAR_Q_READY` | lidar_task | apf_task | q_polar 有新 LiDAR 数据 |
| `BIT_TEMP_Q_READY` | temp_task | web_task | q_temp 有新温度数据 |
| `BIT_FLAME_Q_READY` | flame_task | apf_task | q_polar 有新火焰数据 |
| `BIT_LOG_Q_READY` | apf_task | web_task | q_log 有新一帧日志数据 |
| `BIT_MANUAL_MODE` | web_task | web_task | 0=自动(APF+VFH), 1=手动(前端摇杆) |

## 系统四层框图

```mermaid
graph TB
    subgraph Sensing["感知层 (Sensing Layer)"]
        LIDAR["LD14P 激光雷达<br/>UART1, 115200bps<br/>360点/圈, 4Hz"]
        FLAME["5路火焰传感器<br/>GPIO 11-14,16<br/>前向120°, 4Hz"]
        TEMP["ESP32-S3 内置温度<br/>传感器, 4Hz"]
    end

    subgraph Fusion["融合决策层 (Fusion & Decision Layer)"]
        APF["APF 斥力 + VFH 方向选择<br/>360° 全向直方图扫描<br/>角度 EMA 平滑 + 宽度评分"]
        SYNC["事件组同步<br/>eg_sync: 双位原子性等待<br/>260ms 超时保护"]
    end

    subgraph Actuation["执行层 (Actuation Layer)"]
        EMA["EMA 低通平滑<br/>4Hz→8Hz 解耦"]
        DIFF["差速分解 + 动态转向"]
        PWM["DRV8833 双H桥<br/>LEDC PWM, 20kHz, 10-bit"]
    end

    subgraph Comm["通信层 (Communication Layer)"]
        AP["Wi-Fi AP<br/>SSID: TARS<br/>192.168.10.1:80"]
        WS["WebSocket /ws<br/>JSON 实时推送"]
        HTTP["HTTP 静态文件服务<br/>SPIFFS /spiffs"]
        BROWSER["浏览器端<br/>Canvas 雷达可视化"]
    end

    LIDAR -->|"q_polar (72点)"| APF
    FLAME -->|"q_polar (5点)"| APF
    TEMP -->|"q_temp (float)"| WS
    APF -->|"q_cart (cmd向量, 仅自动)"| EMA
    APF -->|"g_apf_rep/g_cart_att (斥力+引力)"| WS
    APF -->|"q_log (77点)"| WS
    SYNC -.-> APF
    EMA --> DIFF
    DIFF --> PWM
    PWM -->|"L/R PWM"| MOTOR["差速电机组"]
    WS -->|"JSON帧, ~1.9KB"| BROWSER
    HTTP -->|"HTML/CSS/JS"| BROWSER
    AP -.-> WS
    AP -.-> HTTP
```

## GPIO 连接示意图（不考虑 PCB）

```mermaid
graph LR
    MCU["ESP32-S3<br/>N16R8"]

    MCU -->|"UART1: GPIO17(TX)<br/>GPIO18(RX)<br/>飞线至焊盘3/43"| LD14P["LD14P<br/>LiDAR"]
    MCU -->|"GPIO: 11,12,13,14,16<br/>数字输入, 内部上拉"| FLAME["5ch Flame<br/>IR Sensor"]
    MCU -->|"GPIO9<br/>内部温度 (DS18B20 已弃用)"| TSENS["Internal<br/>Temp Sensor"]
    MCU -->|"LEDC: GPIO5,6(A)<br/>GPIO7,15(B)<br/>PWM 20kHz"| DRV8833["DRV8833<br/>Dual H-Bridge"]
    MCU -->|"GPIO48"| LED["On-board LED"]

    DRV8833 -->|"Motor A"| ML["Left Motors<br/>并联"]
    DRV8833 -->|"Motor B"| MR["Right Motors<br/>并联"]

    PWR5V["5V Power (USB)"] --> MCU
    PWR5V["5V Power (USB)"] --> DRV8833
    PWR5V --> LD14P
    MCU -->|"3V3"| FLAME
```

## 网页雷达可视化 + 手动遥控

ESP32 内置 Web 服务器，连接其 Wi-Fi (`TARS`) 后浏览器访问 `http://192.168.10.1`:

**雷达显示**:
- Canvas 极坐标渲染 — 零外部依赖
- 同心圆距离网格 (1m~6m, 危险区/安全区分色)
- 散点着色: 红色 (危险区), 绿色 (安全/感知区), 黄色 (噪声区)
- 实时温度显示 (绿 <30°, 黄 30-50°, 红 >50°) + 统计
- 三向量方向线: APF 斥力 (红色), VFH 引力 (蓝色), 合力指令 (黄色, 自动=red+blue, 手动=摇杆)

**手动遥控**: 顶部状态栏点击「自动」按钮切换手动模式，在雷达图上拖拽即可控制小车方向 (8Hz, 5% 死区)。松开后车停止，按钮切回自动模式恢复 APF+VFH 导航。移动端已禁用触摸手势 (touch-action:none)，切 tab 时自动停车。

网页通过 WebSocket (`ws://192.168.10.1:80/ws`) 接收 JSON 数据流并上行遥控指令。

## APF+VFH 参数在线调优

手动模式下，点击顶部品牌标签（"TARS-Console"）打开参数配置面板。面板分两栏：

**APF 斥力参数 (6项)**:

| 参数 | 默认值 | 范围 | 说明 |
|---|---|---|---|
| `range_rep` | 800 | (100, 4000) | 斥力归一化锚点 (mm)，r=r_ref 时力=K |
| `gain_rep_x` | 80 | [0, 500] | 斥力 X 增益（后退方向） |
| `gain_rep_y` | 100 | [0, 500] | 斥力 Y 增益（转向方向） |
| `rep_nx` | 1.6 | (0.1, 5.0) | X 衰减指数 (1/r^n)，越大近距制动越猛 |
| `rep_ny` | 1.0 | (0.1, 5.0) | Y 衰减指数 (1/r^n)，越小远距转向越灵敏 |
| `att_base` | 200 | [0, 2000] | VFH 引力基础增益 (× 通道宽度缩放) |

**VFH 直方图参数 (6项)**:

| 参数 | 默认值 | 范围 | 说明 |
|---|---|---|---|
| `vfh_thresh` | 1200 | [200, 5000] | 障碍判定阈值 (mm)，低于此视为不可通行 |
| `vfh_min_w` | 4 | [1, 20] | 最小有效通道宽度 (bins，每 bin 5°) |
| `vfh_smooth_w` | 1.5 | [0.1, 10.0] | 直方图平滑权重 (3点加权移动平均) |
| `vfh_free_th` | 0.25 | [0.01, 1.0] | 平滑后低于此值视为可通行 |
| `vfh_goal_bias` | 0.15 | [0, 1.0] | 正前方偏好 (0=无偏好, 1=强偏好) |
| `vfh_ema_alpha` | 0.8 | (0, 1.0] | 角度 EMA 平滑系数 (0=纯惯性, 1=无平滑) |

面板底部三个按钮：「应用」推送参数、「重置」恢复编译默认值、「退出」关闭面板。每次打开面板时会自动请求当前参数值。

> **注意**: 参数修改仅在手动模式下生效，且为**临时生效**——重启后恢复编译默认值（NVS 持久化尚未实现）。

## WebSocket 数据格式

### 周期帧 (ESP32 → 浏览器, 4Hz)

JSON 帧结构 (ESP32 端 snprintf 手写, ~1.9KB):

```json
{
  "ts": 1234567890,
  "temp": 25.50,
  "v_apf": {"dx": -120.0, "dy": 80.0},
  "v_vfh": {"dx": 800.0, "dy": 200.0},
  "v_polars": [
    {"a": 2.5, "d": 1234.5},
    ...
    {"a": 357.5, "d": 4567.8}
  ]
}
```

- `ts`: esp_timer_get_time() 微秒时间戳
- `temp`: 温度值 (°C)，无数据时为 `null`
- `v_apf`: APF 斥力笛卡尔向量 `{dx, dy}` (mm)
- `v_vfh`: VFH 引力笛卡尔向量 `{dx, dy}` (mm)
- `v_polars`: 77 个极坐标点 `{a: angle_deg, d: distance_mm}` (72 LiDAR + 5 Flame); 客户端按 `cmd = v_apf + v_vfh` 计算合力

### 事件帧 (ESP32 → 浏览器, 按需)

```json
// 模式切换确认
{"action":"mode_changed", "mode":"manual"}

// 参数查询/更新/重置的回复 (三种 action 共用同一格式)
{"action":"params", "params":{
  "range_rep":800.0, "gain_rep_x":80.0, "gain_rep_y":100.0,
  "rep_nx":1.6, "rep_ny":1.0, "att_base":200.0,
  "vfh_thresh":1200.0, "vfh_min_w":4, "vfh_smooth_w":1.5,
  "vfh_free_th":0.25, "vfh_goal_bias":0.15, "vfh_ema_alpha":0.8
}}
```

### 上行指令 (浏览器 → ESP32)

所有上行消息统一使用 `action` 字段分发:

```json
{"action":"set_mode", "mode":"manual"}       // 切换手动模式
{"action":"set_mode", "mode":"auto"}         // 恢复自动驾驶
{"action":"cmd", "dx":1234, "dy":-567}       // 摇杆控制指令 (8Hz, 仅手动模式生效)
{"action":"get_params"}                       // 请求当前 APF+VFH 参数 (任意模式)
{"action":"update_params", "params":{         // 推送参数 (仅手动模式生效)
  "range_rep":800, "gain_rep_x":80, "gain_rep_y":100,
  "rep_nx":1.6, "rep_ny":1.0, "att_base":200,
  "vfh_thresh":1200, "vfh_min_w":4, "vfh_smooth_w":1.5,
  "vfh_free_th":0.25, "vfh_goal_bias":0.15, "vfh_ema_alpha":0.8
}}
{"action":"reset_params"}                     // 重置为编译默认值 (仅手动模式生效)
```

## 构建与烧录

```bash
# 仅编译项目（生成.pio/build/<环境名>/目录）
pio run

# 编译并烧录固件
pio run -t upload

# 编译 data/ 目录内文件（SPIFFS）为分区镜像
pio run -t buildfs

# 编译分区镜像并上传
pio run -t uploadfs

# 串口监视，若固件缺失则打包
pio run -t monitor

# 清理
pio run -t clean

# 查看已解析的构建变量
pio run -t envdump
```

构建、烧录固件+网页、烧录一条龙（Build + Build File system Image + Upload Filesystem Image + Upload and Monitor）:

```
pio run -t upload -t uploadfs -t monitor
```

> 烧录波特率 921600，监视器波特率 115200（见 `platformio.ini`）。

## SDK 关键配置 (`sdkconfig.defaults`)

| 配置项 | 值 | 说明 |
|---|---|---|
| `CONFIG_SPIRAM_MODE_OCT` | y | 启用 Octal PSRAM（N16R8 必须） |
| `CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY` | n | 任务栈保留在内部 RAM，避免 PSRAM 缓存错误 |
| `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP` | y | WiFi/lwIP 堆分配移至 PSRAM，释放内部 RAM 给 TCP 连接 |
| `CONFIG_FREERTOS_USE_TICKLESS_IDLE` | n | 禁止 Tickless Idle（否则 `vTaskDelay` 永不返回） |
| `CONFIG_HTTPD_WS_SUPPORT` | y | 启用 HTTPD WebSocket 支持 |
| `CONFIG_MBEDTLS_CERTIFICATE_BUNDLE` | n | 禁用证书包（避免 x509_crt_bundle 编译错误） |
| `CONFIG_LWIP_MAX_SOCKETS` | 22 | 扩大 socket 上限 (httpd + 多设备 + 内部预留) |
| `CONFIG_LWIP_TCP_SND_BUF_DEFAULT` | 2920 | 缩减每连接 TCP 发送缓冲 (2×MSS，原 5760 太大) |
| `CONFIG_LWIP_TCP_WND_DEFAULT` | 2920 | 缩减每连接 TCP 窗口大小 |
| `CONFIG_LWIP_TCP_MSL` | 10000 | 缩短 TCP MSL (60s→10s，减少僵尸连接占用) |
| `CONFIG_LWIP_IP4_REASSEMBLY` | n | 关闭 IP 分片重组 (纯 HTTP/WS 不会有大包) |
| `CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0` | y | lwIP 协议栈固定在 Core 0（与 WiFi 同核，避免跨核缓存同步） |
| `CONFIG_FREERTOS_USE_TRACE_FACILITY` | y | 启用 FreeRTOS 运行时统计 (`vTaskGetRunTimeStats`) |

## 分区表 (`default_16MB.csv`)

| 分区名 | 大小 | 说明 |
|---|---|---|
| factory | 2MB | 出厂固件 |
| ota_0 / ota_1 | 各 2MB | OTA 双分区 |
| spiffs | 1MB | Web 页面存储 |

## 关键设计决策

- **APF + VFH 双层导航**: APF 斥力处理近距避障 (连续幂律, 无跳变); VFH 360° 全向直方图扫描选择最优通行方向, 引力方向由 VFH 动态决定 (不再固定前向)
- **APF 斥力/引力分离输出**: `apf_compute()` 分别输出斥力 `g_apf_rep` 和引力 `g_cart_att` 两个全局变量, Web 可视化和电机指令解耦 — 前端客户端按 `cmd = v_apf + v_vfh` 合成合力
- **VFH 角度 EMA 平滑**: α=0.8, 约 2 帧 (500ms) 收敛, 环绕安全处理 0°/360° 边界, 消除通道选择跳变
- **VFH 通道评分**: `score = width + GOAL_BIAS × width × cos(center)`, 宽度相近时偏好正前方, 360° 全向无锥区限制
- **双频率解耦**: APF+VFH 解算 4Hz, 电机控制 8Hz, 通过 EMA 平滑 消除帧间抖动
- **差速控制**: 幅值-角度分解。`atan2(dy,dx)` 确定方向; `cos(angle)` → 速度, `sin(angle) × GAIN_BASE` → 转向率。角度 ≥ `MOTOR_SPIN_DEG` (75°) 切换原地转向模式。转向率与线速度解耦, 避免高速前进时比例限幅导致转向速度减半
- **无锁同步**: FreeRTOS 事件组 + 队列完成所有任务间通信, 无共享内存竞争
- **极坐标统一**: LiDAR 72 扇区 + 火焰 5 虚拟点均以 `(angle_deg, distance_mm)` 极坐标表示, 算法输入为 77 维齐次向量
- **拆分斥力剖面**: x 分量使用 1/r² 控制后退时机, y 分量使用 1/r 使转向力分布更均匀, 避免近距爆发
- **日志无阻塞**: Web 任务仅等 BIT_LOG_Q_READY, 温度非阻塞取最新值, 确保雷达数据流不因低速传感器卡顿
- **APF 双模式持续解算**: 无论手动/自动模式, `apf_compute()` 每帧均运行, 为 Web 可视化提供实时斥力/引力数据; 手动模式下仅跳过 `q_cart` 的写操作, 确保模式切换时视觉反馈不中断
- **板载 Web 可视化**: Canvas 2D API 实现极坐标雷达图 + 三向量力显示 (rep=红, att=蓝, cmd=黄), 无任何 CDN 依赖, 13KB 文件通过 SPIFFS 部署在 ESP32 上
- **Wi-Fi AP 模式**: 开放热点, 无需外部路由器, 任何设备连接后即可打开网页查看雷达
- **手动遥控模式**: 通过 `BIT_MANUAL_MODE` 事件位协调 `apf_task` 仅写 `q_cart` 控制权 (APF 持续解算为 Web 可视化提供斥力/引力), `ws_handler` 接管 `q_cart` 手动指令, 前端 Canvas 摇杆实时直控
- **APF+VFH 参数在线调优**: WebSocket action 协议支持实时查询/更新/重置 12 项导航参数, 仅手动模式下可写, 单字段范围校验防止非法值 (参数暂存 RAM, NVS 持久化待实现)
