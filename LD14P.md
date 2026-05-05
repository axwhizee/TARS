# LD14P 激光雷达 ESP32-S3 驱动开发记录

> 基于实际调试经验整理，包含与官方手册的差异及踩坑记录

## 一、实际验证参数

| 参数 | 手册标注值 | 实测值 | 说明 |
|------|-----------|--------|------|
| **通信接口** | UART @ 230400bps, 8N1 | **UART @ 115200bps, 8N1** | ⚠️ 实际模组为115200 |
| **扫描频率** | 默认6Hz | 4Hz (可设) | 通过0xA2命令配置 |
| **数据帧格式** | 47字节/包, 12点/包 | 47字节/包, 12点/包 | 协议格式正确 |
| **测距量程** | 0.1~8m | — | 未验证 |
| **逻辑电平** | 3.3V TTL | 3.3V TTL | 正确 |
| **工作电压** | DC 5V±10% | 5V | 正确 |

## 二、ESP32-S3 硬件连接

```
┌─────────────┐          ┌─────────────────┐
│   LD14P     │          │   ESP32-S3      │
│  4PIN JST   │          │  (DevKitC-1)    │
├─────────────┤          ├─────────────────┤
│ 1 PWM/RX    │──────────│ GPIO17 (U1TXD)  │ ← 串口命令/控速
│ 2 GND       │──────────│ GND             │
│ 3 TX        │──────────│ GPIO18 (U1RXD)  │ ← 雷达数据接收
│ 4 VCC       │──────────│ 5V (外部电源)   │
└─────────────┘          └─────────────────┘
```

**关键注意事项：**
1. **VCC必须接5V**：工作电流≤300mA，启动电流≤1A
2. **TX/RX交叉连接**：LD14P TX → ESP32 RX (GPIO18),
   LD14P RX → ESP32 TX (GPIO17)
3. **共地**：必须连接GND
4. **ESP32-S3 推荐 UART1 引脚**：TX=GPIO17, RX=GPIO18（勿用GPIO16）

## 三、驱动代码结构

```
├── include/
│   ├── all_defs.h              ← 公共定义: vector_polar_t, UART/引脚宏, 扇区数
│   ├── drivers/ld14p.h         ← 驱动API: init, feed_byte, process_frame, get_cloud
│   └── tasks/lidar_task.h      ← 传感器任务声明 + lidar_sensor_params_t
└── src/
    ├── main.c                  ← app_main: ld14p_init(4) + 队列/事件组 + 任务创建
    ├── drivers/ld14p.c         ← 协议状态机, CRC8, 角度插值, 圈检测, 频率命令
    └── tasks/lidar_task.c      ← 传感器任务: UART轮询 → 喂协议 → 降采样 → 推送队列
```

### 数据流

```
TX(17)──→ LD14P PWM/RX (0xA2频率命令, 115200bps)
RX(18)──← LD14P TX (115200bps, 47字节数据包)
    │
    ▼
ld14p_sensor_task (prio 3, stack 8192 words)
    ├─ uart_read_bytes(timeout=0) → 轮询 drain UART ring buffer
    ├─ ld14p_feed_byte(byte) → 2态状态机 → ld14p_frame_t*
    ├─ ld14p_process_frame(frm) → 角度插值 → cloud_360[] → 圈检测
    ├─ ld14p_get_cloud(raw) → vector_polar_t[360] 快照
    ├─ lidar_process(raw, sectors) → 降采样 360→36 扇区 (1/d²加权)
    └─ xQueueSend(q_polar) ×36 + xEventGroupSetBits(BIT_LIDAR_READY)
                    │
            Queue[36] vector_polar_t + EventGroup
                    │
            APF 任务 (待实现)
                └─ 等待 BIT_LIDAR_READY → 消费 q_polar → 清标志位
```

### 公开 API

| 函数 | 签名 | 职责 |
|------|------|------|
| `ld14p_init(freq_hz)` | `esp_err_t → void` | 初始化UART1(115200, TX17/RX18), 重置cloud_360为0xFFFF, 发送0xA2频率命令 |
| `ld14p_feed_byte(byte)` | `uint8_t → const ld14p_frame_t*` | 字节级2态FSM: 搜0x54帧头→拼47字节→VerLen+CRC8双校, 返回帧指针或NULL |
| `ld14p_process_frame(frm)` | `const ld14p_frame_t* → bool` | 角度插值→覆盖cloud_360[]→检测完整一圈(防抖150ms), 返回true表示圈完成 |
| `ld14p_get_cloud(out)` | `vector_polar_t[360] → uint32_t` | 快照cloud_360[] → vector_polar_t[360], 返回有效点数(distance<60000) |

**设计原则**：
- `ld14p_frame_t` 是 `packed` 结构体，直接映射47字节线格式（ESP32 LE = LSB-first），无需手动字节索引
- `cloud_360[360]` 持久化，每圈逐点覆盖写入，不memset清空
- 圈检测：`start_angle` 从 >30000(>300°) 翻转到 <6000(<60°) 时触发，配合150ms防抖
- `ld14p_get_cloud` 不做数据筛选/填充，raw copy 原样输出，数据清洗交由后续统一处理管线

### 内部数据类型

```c
// 单个采样点 (3 字节, 直接映射线格式)
typedef struct __attribute__((packed)) {
    uint16_t distance;       // 距离值 (mm), LSB在前, 0xFFFF = 未填充
    uint8_t  intensity;      // 反射强度 (0~255)
} ld14p_point_t;

// 完整数据帧 (47 字节, 直接映射线格式)
typedef struct __attribute__((packed)) {
    uint8_t       header;          // [0]  0x54
    uint8_t       ver_len;         // [1]  0x2C
    uint16_t      speed;           // [2..3] 转速 (°/s)
    uint16_t      start_angle;     // [4..5] 起始角度 (×0.01°)
    ld14p_point_t points[12];      // [6..41] 12 个采样点
    uint16_t      end_angle;       // [42..43] 结束角度
    uint16_t      timestamp;       // [44..45] 时间戳 (ms)
    uint8_t       crc8;            // [46] CRC8
} ld14p_frame_t;

_Static_assert(sizeof(ld14p_frame_t) == 47, "Must be 47 bytes");
```

### vector_polar_t 结构

```c
typedef struct {
    float distance_mm;  // 距离(毫米), 65535.0 表示无效
    float angle_deg;    // 角度(度), 0°正前方, 顺时针递增
} vector_polar_t;
```

### 降采样算法 (360 → 36 扇区)

360个原始点按10°间隔划分为36个扇区（0°~9°, 10°~19°, ...），每扇区内用最小距离加权平均：

```
weight = 1 / distance²          // 距离越近权重越大, 障碍物信号被强化
sector_distance = Σ(weight × distance) / Σ(weight)
```

扇区输出角度为各扇区中点：5°, 15°, 25°, ..., 355°。若扇区内无有效点，输出 65535.0 (无效)。

### 任务间同步

| 机制 | 用途 |
|------|------|
| `xQueueCreate(36, sizeof(vector_polar_t))` | 36字节深度，存放降采样后的扇区数据 |
| `xEventGroupCreate()` | BIT_LIDAR_READY 标志位，每圈生产者置位、消费者清除 |
| Frame-drop 保护 | 生产者推送前检测 BIT_LIDAR_READY，若仍置位（上圈未被消费）则跳过本轮推送并打印警告 |

每圈推送流程：
1. 检查 `BIT_LIDAR_READY` → 若已置位 → `ESP_LOGW("Frame dropped")` → 跳过
2. `xQueueSend(q_polar, &sectors[i], 0)` × 36
3. `xEventGroupSetBits(BIT_LIDAR_READY)`

**现状**：消费者（APF任务）尚未实现，因此每圈仅首次推送成功，后续每圈均触发 "Frame dropped" 警告。这不影响数据质量，APF任务到位后即消失。

## 四、实测协议细节

### 数据包格式（47字节固定）

```
[0]   0x54        : 帧头（固定）
[1]   0x2C        : VerLen（帧类型=1, 点数=12）
[2-3]  speed      : 转速(°/s, LSB在前)
[4-5]  start_angle: 起始角度(0.01°单位, LSB在前)
[6-41] points[12] : 12个点, 每点3字节 = [distLSB][distMSB][intensity]
[42-43] end_angle : 结束角度(0.01°单位)
[44-45] timestamp : 时间戳(ms, 0~30000循环)
[46]   crc8       : CRC8(多项式0x4D), 覆盖字节0~45
```

### CRC8 实现

多项式 `x^8 + x^6 + x^3 + x^2 + 1` (0x4D)，Init=0x00，无最终XOR。使用256字节查表法。

```c
static const uint8_t CRC_TABLE[256] = { /* 见 ld14p.c */ };

static uint8_t crc8_calc(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++)
        crc = CRC_TABLE[(crc ^ data[i]) & 0xFF];
    return crc;
}
```

### 角度插值

每包12点，根据起始角度与结束角度步进插值，处理0°→360°跨越：

```c
int16_t diff = end_angle - start_angle;
if (diff < 0) diff += 360 * 100;               // 跨0°补偿, 100 = 0.01°分辨率
float step = (float)diff / (12 - 1);           // 12个点有11个间隔
for (int i = 0; i < 12; i++) {
    int deg = (start_angle + (int)(i * step)) / 100;  // 0.01° → 1°
    deg %= 360;
    cloud_360[deg] = points[i];                // 覆盖写入
}
```

### 坐标系

传感器原生坐标系：**正前方0°，顺时针递增**。本驱动直接使用此坐标系，不做ROS转换。

如需ROS坐标系（逆时针递增），转换公式：
```c
int ros_angle = (54000 - raw_angle) / 100 % 360;  // +54000避免负数
```

## 五、频率设置

### 串口命令（0xA2）

```c
uint16_t speed = freq_hz * 360;                // 例: 4Hz → 1440°/s
uint8_t cmd[8] = { 0x54, 0xA2, 0x04,          // 帧头+命令+数据长度4
                   (uint8_t)(speed & 0xFF), (uint8_t)(speed >> 8),
                   0x00, 0x00, 0x00 };
cmd[7] = crc8_calc(cmd, 7);                   // CRC覆盖前7字节
uart_write_bytes(UART_NUM_1, cmd, 8);
```

命令通过 TX(17) → LD14P PWM/RX 发送，波特率与数据接收相同(115200)。发送后等待200ms让电机响应。

### PWM控速（备选）

若使用PWM控速（不推荐），将PWM/RX接GND即进入内部6Hz控速模式。

## 六、UART驱动设计关键点

### 非阻塞读取

```
uart_driver_install(UART1, 4096, 0, 0, NULL, 0)
                                ^^^^  event_queue = NULL
```

**原因**：event_queue=NULL 时，带超时的 `uart_read_bytes` 在 `v5.5.3` 上会触发 `StoreProhibited @ 0x00000008` crash（内核尝试通知不存在的FreeRTOS对象）。

**解决方案**：
- `uart_read_bytes(..., timeout=0)` 非阻塞调用
- `vTaskDelay(2)` 让步（**不能用vTaskDelay(1)**：1-tick delay在Tickless模式下不可靠）

### 轮询策略

```c
while (1) {
    uint8_t buf[256];
    int len;
    while ((len = uart_read_bytes(UART1, buf, 256, 0)) > 0) {
        for (int i = 0; i < len; i++) {
            const ld14p_frame_t *frm = ld14p_feed_byte(buf[i]);
            if (frm && ld14p_process_frame(frm)) { /* 圈完成 */ }
        }
    }
    vTaskDelay(2);  // 让步, 让UART ISR填充ring buffer
}
```

4Hz下数据流入速率约 47×30×4 ≈ 5640 字节/秒，256字节批处理 + vTaskDelay(2) 可稳定跟上（`rx_buf` 始终为0）。

### 帧间间隙

LD14P相邻帧的 `end_angle` 与下一帧 `start_angle` 之间约1~2°物理间隙（帧间不连续），导致360°点云中有1~2个角度保持 `0xFFFF`（无效值）。这是传感器固有行为，非驱动bug。数据清洗/插值交由后续统一处理管线完成。

## 七、与官方手册的差异

| 项目 | 官方手册 | 实际情况 | 影响 |
|------|---------|---------|------|
| **UART波特率** | 230400bps | **115200bps** | 用230400导致数据全为乱码，无法找到帧头0x54 |
| **UART1 RX引脚** | 未指定 | 推荐**GPIO18** | GPIO16在ESP32-S3上可能异常 |
| **PWM/RX引脚** | 可悬空 | 需拉低或接TX | 悬空时LD14P电机可能不转 |
| **sdkconfig** | 无 | **必须禁用Tickless Idle** 和 **禁止PSRAM栈** | 否则vTaskDelay永久阻塞 / 栈访问崩溃 |
| **0xA2频率命令** | 支持 | **波特率必须匹配** (115200) | 不匹配时命令被误解释，雷达停止发数据 |

## 八、已知问题与排查方法

### 8.1 波特率不匹配（最常见的坑）

**现象**：收到数据但无`0x54`帧头（RAW HEX非`54 2C`开头），CRC全部失败。

**排查**：dump前47字节看协议格式。
```c
uint8_t buf[47];
uart_read_bytes(UART1, buf, 47, 0);
ESP_LOGI(TAG, "RAW: %02x %02x %02x ...", buf[0], buf[1], buf[2]);
```

**解决**：尝试115200和230400两种波特率。

### 8.2 系统崩溃 - PSRAM缓存错误

**现象**：`Guru Meditation Error: Core 0 panic'ed (Cache error)`，
错误地址 `0x3c02xxxx`（PSRAM区域）。

**原因**：`CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y` 导致任务栈在PSRAM，
大局部变量触发了缓存错误。

**解决**：在 `sdkconfig.defaults` 中添加：
```
CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=n
```
然后删除已生成的 `sdkconfig.esp32-s3-devkitc-1`，重新编译。

### 8.3 vTaskDelay 永不返回

**现象**：任务在`vTaskDelay(1)`后永久阻塞，心跳日志消失。

**原因**：`CONFIG_FREERTOS_USE_TICKLESS_IDLE=y`（默认开启）。
系统空闲时进入休眠，tick中断停止，vTaskDelay的1 tick永远等不到。

**解决**：在 `sdkconfig.defaults` 中添加：
```
CONFIG_FREERTOS_USE_TICKLESS_IDLE=n
```

**最低延迟**：即使禁用Tickless，vTaskDelay(1) 在ESP-IDF v5.5.3上也不可靠。
本驱动使用 `vTaskDelay(2)` 保障可靠让步。

### 8.4 栈溢出

**现象**：`Guru Meditation Error (LoadProhibited)`，寄存器中出现 `0xa5a5a5a5`（FreeRTOS栈标记）。
多核时可能报在Core 1而非Core 0。

**原因**：局部大数组 + 函数调用链超出栈空间。

**解决**：传感器任务栈设 **8192 words**（= 32768 字节），预留充足空间。

### 8.5 uart_read_bytes 带超时崩溃

**现象**：`StoreProhibited` at address `0x00000008`。

**原因**：`uart_driver_install(port, rx_buf, 0, 0, NULL, 0)` 中事件队列为NULL，
调用 `uart_read_bytes()` 带 `timeout>0` 时内部试图访问不存在的通知对象。

**解决**：
- 使用 `uart_read_bytes(..., 0)`（非阻塞），配合 `vTaskDelay` 让步
- 或安装时传入有效事件队列句柄（暂未采用，避免额外RAM开销）

### 8.6 无UART数据

**现象**：`total_bytes` 持续为0，无任何RAW HEX输出。

**排查步骤**：
1. 测量RX引脚直流电压（3.3V信号应为~1.65V）
2. 断开LD14P TX线看数据是否消失（区分LD14P信号和串扰）
3. 确认PWM/RX引脚已正确连接（接地=6Hz内部控速，接TX=串口命令控速）
4. 确认5V供电正常（VCC≥4.5V）
5. 检查波特率是否正确

### 8.7 频率命令导致数据中断

**现象**：发送0xA2命令后LD14P停止发送数据。

**排查**：先不发送命令（注释掉 `ld14p_init` 内部的 `send_freq_command` 调用），
看默认6Hz数据是否正常。如果默认6Hz正常、发命令后异常，可能是波特率不匹配导致
命令内容被LD14P误解释。

**确认**：115200波特率下0xA2命令正常工作（本驱动已验证）。

### 8.8 "Frame dropped" 警告 (每圈)

**现象**：每圈打印一次 `ESP_LOGW("Frame dropped: previous data unread")`。

**原因**：消费者任务（APF）尚未实现，`BIT_LIDAR_READY` 标志位在生产后从未被清除。
首圈推送成功，后续每圈检测到标志位仍置位，跳过推送并打印警告。

**影响**：不影响首圈数据质量。队列中保留的是首圈降采样结果。APF消费者就位后正常。

### 8.9 点云valid固定缺1~2度

**现象**：`valid` 计数显示359/360或358/360有效。

**原因**：LD14P相邻帧的 `end_angle` 与下一帧 `start_angle` 之间约1~2°物理间隙。
例如：帧A末点落在~4°，帧B首点从~6°开始，则第5°恒为0xFFFF。

**处理策略**：本驱动不在 `get_cloud` 层面填充。空值由后续统一数据清洗管线处理，
各传感器数据将在同一阶段进行插值/筛选/融合。

## 九、SDK配置 (sdkconfig.defaults)

```
CONFIG_FREERTOS_USE_TICKLESS_IDLE=n          # 必须: 确保vTaskDelay正常返回
CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=n  # 必须: 防止PSRAM栈缓存错误
```

其他使用默认值：
- 16MB flash (CONFIG_ESPTOOLPY_FLASHSIZE_16MB)
- Octal PSRAM 80MHz (CONFIG_SPIRAM_MODE_OCT, CONFIG_SPIRAM_SPEED_80M)
- mbedTLS cert bundle disabled

## 十、编译与运行

```bash
pio run                          # 构建
pio run -t upload                # 烧录
pio run -t monitor               # 串口监视 (115200)
pio run -t upload && pio run -t monitor  # 烧录 + 监视
```

目标: ESP32-S3-DevKitC-1, framework: ESP-IDF v5.5.3

## 十一、参考资源

1. **协议参考**：https://jishuzhan.net/article/2033076476967452674
2. **官方SDK**：https://github.com/ldrobotSensorTeam/ldlidar_sl_sdk
3. **CRC表生成器**：`ld14p.c` 中的256字节表（多项式0x4D）
