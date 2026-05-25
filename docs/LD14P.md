# LD14P 激光雷达 ESP32-S3 驱动开发记录

> 基于实际调试经验整理，包含与官方手册的差异及踩坑记录

## 一、实际验证参数

| 参数 | 手册标注值 | 实测值 | 说明 |
|------|-----------|--------|------|
| **通信接口** | UART @ 230400bps, 8N1 | **UART @ 115200bps, 8N1** | ⚠️ 实际模组为115200 |
| **扫描频率** | 默认6Hz | 6Hz | 面板数据正确 |
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
│   ├── drivers/ld14p.h        ← 驱动API: init, feed_byte, scan_ready, get_scan
│   └── tasks/
│       ├── lidar_task.h       ← 传感器任务声明
│       └── logger.h           ← 通用日志任务（与LD14解耦）
└── src/
    ├── main.c                 ← ld14p_init(4) + xQueueCreate(360) + 任务创建
    ├── drivers/ld14p.c        ← 协议状态机, CRC8, 圈检测, 频率命令
    └── tasks/
        ├── lidar_task.c       ← 传感器任务: 连续读UART→状态机→推送队列
        └── logger.c           ← 日志任务: 每2秒取360点→UART0上传
```

### 数据流

```
TX(17)──→ LD14P PWM/RX (频率命令)
RX(18)──← LD14P TX (115200, 47字节数据包)
    │
    ▼
ld14p_sensor_task (prio 5, stack 8192 words)
    ├─ uart_read_bytes → ld14p_feed_byte → 状态机 → cloud_360[]
    └─ ld14p_scan_ready → ld14p_get_scan
         → xQueueReset(queue) → xQueueSend ×360
                                 │
                          Queue[360] vector_polar_t
                                 │
logger_task (prio 4, 每2秒)
    └─ xQueueReceive ×360 → UART0: [0xAA][len][360×vector_polar_t]
```

### 各函数职责

| 函数 | 职责 |
|------|------|
| `ld14p_init(freq_hz)` | 初始化UART1(115200), 重置cloud_360, 发送0xA2频率命令 |
| `ld14p_feed_byte(byte)` | 字节级状态机: 搜帧头0x54→拼47字节→CRC→解析12点→插值→更新cloud_360[] |
| `ld14p_scan_ready()` | 检测是否完成一整圈扫描（角度从>30000翻转到<6000时置位），**每次调用消耗该标志** |
| `ld14p_get_scan(out, count)` | 加锁拷贝cloud_360[]快照到用户缓冲区（角度0~359, 距离mm） |

### vector_polar_t 结构

```c
typedef struct {
    float distance_mm;  // 距离(毫米)
    float angle_deg;    // 角度(度), 0°正前方, 顺时针递增
} vector_polar_t;
```

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

多项式 `x^8 + x^6 + x^3 + x^2 + 1` (0x4D)，Init=0x00，无最终XOR。

```c
static const uint8_t CRC_TABLE[256] = { /* 256字节查表, 见ld14p.c */ };

static uint8_t ld14p_crc8(uint8_t *data, uint8_t len) {
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
if (diff < 0) diff += 36000;           // 36000 = 360° × 100
float step = (float)diff / 11.0f;      // 12个点有11个间隔
for (int i = 0; i < 12; i++) {
    int raw = start_angle + (int)(i * step);
    int angle_deg = (raw / 100) % 360;  // 0.01° → 1°, 取模360
    cloud_360[angle_deg] = points[i];
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
uint16_t speed = freq_hz * 360;           // 例: 4Hz → 1440°/s
uint8_t cmd[8] = { 0x54, 0xA2, 0x04,      // 帧头+命令+长度
                   (uint8_t)(speed & 0xFF), (uint8_t)(speed >> 8),
                   0x00, 0x00, 0x00 };
cmd[7] = ld14p_crc8(cmd, 7);              // CRC覆盖前7字节
uart_write_bytes(UART_NUM_1, cmd, 8);
```

命令通过 TX(17) → LD14P PWM/RX 发送，波特率必须与数据接收相同(115200)。

### PWM控速（备选）

若使用PWM控速（不推荐），将PWM/RX接GND即进入内部6Hz控速模式。

## 六、与官方手册的差异

| 项目 | 官方手册 | 实际情况 | 影响 |
|------|---------|---------|------|
| **UART波特率** | 230400bps | **115200bps** | 用230400导致数据全为乱码，无法找到帧头0x54 |
| **UART1 RX引脚** | 未指定 | 推荐**GPIO18** | GPIO16在ESP32-S3上可能异常 |
| **PWM/RX引脚** | 可悬空 | 需要拉低或接TX | 悬空时LD14P电机可能不转 |
| **sdkconfig** | 无 | 需禁用Tickless Idle和PSRAM栈 | 否则vTaskDelay和栈访问崩溃 |

## 七、已知问题与排查方法

### 7.1 波特率不匹配（最常见的坑）

**现象**：收到数据但无`0x54`帧头（RAW HEX非`54 2C`开头），CRC全部失败。

**排查**：dump前47字节看协议格式。在传感器任务中：
```c
// 收到前47字节后打印
ESP_LOGI(TAG, "RAW: %02x %02x %02x ...", buf[0], buf[1], buf[2], ...);
```

**解决**：尝试115200和230400两种波特率。

### 7.2 系统崩溃 - PSRAM缓存错误

**现象**：`Guru Meditation Error: Core 0 panic'ed (Cache error)`，
错误地址 `0x3c02xxxx`（PSRAM区域）。

**原因**：`CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y` 导致任务栈在PSRAM，
大局部变量（`scan_buf[360]` = 2880字节）触发了缓存错误。

**解决**：在 `sdkconfig.defaults` 中添加：
```
CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=n
```
然后删除已生成的 `sdkconfig.esp32-s3-devkitc-1`，重新编译。

### 7.3 vTaskDelay 永不返回

**现象**：任务在`vTaskDelay(1)`后永久阻塞，心跳日志消失。

**原因**：`CONFIG_FREERTOS_USE_TICKLESS_IDLE=y`（默认开启）。
系统空闲时进入休眠，tick中断停止，vTaskDelay的1 tick永远等不到。

**解决**：在 `sdkconfig.defaults` 中添加：
```
CONFIG_FREERTOS_USE_TICKLESS_IDLE=n
```

**备用方案**：用 `uart_read_bytes(..., 10ms超时)` 替代 `vTaskDelay`，
利用UART驱动的任务通知机制来阻塞（需确保UART安装时传入事件队列）。

### 7.4 栈溢出

**现象**：`Guru Meditation Error (LoadProhibited)`，寄存器中出现 `0xa5a5a5a5`（FreeRTOS栈标记）。
多核时可能报在Core 1而非Core 0。

**原因**：`scan_buf[360]` = 2880字节，加上函数调用链，超出栈空间。

**解决**：
```c
// main.c: 增大传感器任务栈
xTaskCreate(ld14p_sensor_task, "ld14p_sensor", 8192, /* 4096→8192 words */ ...);
```

### 7.5 uart_read_bytes 带超时崩溃

**现象**：`StoreProhibited` at address `0x00000008`。

**原因**：`uart_driver_install(port, rx_buf, 0, 0, NULL, 0)` 中事件队列为NULL，
调用 `uart_read_bytes()` 带 `timeout>0` 时内部试图访问不存在的通知对象。

**解决**：
- 使用 `uart_read_bytes(..., 0)`（非阻塞），配合 `vTaskDelay` 让步
- 或安装时传入有效事件队列句柄

### 7.6 串口TX缓冲溢出

**现象**：`E uart: uart_write_bytes(1629): uart driver error`。

**原因**：logger任务单次写入2883字节到UART0，但TX缓冲不足。

**解决**：不致命，数据仍能传输；或增大UART0 TX缓冲大小。

### 7.7 无UART数据

**现象**：`total_bytes` 持续为0，无任何RAW HEX输出。

**排查步骤**：
1. 测量RX引脚直流电压（3.3V信号应为~1.65V）
2. 断开LD14P TX线看数据是否消失（区分LD14P信号和串扰）
3. 确认PWM/RX引脚已正确连接（接地=6Hz内部控速，接TX=串口命令控速）
4. 确认5V供电正常（VCC≥4.5V）
5. 检查波特率是否正确

### 7.8 频率命令导致数据中断

**现象**：发送0xA2命令后LD14P停止发送数据。

**排查**：先不发送命令（注释掉 `ld14p_set_freq` 调用），看默认6Hz数据是否正常。
如果默认6Hz正常、发命令后异常，可能是波特率不匹配导致命令内容被LD14P误解释。

---

## 八、参考资源

1. **协议参考**：https://jishuzhan.net/article/2033076476967452674
2. **官方SDK**：https://github.com/ldrobotSensorTeam/ldlidar_sl_sdk
3. **CRC表生成器**：`ld14p.c` 中的256字节表（多项式0x4D）
