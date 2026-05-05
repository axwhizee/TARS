/*
 * ld14p.c — LD14P 激光雷达底层驱动
 *
 * 包含:
 *   - UART1 初始化 (115200-8N1, TX=GPIO17, RX=GPIO18)
 *   - 2 状态字节级协议解析器 (搜帧头 → 拼包 → 解析)
 *   - CRC8 校验 (多项式 0x4D, 256 字节查表)
 *   - 360° 圈检测 (角度翻转法)
 *   - 角度插值 (start_angle → end_angle, 12 点均匀分布)
 *   - 频率控制命令 (0xA2)
 *
 * 数据流:
 *   UART1 RX ISR → 环形缓冲区 (4096 字节)
 *        ↓ uart_read_bytes(timeout=0)
 *   ld14p_feed_byte() 逐字节
 *        ↓ 2 状态 FSM
 *   拼成 47 字节包 → ld14p_update_cloud()
 *        ↓ CRC8 校验
 *        ↓ 12 点 × 3 字节解析 + 角度插值
 *   cloud_360[0..359] 持续更新 (持久化, 不清空)
 *        ↓ start_angle 翻转检测
 *   revolution_flag = true → ld14p_scan_ready() 消费 → ld14p_get_scan() 快照
 */

#include "drivers/ld14p.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "LD14P";

/* ---------- 协议常量 ---------- */
#define LD14P_HEADER         0x54   /* 固定帧头: 0101 0100 */
#define LD14P_PACKET_LEN     47     /* 固定包长: 1(H)+1(VerLen)+2(转速)+2(始角)+36(12点×3)+2(终角)+2(时间戳)+1(CRC) */
#define LD14P_POINT_PER_PACK 12     /* 每包固定 12 个采样点 */
#define LD14P_ANGLE_RES      100    /* 角度分辨率: 1° = 100 LS B, 即 0.01° 单位 */
#define LD14P_CMD_SPEED      0xA2   /* 频率控制命令码 */
#define LD14P_CMD_LEN        4      /* 命令数据长度 */

/* ---------- CRC8 查表 (多项式 x^8 + x^6 + x^3 + x^2 + 1 = 0x4D) ---------- */
static const uint8_t CRC_TABLE[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
    0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
    0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
    0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
    0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
    0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
    0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
    0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
    0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
    0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
    0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
    0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
    0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
    0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
    0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
    0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
    0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
    0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
    0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
    0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
    0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
    0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

static uint8_t ld14p_crc8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++)
        crc = CRC_TABLE[(crc ^ data[i]) & 0xFF];
    return crc;
}

/* ---------- 内部数据结构 ---------- */

typedef struct {
    uint16_t distance;   /* 距离 (毫米), 原始值 0~8000, 0xFFFF = 无效 */
    uint8_t  intensity;  /* 反射强度 (0~255) */
} ld14p_point_t;

/* cloud_360[360]: 360° 点云持久化缓冲区.
 *   索引即角度 (0° ~ 359°), 每圈数据覆盖刷新.
 *   不清空, 只在 init 时一次性 memset(0xFF) 标记未填充.
 *   distance=0xFFFF → 无效点, <60000 → 有效测量. */
static ld14p_point_t cloud_360[LD14P_POINTS_PER_REV];

static uint16_t prev_start_angle = 0;       /* 上一个数据包的起始角度 (0.01° 单位) */
static volatile bool revolution_flag = false; /* 圈完成标志, sensor 任务消费 */

static uint32_t crc_ok_count   = 0;   /* 本圈内 CRC 成功的包数 */
static uint32_t crc_fail_count = 0;   /* 本圈内 CRC 失败的包数 */
static uint32_t rev_last_tick  = 0;   /* 上次圈检测的 tick, 用于 150ms 防抖 */
static uint32_t total_fed      = 0;   /* 累计喂字节总数 (仅统计, 无原子保护) */
static uint8_t  last_freq      = 4;   /* 最近设置的频率, 供 ld14p_kick() 重发命令 */

/* ---------- 前向声明 ---------- */
static void ld14p_set_freq(uint8_t freq_hz);

/*
 * ============================================================================
 *  ld14p_update_cloud — 解析一个完整的 47 字节数据包
 * ============================================================================
 *
 * 调用时机: 协议状态机拼够 47 字节后调用 (无论 CRC 是否通过都会调用).
 *
 * 数据包格式 (47 字节固定长):
 *   [0]       0x54        帧头
 *   [1]       0x2C        VerLen: 高 3 位=帧类型(1), 低 5 位=测量点数(12)
 *   [2..3]    speed       LSB 在前, 转速 (°/s)
 *   [4..5]    start_angle LSB 在前, 起始角度 (×0.01°, 0~35999)
 *   [6..41]   points[12]  每点 3 字节: [dist_LSB][dist_MSB][intensity]
 *   [42..43]  end_angle   LSB 在前, 结束角度 (×0.01°)
 *   [44..45]  timestamp   时间戳 (ms, 0~30000 循环)
 *   [46]      crc8        CRC8 校验值, 覆盖字节 [0..45]
 *
 * 本函数做三件事 (按顺序):
 *   1) 角度追踪 + 圈检测 (不依赖 CRC, 每包都执行)
 *   2) CRC8 校验 (失败则丢弃本包点数据)
 *   3) 点云写入 cloud_360[] (覆盖对应角度, 不清空旧数据)
 */
static void ld14p_update_cloud(uint8_t *buf)
{
    /*
     * ─────────── 步骤 1: 提取起止角度 ───────────
     * 协议规定: LSB 先发, MSB 后发 → (高字节 << 8) | 低字节
     */
    uint16_t start_angle = (buf[5] << 8) | buf[4];
    uint16_t end_angle   = (buf[43] << 8) | buf[42];

    /*
     * ─────────── 步骤 2: 圈检测 (角度翻转法) ───────────
     *
     * 原理: LD14P 的角度值从 0 递增到 35999 (359.99°), 然后翻回 0.
     * 当上一个包的 start_angle >300° (即 >30000) 且
     *     当前包的 start_angle <60°  (即 <6000) 时,
     * 说明激光头刚刚穿过 0° 线 → 完成一整圈 360° 扫描.
     *
     * 为什么角度追踪不依赖 CRC?
     *   如果 CRC 失败率很高, 仅靠 CRC 通过的包追踪角度会导致
     *   prev_start_angle 长期不更新, 圈检测永远不触发.
     *   解决方案: 每个拼成的 47 字节包都用来追踪角度
     *   (即使 CRC 失败, start_angle 的取值通常仍然是正确的).
     *
     * 防抖: 两次圈检测间隔必须 >150ms.
     *   防止随机噪声数据中偶然的 start_angle 匹配组合导致误触发.
     *   正常速率下: 4Hz → 250ms/圈, 6Hz → 167ms/圈, 150ms 门限合理.
     */
    uint32_t now = xTaskGetTickCount();
    if (prev_start_angle > 30000 && start_angle < 6000
        && (now - rev_last_tick) > pdMS_TO_TICKS(150)) {
        revolution_flag = true;    /* 置位 → ld14p_scan_ready() 下次返回 true */
        rev_last_tick   = now;
        ESP_LOGI(TAG, "Revolution complete! pkts_ok=%lu pkts_fail=%lu start=%u end=%u",
                 crc_ok_count, crc_fail_count, start_angle, end_angle);
        crc_ok_count   = 0;       /* 重置本圈计数器, 开始新一圈统计 */
        crc_fail_count = 0;
        /*
         * 注意: 这里不清空 cloud_360[].
         * cloud_360 是持久化的 — 旧圈数据保留, 新圈数据覆盖刷新.
         * 如果在这里 memset 清空, 而 sensor 任务还没有调用 get_scan 读取,
         * 旧圈数据就丢失了. 详见调试记录中 360→31 valid pts 的问题.
         */
    }
    prev_start_angle = start_angle;  /* 更新用于下一包的角度比较 */

    /*
     * ─────────── 步骤 3: CRC8 校验 ───────────
     * 计算范围: buf[0..45] (不含末尾 CRC 字节)
     * 与 buf[46] 比较, 不匹配 → 本包被视为噪声/传输错误, 丢弃点数据.
     * 每个圈最多打印 3 条 CRC 失败警告 (避免日志洪泛).
     */
    uint8_t crc = ld14p_crc8(buf, LD14P_PACKET_LEN - 1);
    if (crc != buf[LD14P_PACKET_LEN - 1]) {
        crc_fail_count++;
        if (crc_fail_count <= 3)
            ESP_LOGW(TAG, "CRC fail #%lu calc=%02x recv=%02x",
                     crc_fail_count, crc, buf[LD14P_PACKET_LEN - 1]);
        return;  /* 跳过本包的点数据写入 */
    }
    crc_ok_count++;

    /*
     * ─────────── 步骤 4: 角度插值 ───────────
     *
     * 一个包里有 12 个采样点.
     * 雷达在 start_angle ~ end_angle 之间匀速旋转, 每个采样点的时间间隔相等.
     * 因此 12 个点的角度 = start_angle + i × step (i = 0..11).
     *
     * diff: 本包的角跨度 (0.01° 单位).
     *   如果 end < start → 说明跨过了 360° 线, 加 36000 补偿.
     *
     * step: 相邻采样点的角度增量.
     *   12 个点有 11 个间隔, 所以 step = diff / 11.
     *
     * 单位换算: raw / 100 → 度 (0.01° → 1°)
     *           % 360  → 归一化到 0~359°
     */
    int16_t diff = end_angle - start_angle;
    if (diff < 0) diff += 360 * LD14P_ANGLE_RES;   /* 跨零度补偿 */
    float step = (float)diff / (LD14P_POINT_PER_PACK - 1);

    for (int i = 0; i < LD14P_POINT_PER_PACK; i++) {
        int raw = start_angle + (int)(i * step);
        int deg = (raw / LD14P_ANGLE_RES) % LD14P_POINTS_PER_REV;

        /*
         * ─── 步骤 5: 写入 cloud_360[deg] ───
         *
         * 点数据偏移 (每点 3 字节):
         *   byte[6 + 3×i]     = dist_LSB  (距离低字节)
         *   byte[7 + 3×i]     = dist_MSB  (距离高字节)
         *   byte[8 + 3×i]     = intensity (反射强度)
         *
         * 协议规定 LSB 在前, 所以: distance = (MSB << 8) | LSB
         *
         * 直接覆盖 cloud_360[deg] — 不存在先读后写的数据竞争,
         * 因为所有 cloud_360 访问都在同一个 sensor 任务中完成.
         */
        cloud_360[deg].distance  = (buf[3 * i + 7] << 8) | buf[3 * i + 6];
        cloud_360[deg].intensity = buf[3 * i + 8];
    }
}

/*
 * ============================================================================
 *  ld14p_feed_byte — 字节级协议状态机
 * ============================================================================
 *
 * 这是整个驱动的入口. sensor 任务从 UART 读到的每个字节都调用此函数.
 *
 * 2 状态 FSM:
 *   state=0 (搜帧头): 丢弃所有非 0x54 的字节. 一旦发现 0x54 → 进入 state=1.
 *   state=1 (拼包):   将后续字节依次填入 buf, 满 47 字节 → 调用 ld14p_update_cloud()
 *                      → 返回 state=0 继续搜下一帧头.
 *
 * 为什么是这种"丢弃非 0x54"的设计?
 *   雷达连续发数, 一旦芯片启动或 UART 线路有噪声, 初始收到的数据可能
 *   不在帧对齐位置. 必须找到第一个 0x54 才开始正确拼包.
 *   之后每 47 字节自动对齐 (协议固定包长), 后面的 0x54 会自然落在 buf[0].
 *
 *   如果中途因噪声丢字节 → 下一次搜到的 0x54 对齐位置可能偏差,
 *   但 CRC8 校验会快速淘汰错位的包. 通常 1~2 个包内就能重新对齐.
 */
void ld14p_feed_byte(uint8_t byte)
{
    /*
     * static 变量在函数内部, 生命周期 = 整个程序, 作用域 = 仅本函数.
     * 好处: 外部无法直接访问状态机内部变量, 封装性好.
     */
    static uint8_t state = 0;                   /* 0=搜帧头, 1=拼包 */
    static uint8_t buf[LD14P_PACKET_LEN];       /* 包组装缓冲区 */
    static uint8_t idx  = 0;                    /* 当前写入位置 */

    total_fed++;  /* 统计用, 无锁递增 (仅 sensor 任务访问, 无竞争) */

    if (state == 0) {
        /*
         * ─── 状态 0: 搜帧头 0x54 ───
         * 所有非 0x54 的字节被静默丢弃.
         * 命中 0x54 → 存入 buf[0], idx 跳到 1, 切换到 state=1.
         */
        if (byte == LD14P_HEADER) {
            buf[0] = LD14P_HEADER;
            idx    = 1;
            state  = 1;
        }
    } else {
        /*
         * ─── 状态 1: 拼 47 字节包 ───
         * 每个字节存入 buf[idx], idx 递增.
         * idx 到达 47 → 一个完整包完成:
         *   1) 重置状态机 (准备收下一包)
         *   2) 调用 ld14p_update_cloud() 解析本包
         *   注意: update_cloud 内部会做 CRC 校验, 通过才写入 cloud_360.
         */
        buf[idx++] = byte;
        if (idx >= LD14P_PACKET_LEN) {
            state = 0;
            idx   = 0;
            ld14p_update_cloud(buf);
        }
    }
}

/* ────────── 公开查询 API ────────── */

uint32_t ld14p_get_total_bytes(void)
{
    return total_fed;
}

void ld14p_kick(void)
{
    ld14p_set_freq(last_freq);
}

/*
 * ld14p_scan_ready — 检查并消费圈完成标志
 *
 * 每次调用:
 *   - 如果 revolution_flag==true → 返回 true, 同时清除标志 (一次消费).
 *   - 如果 revolution_flag==false → 返回 false.
 *
 * 典型用法 (在 sensor 任务中):
 *   if (ld14p_scan_ready()) {
 *       ld14p_get_scan(...);  // 快照 360° 数据
 *   }
 */
bool ld14p_scan_ready(void)
{
    bool ret = revolution_flag;
    if (ret) revolution_flag = false;
    return ret;
}

/*
 * ld14p_get_scan — 获取当前 360° 点云快照
 *
 * 将驱动内部的 cloud_360[360] 拷贝到用户提供的 out 缓冲区.
 * 输出: out[i].angle_deg = i (0° ~ 359°, 传感器正前方为 0°, 顺时针递增)
 *       out[i].distance_mm = 距离 (毫米) 或 65535 (未收到数据)
 *
 * 不做坐标系转换 (保持 LD14P 原生坐标系).
 * 不需要互斥锁 — 调用者 (sensor 任务) 独占访问 cloud_360.
 */
esp_err_t ld14p_get_scan(vector_polar_t *out, uint16_t *count)
{
    if (!out || !count) return ESP_ERR_INVALID_ARG;

    *count = LD14P_POINTS_PER_REV;
    for (int i = 0; i < LD14P_POINTS_PER_REV; i++) {
        out[i].angle_deg   = (float)i;
        out[i].distance_mm = (float)cloud_360[i].distance;
    }
    return ESP_OK;
}

/*
 * ============================================================================
 *  ld14p_set_freq — 发送 0xA2 频率控制命令
 * ============================================================================
 *
 * LD14P 的 PWM/RX 引脚是双功能复用:
 *   - 浮空或接地 → PWM 控速模式 (内部 6Hz 默认)
 *   - 接 TX 发送串口命令 → 软件控速模式
 *
 * 本项目将 GPIO17 (TX) 接到 PWM/RX, 所以可以用串口命令精确控制转速.
 *
 * 命令格式 (8 字节):
 *   [0]  0x54        帧头
 *   [1]  0xA2        命令码 (设置转速)
 *   [2]  0x04        数据长度
 *   [3..4] speed     LSB 在前, 目标转速 (°/s)
 *   [5..6] 0x0000    预留
 *   [7]  crc8        CRC 覆盖前 7 字节
 *
 * 例: 4Hz → speed = 4 × 360 = 1440°/s
 */
static void ld14p_set_freq(uint8_t freq_hz)
{
    last_freq = freq_hz;
    uint16_t speed = (uint16_t)freq_hz * 360;
    uint8_t cmd[8] = {
        0x54,                      /* 帧头 */
        LD14P_CMD_SPEED,           /* 命令码 0xA2 */
        LD14P_CMD_LEN,             /* 数据长度 4 */
        (uint8_t)(speed & 0xFF),   /* speed LSB */
        (uint8_t)(speed >> 8),     /* speed MSB */
        0x00, 0x00,               /* 预留 */
        0x00                       /* CRC 占位, 下面计算 */
    };
    cmd[7] = ld14p_crc8(cmd, 7);  /* CRC 覆盖 cmd[0..6] */
    uart_write_bytes(LD14P_UART_NUM, cmd, sizeof(cmd));
    ESP_LOGI(TAG, "Set freq %d Hz (%d deg/s)", freq_hz, speed);
}

/*
 * ============================================================================
 *  ld14p_init — LD14P 驱动初始化
 * ============================================================================
 *
 * 调用一次 (在 app_main 中), 完成:
 *   1) 参数校验 (freq_hz: 2~8)
 *   2) 清空 cloud_360[] (所有距离 = 0xFFFF → 标记为无效)
 *   3) 安装 UART1 驱动 (115200-8N1, GPIO17=TX, GPIO18=RX)
 *   4) 发送频率命令 (0xA2, 让 LD14P 切换到指定扫描频率)
 *   5) 等待 300ms (电机稳定 + 初始化环形缓冲区积压)
 *
 * 重要设计决策:
 *   - event_queue = NULL, 不使用 UART 事件队列.
 *     因此 sensor 任务必须用 timeout=0 的非阻塞读取, 否则触发 StoreProhibited.
 *   - TX buffer = 0  因为只发送 8 字节命令, 足够用硬件 FIFO 直写.
 *   - RX buffer = 2048×2 = 4096 字节  约 355ms 的数据缓冲容量.
 *     sensor 任务以 20ms 间隔 drain, 在此容量下不会溢出.
 */
esp_err_t ld14p_init(uint8_t freq_hz)
{
    if (freq_hz < 2 || freq_hz > 8) return ESP_ERR_INVALID_ARG;

    /*
     * 一次性初始化 cloud_360 为全 0xFF.
     * distance=0xFFFF → 判定为无效 (distance > 60000).
     * 之后永不主动清空, 由每圈数据自然覆盖刷新.
     */
    memset(cloud_360, 0xFF, sizeof(cloud_360));

    /* ─── UART1 配置: 115200 bps, 8N1, 无流控 ─── */
    uart_config_t cfg = {
        .baud_rate  = LD14P_UART_BAUD,         /* 115200 (实测, 非手册的 230400) */
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /*
     * ESP-IDF v5.x 推荐初始化顺序:
     *   param_config → set_pin → driver_install
     * (旧版本顺序为 param_config → driver_install → set_pin, 两种都兼容)
     */
    esp_err_t err = uart_param_config(LD14P_UART_NUM, &cfg);
    if (err != ESP_OK) return err;

    err = uart_set_pin(LD14P_UART_NUM,
                       LD14P_UART_TX_PIN,      /* GPIO17 → LD14P PWM/RX */
                       LD14P_UART_RX_PIN,      /* GPIO18 → LD14P TX */
                       UART_PIN_NO_CHANGE,     /* RTS 不使用 */
                       UART_PIN_NO_CHANGE);    /* CTS 不使用 */
    if (err != ESP_OK) return err;

    /*
     * uart_driver_install 参数:
     *   rx_buf = 4096, tx_buf = 0,
     *   queue_size = 0 (不用事件队列),
     *   uart_queue  = NULL → **关键**: sensor 任务必须用 timeout=0 读, 否则崩溃.
     */
    err = uart_driver_install(LD14P_UART_NUM,
                              LD14P_UART_RX_BUF * 2, /* 4096 字节 RX 环形缓冲区 */
                              0,                      /* TX 缓冲区大小 (0=仅硬件FIFO) */
                              0,                      /* 事件队列大小 */
                              NULL,                   /* 事件队列句柄 (NULL!) */
                              0);                     /* 中断分配标志 */
    if (err != ESP_OK) return err;

    /*
     * 时序控制:
     *   - 100ms 等待: 让 UART 硬件稳定, 环形缓冲区开始接收 LD14P 数据.
     *   - 发送频率命令: 0xA2 命令通过 TX(17) → LD14P PWM/RX.
     *   - 200ms 等待: 让 LD14P 电机响应新转速并稳定.
     *
     * 这 300ms 期间, UART RX 环形缓冲区会积压约 300ms × 11520 B/s ≈ 3456 字节.
     * sensor 任务启动后用 fast drain (50批×256B) 可在 1~2 个循环内全部处理掉.
     */
    vTaskDelay(pdMS_TO_TICKS(100));
    ld14p_set_freq(freq_hz);
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "LD14P ready on UART1 (TX:17 RX:18) @ %d Hz", freq_hz);
    return ESP_OK;
}
