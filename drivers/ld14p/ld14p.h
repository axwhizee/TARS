/**
 * @file ld14p.h
 * @brief LD14P 激光雷达驱动 — 公共 API 唯一入口
 *
 * 调用链：init → process（或 feed_byte → collect）→ [calibrate] → get_cloud
 *
 * 使用示例：
 * @code
 * static ld14p_handle_t lidar;
 * static const ld14p_cfg_t cfg = { .target_freq_hz = 4 };
 * if (ld14p_init(&lidar, &cfg) != LD14P_OK) { return; }  // 初始化失败处理
 *
 * const ld14p_polar_t *scan;
 * if (ld14p_process(&lidar, &scan) == LD14P_OK) {  // 一圈完成
 *   process_scan(scan);                            // 消费 scan[360]
 * }
 * @endcode
 */
#pragma once
#include "ld14p_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 LD14P：UART 安装 → 清点云 → 发送频率命令
 *
 * @param h   实例句柄（调用方静态分配，须清零或为未初始化状态）
 * @param cfg 初始化配置，可为 NULL（使用默认值）
 * @return LD14P_OK 成功；LD14P_ERR_PARAM 参数非法；LD14P_ERR_INIT 初始化失败
 */
ld14p_err_t ld14p_init(ld14p_handle_t *h, const ld14p_cfg_t *cfg);

/**
 * @brief 反初始化：释放 UART 资源并复位句柄状态
 *
 * @param h 实例句柄
 * @return LD14P_OK 成功；其他值表示移植层反初始化失败
 */
ld14p_err_t ld14p_deinit(ld14p_handle_t *h);

/**
 * @brief 发送 0xA2 转速控制命令，设定目标扫描频率
 *
 * @param h       实例句柄
 * @param freq_hz 目标扫描频率（2~8 Hz）
 * @return LD14P_OK 成功；LD14P_ERR_PARAM 频率越界；LD14P_ERR_UART 发送失败
 */
ld14p_err_t ld14p_set_freq(ld14p_handle_t *h, uint16_t freq_hz);

/**
 * @brief 字节级状态机 — 逐个字节喂入，搜帧头 0x54 拼 47B，VerLen + CRC8 双校
 *
 * @param h         实例句柄
 * @param byte      UART 读取的一个字节
 * @param out_frame [出参] 通过校验的完整帧指针；无完整帧时为 NULL
 * @return LD14P_OK 有完整帧；LD14P_ERR_NOT_READY 帧未拼满；
 *         LD14P_ERR_FRAME / LD14P_ERR_CRC 帧被丢弃
 */
ld14p_err_t ld14p_feed_byte(ld14p_handle_t *h, uint8_t byte, const ld14p_frame_t **out_frame);

/**
 * @brief 角度插值写入点云 + 跨零点圈检测
 *
 * @param h        实例句柄
 * @param frm      ld14p_feed_byte() 返回的有效帧
 * @param out_scan [出参] 一圈完成时返回 cloud[LD14P_POINTS_ALL] 指针，否则 NULL
 * @return LD14P_OK 一圈完成（调用方应尽快消费）；LD14P_ERR_NOT_READY 圈未完成
 */
ld14p_err_t ld14p_collect(ld14p_handle_t *h, const ld14p_frame_t *frm, const ld14p_polar_t **out_scan);

/**
 * @brief 轮询处理（任务主入口）：读 UART → feed_byte → collect
 *
 * 阻塞语义由移植层决定；本项目使用 timeout=0 非阻塞轮询。
 *
 * @param h        实例句柄
 * @param out_scan [出参] 一圈完成时返回 cloud 指针，否则 NULL
 * @return LD14P_OK 一圈完成；LD14P_ERR_NOT_READY 无完整圈；LD14P_ERR_UART 读错误
 */
ld14p_err_t ld14p_process(ld14p_handle_t *h, const ld14p_polar_t **out_scan);

/**
 * @brief SlTransform 几何校准：修正激光器偏离旋转中心的误差（in-place）
 *
 * 仅修改 points 的 ang 字段，dst 保持不变。
 *
 * @param points   待校准点云（调用方快照）
 * @param count    点数
 * @param offset_x X 轴偏移 (mm)，官方默认 5.9
 * @param offset_y Y 轴偏移 (mm)，官方默认 -18.975571
 * @return LD14P_OK 成功；LD14P_ERR_PARAM 参数非法
 */
ld14p_err_t ld14p_calibrate(ld14p_polar_t *points, uint16_t count, float offset_x, float offset_y);

/**
 * @brief 获取当前累积点云指针（未完成一圈时也可读取）
 *
 * @param h 实例句柄
 * @return cloud 指针；h 为 NULL 时返回 NULL
 */
const ld14p_polar_t *ld14p_get_cloud(ld14p_handle_t *h);

#ifdef __cplusplus
}
#endif
