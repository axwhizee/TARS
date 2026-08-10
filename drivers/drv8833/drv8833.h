/**
 * @file drv8833.h
 * @brief DRV8833 电机驱动 — 公共 API 唯一入口
 *
 * 调用链：init → set_speed / set_velocity + update（周期调用）
 *
 * 使用示例：
 * @code
 * static drv8833_handle_t motor;
 * if (drv8833_init(&motor, NULL) != DRV8833_OK) { return; }   // 失败处理
 *
 * drv8833_set_velocity(&motor, 100.0f, 50.0f);   // 目标速度 (mm)
 * drv8833_update(&motor);                        // 每个控制周期调用
 * @endcode
 *
 * 底盘扩展：DIFF_4WD 已实现；2WD/麦克纳姆/履带为预留接口（DRV8833_ERR_NOT_SUPPORTED）。
 */
#pragma once
#include "drv8833_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 DRV8833：LEDC 定时器/通道配置 → 滑行
 *
 * @param h   实例句柄（调用方静态分配，须清零或为未初始化状态）
 * @param cfg 初始化配置，可为 NULL（使用默认值）
 * @return DRV8833_OK 成功；DRV8833_ERR_PARAM 参数非法；
 *         DRV8833_ERR_NOT_SUPPORTED 底盘类型未实现；DRV8833_ERR_INIT 初始化失败
 */
drv8833_err_t drv8833_init(drv8833_handle_t *h, const drv8833_cfg_t *cfg);

/**
 * @brief 反初始化：PWM 全部输出 0 并复位句柄状态
 *
 * @param h 实例句柄
 * @return DRV8833_OK 成功；其他值表示移植层反初始化失败
 */
drv8833_err_t drv8833_deinit(drv8833_handle_t *h);

/**
 * @brief 设置左右电机转速（-100~100），已修正克服静摩擦力的占空比下限
 *
 * @param h     实例句柄
 * @param left  左侧电机：[-100, 100]（正=前进，负=后退，0=滑行）
 * @param right 右侧电机：同上
 * @return DRV8833_OK 成功；DRV8833_ERR_PARAM 参数非法
 * @note 内部将 [1,100] 重映射到 [MIN_DUTY, MAX_DUTY]
 */
drv8833_err_t drv8833_set_speed(drv8833_handle_t *h, int8_t left, int8_t right);

/**
 * @brief 制动：两路 IN 全部拉高（绕组短接）
 *
 * @param h 实例句柄
 * @return DRV8833_OK 成功
 */
drv8833_err_t drv8833_brake(drv8833_handle_t *h);

/**
 * @brief 滑行：全部通道占空比 0（Hi-Z，无动力）
 *
 * @param h 实例句柄
 * @return DRV8833_OK 成功
 */
drv8833_err_t drv8833_coast(drv8833_handle_t *h);

/**
 * @brief 设置目标速度（笛卡尔 mm）；实际输出在 update() 中经 LPF + 底盘算法产生
 *
 * @param h  实例句柄
 * @param dx 前向分量 (mm)
 * @param dy 侧向分量 (mm)
 * @return DRV8833_OK 成功；DRV8833_ERR_PARAM 参数非法
 */
drv8833_err_t drv8833_set_velocity(drv8833_handle_t *h, float dx, float dy);

/**
 * @brief 周期控制入口：超时归零 → 一阶低通滤波 → 底盘算法 → PWM 输出
 *
 * 应在控制任务每个周期调用一次（即使没有新指令，用于超时与滤波收敛）。
 *
 * @param h 实例句柄
 * @return DRV8833_OK 成功；DRV8833_ERR_PARAM 参数非法；
 *         DRV8833_ERR_NOT_SUPPORTED 底盘类型未实现
 */
drv8833_err_t drv8833_update(drv8833_handle_t *h);

/**
 * @brief 运行时开关一阶低通滤波（仅当 DRV8833_LPF_ENABLE 编译开启时有效）
 *
 * @param h      实例句柄
 * @param enable true=启用滤波，false=直通
 * @return DRV8833_OK 成功；DRV8833_ERR_NOT_SUPPORTED 编译期未启用 LPF
 */
drv8833_err_t drv8833_set_lpf(drv8833_handle_t *h, bool enable);

#ifdef __cplusplus
}
#endif
