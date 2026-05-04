#pragma once

/**
 * @brief LD14P传感器任务
 *        连续轮询UART1 → 喂入状态机 → 检测到完整一圈 → 清空队列 → 推入360点
 *        以最高优先级运行, 确保数据及时搬运到队列
 * @param arg QueueHandle_t (深度360, 每元素vector_polar_t)
 */
void ld14p_sensor_task(void *arg);
