# 人工势场法智能小车

## 流程规划

1. 系统整体规划
2. 规划完善各模块逻辑
<!-- 3. 搭建Python仿真，验证传感器数据处理、APF算法有效性 -->
<!-- 4. 联合仿真，验证整个系统是否能够协调运行 -->
5. 各模块硬件调试实操（逐个模块验证功能，初步联调）
6. 初步系统功能验证（单一障碍物验证）
7. 调试调优（优化）
8. 完整系统功能验证（仿真场景验证）

## 硬件架构

1. 主控：ESP32-S3
2. 传感器：
    * ~~MLX90640：110°H \* 75°V 的 32 \* 24 像素数据降采样到 16 点，预计采用8或4Hz刷新率，取决于单片机处理能力~~
    * ~~VL53L0X：前向60°间隔对称排列（合120°）的三个距离传感器，刷新率与MLX90640同步~~
    * 使用5路红外火焰传感器，将5路的火焰状态直接转化为5点传感器数据，在4Hz下直接读取
    * 使用二手/拆机的LD14激光雷达降采样为 36 点 4 Hz 传感器数据
    * 温度传感器：DS18B20温度模块，单数据引脚获取温度信息
3. 小车底盘：
    * 保守策略：4轮小车，使用 DRV8833 驱动两侧电机，单侧两电机并联
    * ~~激进策略：2轮平衡车，使用PID平衡控制~~
4. 联网策略：
    * 使用MQTT协议实时上传激光雷达点云等传感器数据
    * 利用云端的强大算力与激光雷达的点云数据，实现对场景的实时2D建模

> 使用VL53L0X/激光雷达时若高温物体导致红外被干扰，可以考虑使用960nm滤光板（淘宝收藏有）

> 在分配引脚时，出于ESP32-S3特性，注意：GPIO0、3、45、46作为`Strapping`引脚，影响芯片启动流程，不建议使用；GPIO19、20、39~42连接了USB调试接口，可能会影响固件烧录与串口（另外UART0连接了43、44）打印；GPIO33~37是PSRAM的数据总线，不建议使用
> 除上述引脚，GPIO26~32属于SPI0/1，被内部固件（XIP）与外部Flash占用，在不了解相关知识时应**禁止使用**
> 总结：建议使用（作为通用）引脚有：GPIO1~2、4~14、15~18、21、38、48~49

## RTOS任务规划

* main中执行所有模块的初始化（非任务）
* 4Hz雷达数据读取、数据转化为统一向量数据（中优先级，阻塞等待中断触发，确保数据及时处理）
* ~~热成像数据读取、距离转化（较雷达稍低，其数据处理较复杂耗时长，避免同优先级的上下文切换损耗）~~
* 4Hz读取火焰传感器读数，转化为简单的向量数据
* 向量数据同步、APF算法运行（高优先级，阻塞等待事件组，含超时保护），输出笛卡尔控制向量
* 采用EMA滤波平滑指令，并将控制向量转化为小车电机控制指令（最高优先级，8Hz周期性触发，确保系统周期稳定）
* 基于MQTT协议的日志上传（数据读取阻塞，低优先级）

预计的加速优化方案：

1. 对于有关角度与三角函数换算的计算：采用直接查表法进行运算，能有效降低计算复杂度与延迟，另外在计算三角函数时，可以使用ESP提供的`cosf`与`sinf`函数
2. 对于点阵等大规模矩阵计算，考虑使用ESP提供的ESP-NN或ESP-DSP库，
3. ESP提供了许多针对浮点计算的加速库，可以放心使用浮点计算

## 项目结构规划

```bash
APF-based-car/
├── lib/                    # 仓库目录，第三方代码库
├── src/                    # 源码目录
│   ├── main.c              # 系统初始化与任务创建
│   ├── tasks/              # 任务实现目录，与任务初始化一一对应
│   |   └── ...
|   └── drivers/            # 硬件驱动目录
│       ├── external/       # 外部模块驱动目录，如激光雷达模块等
│       |   └── ...
│       └── peripheral/     # 单片机外设驱动目录，如UART、I2C等
|           └── ...
└── include/                # 头文件，结构与src中对应
```

## 具体思路

统一仿真数据结构：

1. 传感器获得的距离-角度数据：
    ```c
    typedef struct {
        float distance_mm;
        float angle_deg;
    } vector_polar_t;
    ```
2. 经过转换的笛卡尔向量结构：
    ```c
    typedef struct {
        float x;
        float y;
    } vector_cart_t;
    ```

> 在数据处理过程中，应当充分考虑使用ESP32提供的`dsps_mat_mul_f32`、`esp_nn_multiply_f32`等API加速矩阵乘法等运算的执行速度

<!-- 
### 热成像处理

0. 传感器初始化（在main中）
1. 原始数据读取流程
    1. 调用API获取I2C数据读取 -> 32\*24个原始数据（`temp_matrix`）
    2. 调用API获取温度信息 -> 校准后的数据（或许可以在这一步就进行滤波和降采样以优化性能）
    3. 考虑重写`CalculateTo`函数以降低数据读取的性能开销，或者直接在该阶段进行降采样以优化性能
2. 调用ESP32-NN API执行最大值/平均值池化 -> 16\*1的温度降采样数据
    * 可考虑使用ESP23-DSP API执行更加简化的操作，避免引入ESP-NN框架
3. 温度-距离转换，同时添加方向组成向量信息
    * 若温度-距离转换模型效果不理想，可以考虑面向最终的向量处理设定距离范围，即线性指定感知区/噪声区/危险区对应温度
    * 考虑对于发热物体而言，其距离斥力可能会与温度斥力叠加，温度的影响在感知区中应当减弱（相对于激光雷达而言）
    * 角度映射应当以正前方为0°，顺时针方向增加，与雷达的角度规划相对应

操作流程（为方便将来硬件到位后的验证操作）

0. 在仿真中验证降采样效果、温度-距离转化效果
1. 先直接调用MLX90640 API进行功能验证，使用串口打印出温度结果（考虑进行一定程度的可视化）
2. 实际测试优化后的温度校准函数是否能够正常工作
3. 实际测试降采样效果、温度-距离转换效果
 -->

### 5路火焰传感器

4Hz定时任务，直接获取读数（低表示火焰）

1. 传感器初始化
2. 数据读取
3. 简单判断转化为统一数据向量
4. 发送数据

### 激光雷达处理

快速阻塞检查UART数据，匹配激光雷达设定的4Hz采样速率

1. 传感器初始化
2. 原始数据读取
3. 数据处理降采样（360点->36或72点，取决于算力冗余，采取最小值加权降采样）
4. 添加角度数据，组装统一数据向量
5. 发送数据

### 数据同步与APF

```c
#pragma once
#include <math.h>

#define LEN 36  // 临时定义，根据实际情况修改

vector_polar_t vector_polar_dist[LEN] = {}  // 参考数组
vector_cart_t vector_cart_dist[LEN] = {};   // 参考数组

// 转换函数，实际应当放在`.c`文件中，并且其中的`cos`和`sin`函数可替换为ESP提供的`cosf`和`sinf`函数
// 若计算量较大难以满足性能需求，应当使用查表法，预先计算所有可能度数的rad值
void polar_to_cart_convert(vector_polar_t *vector_polar_dist, vector_cart_t *vector_cart_dist) {
    const float deg_to_rad = M_PI / 180.0f;
    for (int i = 0;i < LEN;i ++) {
        float rad = vector_polar_dist[i].degree * deg_to_rad;
        float dist = vector_polar_dist[i]->distance
        vector_cart_dist[i]->x = dist * cos(rad);
        vector_cart_dist[i]->y = dist * sin(rad);
    }
}

// 相关RTOS函数参考

// 创建队列，LEN应当为全部传感器数据向量个数
QueueHandle_t vector_queue = xQueueCreate(LEN, sizeof(vector_cart_t));
// 发送（传感器任务）
xQueueSend(vector_queue, &vector_d_dist[i], portMAX_DELAY);
// 接收（APF任务）
vector_cart_t vec;
if (xQueueReceive(vector_queue, &vec, pdMS_TO_TICKS(100)) == pdTRUE) {
    // 处理向量...
}
```

1. 数据同步（检查标志位）
2. 数据向量极坐标转笛卡尔坐标，再次检查数据
3. 数据向量（暂定为36+5）筛选与分类，根据幅值分类为危险区（0~1000mm）、感知区（1000~6000mm）、噪声区（6000mm）
4. 笛卡尔坐标向量根据APF算法线性叠加，输出结果

> 对于36+5共计41个数据点，考虑直接预计算序号-角度的对应关系，这样甚至能实现距离-角度的解耦，直接根据距离信号的位置确定其转换到笛卡尔坐标系的参数，降低内存占用与计算延迟

### 向量转电机差速

1. 对接受向量作滤波、插值与限幅（归一化）处理，获得16Hz目标的向量序列
    * 滤波算法：一阶指数平滑（EMA/IIR低通算法）算法，状态暂留，可以实现自然插值
    * $ α = dt_ctrl / (τ + dt_ctrl)，τ为时间常数（建议 0.1~0.15s） $
    * $ y[n] = α * x[n] + (1.0f - α) * y[n-1] $
    * 斜率限制器（Slew Rate Limiter）/向量死区，避免控制指令突变（其实也不需要）
    * 归一化处理，便于后续的差速转换
2. 将向量序列转化为差速占空比信号
3. 将差速占空比信号调制到硬件定时器中，实现调制PWM信号直接控制电机
4. 考虑添加堵转保护机制

## 参考

### 论文

*具体的论文参考详见`Reference.md`中的总结*

### 代码实践

#### 人工势场法

1. [人工势场法MatLab仿真](https://github.com/liuxuexun/Artificial-Potential-Field)
2. [运动规划汇总|C++实现](https://github.com/ai-winter/ros_motion_planning)
3. [运动规划汇总|Python实现](https://github.com/ai-winter/python_motion_planning)
4. [A*与APF混合路径规划MatLab仿真](https://github.com/Huang0035/RRT-and-RRT-star-plus-APF)

#### ESP32-S3

1. [简单的智能循迹小车](https://github.com/xcstudio715/stmcar)
2. [同型号参考：ESP32-S3智能语音助手](https://zhuanlan.zhihu.com/p/29817835305)
3. [ESP-DSP文档](https://docs.espressif.com/projects/esp-dsp/)
4. [ESP-NN文档](https://github.com/espressif/esp-nn)
5. [ESP-NN库](https://components.espressif.com/components/espressif/esp-nn/versions/1.2.1/readme?language=en)

<!-- 
#### MLX90640

1. [MLX90640-C语言库](https://github.com/melexis/mlx90640-library)
 -->

#### DS18B20

1. [DS18B20中文手册](http://file.yfrobot.com.cn/datasheet/DS18B20%E4%B8%AD%E6%96%87%E6%89%8B%E5%86%8C.pdf)
2. [DS18B20数据手册](https://www.analog.com/media/en/technical-documentation/data-sheets/DS18B20.pdf)

#### LD14P

1. [LD14P简单教程](https://blog.kaia.ai/tutorial-connect-ld14p-lidar/)
4. [LD14P-ESPIDF实践](https://jishuzhan.net/article/2033076476967452674)
5. [LD14P官方SDK](https://github.com/ldrobotSensorTeam/ldlidar_sl_sdk/tree/master)
4. [乐动LD14P激光传感器开发手册](https://files.waveshare.com/upload/9/99/LD14P_Development_Manual.pdf)
5. [乐动LD14P激光传感器数据手册](https://www.ldrobot.com/images/2023/03/02/LDROBOT_LD14P%20DataSheet_CN_v0.4_Wlmrp6QT.pdf)
6. [案例](https://github.com/HumbertoDiego/lidar-experiments/blob/main/1-LD14P.md)

#### 电机

1. [DRV8833避坑](https://blog.csdn.net/weixin_27869497/article/details/160731226)
2. [DRV8833原理图](https://img2020.cnblogs.com/blog/1513524/202009/1513524-20200909142148648-57766702.png)
