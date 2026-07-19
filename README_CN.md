# SA MPPI（Shape Aware MPPI）局部规划器：任意形状机器人动态实时轨迹规划

<p align="center">
  <a href="https://en.cppreference.com/"><img src="https://img.shields.io/badge/C++-17-blue.svg" alt="C++17"></a>
  <a href="https://docs.nav2.org/configuration/packages/configuring-mppic.html"><img src="https://img.shields.io/badge/MPPI-Nav2-ff69b4.svg" alt="MPPI"></a>
  <img src="https://img.shields.io/badge/Platform-Cross--Platform-orange.svg" alt="Cross Platform">
  <a href="https://github.com/xtensor-stack/xtensor"><img src="https://img.shields.io/badge/xtensor-0.21-purple.svg" alt="xtensor"></a>
  <img src="https://img.shields.io/badge/License-GPLv3-blue.svg" alt="License">
  <img src="https://img.shields.io/badge/Status-Active-brightgreen.svg" alt="Status">
</p>

> **声明**：本仓库是对 Navigation2 中 MPPI 局部规划器的解耦版本。由于个人技术能力有限，代码中存在一些"魔改"痕迹与不规范之处，部分实现逻辑可能与原版存在差异，**代码实时更新中**。如有不当之处，还望各位大佬多多指正，感谢包容！欢迎提 Issue 或 PR 一起完善。

>[友情提醒](https://github.com/Robot-Nav/SA-MPPI/issues/2) ：一些建议

> **强烈推荐 Website**：[👉 MPPI变种 ](http://robotics-tutorial.dmbot.cn/04_%E7%A7%BB%E5%8A%A8%E6%9C%BA%E5%99%A8%E4%BA%BA%E8%A7%84%E6%8E%A7/20_%E9%87%87%E6%A0%B7%E5%BC%8FMPC/40_MPPI%E5%85%AD%E5%A4%A7%E5%8F%98%E4%BD%93/#rmppi)
> <img width="1056" height="622" alt="image" src="https://github.com/user-attachments/assets/de6184a6-7b1b-442a-9997-b3f97421c26e" /> 

> **参考博客**：
>
> 1、[Shape-Aware MPPI（SA MPPI）算法：基于RC-ESDF的任意形状机器人实时轨迹优化](https://blog.csdn.net/qq_56908984/article/details/160439414)
>
> 2、[RC-ESDF 详解：以机器人为中心的欧几里得有符号距离场](https://blog.csdn.net/qq_56908984/article/details/160087544)
>
> 3、[MPPI_C++：Nav2-MPPI解耦版局部规划器（可魔改）](https://blog.csdn.net/qq_56908984/article/details/158774722)
>
> 4、[SA-MPPI-cost-calculator在线交互网页](https://opti-mppi-cost-calculator.vercel.app/critic_cost_calculator.html)

## 目录

- [项目概述](#1-项目概述)
- [功能特性](#2-功能特性)
- [数学原理](#3-数学原理)
- [架构设计](#4-架构设计)
- [模块详解](#5-模块详解)
- [安装与编译](#6-安装与编译)
- [使用方法](#7-使用方法)
- [参数配置](#8-参数配置)
- [话题与接口](#9-话题与接口)
- [算法细节](#10-算法细节)
- [性能优化](#11-性能优化)
- [与 Nav2 官方 MPPI 控制器的区别](#12-与-nav2-官方-mppi-控制器的区别)
- [许可证](#13-许可证)
- [参考内容](#14-参考内容)

***

## 1. 项目概述

从 ROS2 Nav2 框架中解耦的 MPPI（Model Predictive Path Integral）控制器实现，采用 **C++17** 编写，适配 ROS1 环境。核心控制器与 Nav2 基础设施（rclcpp、nav2\_core、nav2\_costmap\_2d、pluginlib 等）**无依赖**，仅需标准 C++17 和头文件-only 的 `xtensor` 数值库，与 RC-ESDF 算法进行结合，利用其精确距离查询构建分区碰撞检测机制，并在风险区利用 RC-ESDF 解析梯度对 MPPI 采样进行偏置处理，引导采样轨迹向安全区域扩展，从而提高任意形状机器人动态避障的灵活性与平滑性等。

### 1.1 核心定位

- **框架无关**：不依赖 ROS2/Nav2 生命周期节点、Costmap2D、pluginlib 等基础设施，可嵌入任意 C++ 项目
- **激光雷达适配**：直接使用激光点云作为障碍物输入，通过自建BFS局部栅格距离场实现快速距离查询，无需依赖全局代价地图。局部距离场替代了原 Nav2-MPPI 的代价地图（costmap），将障碍物检测从依赖全局图层的集中式架构解耦为机器人自身的感知闭环。在此基础上该距离场模块可进一步结合 RC-ESDF（Robot-Centric Euclidean Signed Distance Field）进行拓展——RC-ESDF 以机器人本体为中心离线预计算符号距离场，支持任意多边形 footprint 的精确距离查询与碰撞判定；在与 MPPI 结合时，可构建碰撞区、风险区和安全区的分区碰撞检测机制，并在风险区利用其解析梯度对采样轨迹进行偏置，引导轨迹向安全区域扩展，在狭窄空间通过性和非圆形机器人避障方面具有显著优势，详情可参考 [SA-MPPI：基于 RC-ESDF 的任意形状机器人实时轨迹优化](https://blog.csdn.net/qq_56908984/article/details/160439414)
- **轻量部署**：仅依赖 xtensor 数值库，编译简单，适合嵌入式和实时场景
- **功能完整**：保留了 Nav2 MPPI 的核心算法逻辑，并增加了多项实用增强

### 1.2 核心能力

- **实时轨迹优化**：以 20Hz 频率优化控制序列（50ms 控制周期）
- **激光避障**：使用基于 BFS 的局部栅格距离场进行高效碰撞检测
- **多线程采样**：使用可配置线程池进行并行轨迹生成
- **动态障碍物支持**：对移动障碍物的预测性碰撞避免，可扩展
- **阻塞检测**：当被障碍物阻挡时防止原地旋转行为
- **多种运动模型**：支持差速驱动、全向移动和阿克曼转向模型
- **全向/差速自动切换**：根据障碍物距离和路径偏离自动切换运动模型

### 1.3 基础效果演示



https://github.com/user-attachments/assets/575febab-0bb6-4e82-a43d-874637169713



***

## 2. 功能特性

### 2.1 核心功能

| 功能          | 描述                            |
| ----------- | ----------------------------- |
| **批量采样**    | 每个控制周期 1000+ 条轨迹样本            |
| **栅格距离场**   | 基于 BFS 的局部距离场，实现 O(1) 障碍物距离查询 |
| **多线程**     | 可配置线程数进行轨迹积分并行计算              |
| **自适应温度**   | 基于代价分布的动态温度调节                 |
| **SG 滤波**   | Savitzky-Golay 滤波器实现平滑控制序列    |
| **足迹支持**    | 非圆形机器人碰撞检测（多边形足迹）             |
| **动态障碍物**   | 基于速度的线性预测碰撞避免                 |
| **阻塞检测**    | 检测原地打转并输出零速度                  |
| **全向/差速切换** | 根据环境自动切换运动模型                  |
| **路径阻塞检测**  | 路径被大量障碍物占据时自动失效对齐代价           |
| **路径点有效性**  | 跳过被障碍物阻挡的路径点                  |
| **启动辅助**    | 解决零速度启动困境 / 无用，可删除            |

### 2.2 代价函数组件

| 代价函数                     | 用途                          | 激活条件    |
| ------------------------ | --------------------------- | ------- |
| `ObstaclesCritic`        | 基于指数排斥代价的碰撞避免（栅格距离场 + 动态预测） | 远离目标时   |
| `PathFollowCritic`       | 沿全局路径的终点跟踪                  | 远离目标时   |
| `PathAlignCritic`        | 轨迹与路径几何对齐（含路径阻塞检测）          | 路径未被阻塞时 |
| `PathAngleCritic`        | 航向与路径方向对齐（三种模式）             | 远离目标时   |
| `GoalCritic`             | 最终位置收敛                      | 接近目标时   |
| `GoalAngleCritic`        | 最终航向对齐                      | 接近目标时   |
| `PreferForwardCritic`    | 惩罚后退运动                      | 远离目标时   |
| `ConstraintCritic`       | 运动学约束执行（差速/全向/阿克曼）          | 始终      |
| `TwirlingCritic`         | 抑制原地旋转行为                    | 远离目标时   |
| `VelocityDeadbandCritic` | 低速稳定性改进                     | 始终      |

### 2.3 跨平台可移植性

| 平台                  | 支持   | 说明             |
| ------------------- | ---- | -------------- |
| **Linux (x86/ARM)** | 完全支持 | 主要开发平台         |
| **ROS1**            | 完全支持 | 提供参考实现         |
| **ROS2**            | 兼容   | 使用 ROS2 消息类型包装 |
| **嵌入式 Linux**       | 完全支持 | 可自行测试          |
| **实时操作系统**          | 兼容   | 热路径无动态内存分配     |

**依赖项（仅核心控制器）**：

- C++17 编译器
- [xtensor 0.21版本](https://github.com/xtensor-stack/xtensor)（头文件-only 数值库）
- 仅标准 C++ 库

***

## 3. 数学原理

### 3.1 MPPI 核心算法

MPPI 算法通过采样轨迹并计算控制序列的加权平均来解决随机最优控制问题。与传统 MPC 需要求解约束优化问题不同，MPPI 将最优控制问题转化为**路径积分（Path Integral）估计问题**，通过蒙特卡洛采样近似求解，避免了复杂的梯度计算和约束处理。

#### 状态动力学（差速驱动）

$$\dot{x} = v\_x \cos\theta$$

$$\dot{y} = v\_x \sin\theta$$

$$\dot{\theta} = \omega\_z$$

其中：

- $(x, y)$：机器人在世界坐标系中的位置
- $\theta$：机器人航向
- $v\_x$：线速度
- $\omega\_z$：角速度

#### 离散化运动模型

$$x\_{t+1} = x\_t + (v\_x \cos\theta - v\_y \sin\theta) \cdot \Delta t$$

$$y\_{t+1} = y\_t + (v\_x \sin\theta + v\_y \cos\theta) \cdot \Delta t$$

$$\theta\_{t+1} = \theta\_t + \omega \cdot \Delta t$$

其中 $v\_y$ 仅在全向运动模型中非零。

#### 轨迹采样

对于每个批次 $i$ 和时间步 $t$：

$$u\_t^{(i)} = \bar{\nu}\_t + \epsilon\_t^{(i)}, \quad \epsilon\_t^{(i)} \sim \mathcal{N}(0, \Sigma)$$

其中：

- $\bar{\nu}\_t$：前一次迭代的平均控制量（当前控制序列）
- $\epsilon\_t^{(i)}$：标准差为 $\sigma$ 的高斯噪声

#### 轨迹代价函数

轨迹 $i$ 的总代价：

$$S(\tau^{(i)}) = \sum\_{t=0}^{T-1} \left\[ q(x\_t^{(i)}) + \frac{\gamma}{2\sigma^2} \bar{\nu}\_t^2 + \frac{\gamma}{\sigma^2} \bar{\nu}\_t \cdot \epsilon\_t^{(i)} \right] \cdot \Delta t$$

其中：

- $q(x\_t^{(i)})$：状态代价（由各 Critic 函数计算）
- $\gamma$：控制代价权重参数
- $\sigma$：采样噪声标准差
- $\bar{\nu}\_t$：当前控制序列
- $\epsilon\_t^{(i)}$：第 $i$ 条轨迹的噪声
- $T$：时间范围（步数）

代价函数由三部分组成：

1. **状态代价** $q(x\_t^{(i)})$：由 10 个 Critic 函数独立计算并累加
2. **控制正则项** $\frac{\gamma}{2\sigma^2}\bar{\nu}\_t^2$：惩罚控制量过大
3. **控制-噪声交叉项** $\frac{\gamma}{\sigma^2}\bar{\nu}\_t \cdot \epsilon\_t^{(i)}$：确保最优控制更新无偏

#### Softmax 加权

最优控制更新使用 softmax 权重：

$$w^{(i)} = \frac{\exp\left(-\frac{1}{\lambda}(S(\tau^{(i)}) - S\_{\min})\right)}{\sum\_{j=1}^{K} \exp\left(-\frac{1}{\lambda}(S(\tau^{(j)}) - S\_{\min})\right)}$$

其中：

- $\lambda$：温度参数，控制权重分布的尖锐程度
- $S\_{\min}$：所有轨迹中的最小代价（用于数值稳定）
- $K$：批次大小（样本数）

还支持**均值归一化**模式：

$$w^{(i)} = \frac{\exp\left(-\frac{1}{\lambda}(S(\tau^{(i)}) - \bar{S})\right)}{\sum\_{j=1}^{K} \exp\left(-\frac{1}{\lambda}(S(\tau^{(j)}) - \bar{S})\right)}$$

其中 $\bar{S}$ 为所有轨迹代价的均值。

#### 控制更新

更新后的控制序列：

$$\bar{\nu}_t^{\text{new}} = \sum_{i=1}^{K} w^{(i)} \cdot u\_t^{(i)}$$

### 3.2 障碍物代价函数

障碍物代价函数使用**指数排斥代价**：

$$C\_{\text{obs}}(d) = \begin{cases} C\_{\text{collision}} & \text{if } d < d\_{\text{margin}} \ w\_{\text{repulsion}} \cdot \exp(-\alpha \cdot (d - d\_{\text{margin}})) & \text{if } d\_{\text{margin}} \le d < r\_{\text{inflation}} \ 0 & \text{otherwise} \end{cases}$$

其中：

- $d$：到最近障碍物的距离
- $d\_{\text{margin}}$：碰撞边界距离（`collision_margin`）
- $r\_{\text{inflation}}$：膨胀半径（`inflation_radius`）
- $\alpha$：代价缩放因子（`cost_scaling`）
- $w\_{\text{repulsion}}$：排斥权重（`repulsion_weight`）
- $C\_{\text{collision}}$：碰撞代价（极大值，如 100000）

### 3.3 栅格距离场

局部距离场使用 BFS（广度优先搜索）计算，以机器人为中心构建：

```
对于每个栅格单元 (i, j)：
  D[i,j] = min_{障碍物单元} ||(i,j) - (i_obs, j_obs)||
```

构建过程：

1. 将所有障碍物点映射到栅格坐标，距离设为 0
2. 从所有障碍物栅格开始 BFS 四邻域传播
3. 每个邻居的距离 = 当前距离 + 栅格分辨率

查询时直接通过坐标映射获取距离，时间复杂度 O(1)。

默认参数：分辨率 0.05m，100×100 栅格，覆盖 5m×5m 局部区域。

### 3.4 加速度约束

速度更新遵守加速度限制：

$$v\_{t+1} = v\_t + \text{clamp}(v\_{\text{desired}} - v\_t, -a\_{\max} \Delta t, a\_{\max} \Delta t)$$

其中：

- $a\_{\max}$：最大加速度
- $\Delta t$：时间步长

该约束在运动模型的 `predict()` 方法中实现，确保生成的轨迹满足物理加速度限制。

### 3.5 阻塞检测

控制器使用旋转比率检测阻塞情况：

$$\text{ratio} = \frac{|\omega\_z|}{|v\_x| + \epsilon}$$

如果 $\text{ratio} > \theta\_{\text{threshold}}$ 持续 $N$ 帧，则认为机器人被阻塞并命令零速度。

阻塞检测分为两层：

1. **TwirlingCritic**：通过代价函数惩罚"低前进+高旋转"行为，属于软约束
2. **Optimizer::isBlocked()**：通过检测控制序列的平均 wz/vx 比率，连续多帧超过阈值时输出零速度，属于硬约束

### 3.6 Savitzky-Golay 滤波

控制序列平滑使用 SG 滤波系数：

$$\nu\_t^{\text{filtered}} = \sum\_{k=-2}^{2} c\_{k+2} \cdot \nu\_{t+k}$$

默认系数：$c = \[-0.085714, 0.342857, 0.485714, 0.342857, -0.085714]$（5 点 2 次拟合）。

边界处理：当索引超出控制序列范围时，使用控制历史（前几帧的实际控制量）进行填充。

### 3.7 自适应温度

当启用自适应温度时，温度参数根据代价分布动态调整：

$$T = \text{clamp}\left(T\_{\text{base}} \cdot \left(1 + \frac{\ln(S\_{\max} - S\_{\min} + 1)}{5}\right), T\_{\min}, T\_{\max}\right)$$

代价分布范围大时自动增大温度平滑权重，代价分布集中时降低温度突出最优轨迹。

***

## 4. 架构设计

### 4.1 分层架构

采用**分层架构**，核心算法与框架特定实现分离：

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROS1 应用层                               │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              mppi_ros1_node.cpp                          │   │
│  │  订阅: /plan, /odom, /scan  |  发布: /cmd_vel, 调试轨迹  │   │
│  │  参数加载 | 启动辅助 | 坐标变换 | 异常处理               │   │
│  └──────────────────────┬───────────────────────────────────┘   │
└─────────────────────────┼───────────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────────┐
│                     控制器接口层                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              MPPIController (controller.hpp)              │   │
│  │  初始化 | 代价参数设置 | 障碍物更新 | 运动模型切换       │   │
│  │  路径点有效性 | 全向/差速自动切换                         │   │
│  └──────────┬──────────────────────────┬────────────────────┘   │
└─────────────┼──────────────────────────┼────────────────────────┘
              │                          │
┌─────────────▼──────────┐  ┌────────────▼────────────────────────┐
│     优化器核心层        │  │         代价函数层                   │
│  ┌──────────────────┐  │  │  ┌─────────────────────────────┐    │
│  │  Optimizer       │  │  │  │     CriticManager           │    │
│  │  采样 | 评分     │  │  │  │  ┌───────────────────────┐  │    │
│  │  加权更新 | 滤波 │  │  │  │  │ ObstaclesCritic       │  │    │
│  │  阻塞检测       │  │  │  │  │ PathFollowCritic      │  │    │
│  │  路径裁剪       │  │  │  │  │ PathAlignCritic       │  │    │
│  └───────┬──────────┘  │  │  │  │ PathAngleCritic       │  │    │
│          │              │  │  │  │ GoalCritic            │  │    │
│  ┌───────▼──────────┐  │  │  │  │ GoalAngleCritic       │  │    │
│  │  NoiseGenerator  │  │  │  │  │ PreferForwardCritic   │  │    │
│  │  异步噪声线程    │  │  │  │  │ ConstraintCritic      │  │    │
│  └──────────────────┘  │  │  │  │ TwirlingCritic        │  │    │
│  ┌──────────────────┐  │  │  │  │ VelocityDeadbandCrit  │  │    │
│  │  MotionModel     │  │  │  │  └───────────────────────┘  │    │
│  │  Diff | Omni     │  │  │  └─────────────────────────────┘    │
│  │  Ackermann       │  │  │                                      │
│  └──────────────────┘  │  │                                      │
└────────────────────────┘  └──────────────────────────────────────┘
              │                          │
┌─────────────▼──────────────────────────▼──────────────────────────┐
│                       数据模型层                                   │
│  types | constraints | control_sequence | state | trajectories   │
│  path  | optimizer_settings                                        │
├───────────────────────────────────────────────────────────────────┤
│                       工具层                                      │
│  math_utils (角度规范化, clamp) | noise_generator (异步噪声)      │
├───────────────────────────────────────────────────────────────────┤
│                       外部依赖 (最小化)                            │
│  xtensor (头文件-only) | xsimd (SIMD加速) | xtl (基础库) | C++17 │
└───────────────────────────────────────────────────────────────────┘
```

### 4.2 关键设计原则

1. **核心零 ROS 依赖**：所有 `include/` 下的头文件仅使用标准 C++17 和 xtensor，不包含任何 ROS 头文件
2. **模块化拆分**：按功能职责拆分为 20+ 个头文件，每个文件专注于单一职责
3. **基于继承的设计**：运动模型和代价函数均通过基类定义接口，派生类实现具体逻辑
4. **异步噪声生成**：独立线程预生成噪声，减少主循环延迟
5. **多线程轨迹积分**：可配置线程数并行计算
6. **确定性执行**：控制循环无非确定性操作

### 4.3 文件组织结构

```
mppi_laser_example/
├── include/
│   ├── controller.hpp              # 对外接口：MPPIController 类
│   ├── optimizer.hpp               # 优化器核心：采样、评分、加权更新
│   ├── motion_models.hpp           # 运动模型：差速/全向/阿克曼
│   ├── models/
│   │   ├── types.hpp               # 基础类型：Point2D, Pose2D, Twist2D, Control, ObstaclePrediction
│   │   ├── constraints.hpp         # 约束参数：ControlConstraints, SamplingStd
│   │   ├── control_sequence.hpp    # 控制序列数据结构
│   │   ├── state.hpp               # 状态容器（批次速度+控制量）
│   │   ├── trajectories.hpp        # 轨迹容器（批次位置+朝向）
│   │   ├── path.hpp                # 全局路径数据结构
│   │   └── optimizer_settings.hpp  # 优化器配置参数
│   ├── critics/
│   │   ├── critic_function.hpp     # 代价函数基类
│   │   ├── critic_data.hpp         # 评价数据结构
│   │   ├── critic_manager.hpp      # 代价函数管理器
│   │   ├── obstacles_critic.hpp    # 障碍物代价（栅格距离场）
│   │   ├── path_follow_critic.hpp  # 路径跟随代价
│   │   ├── path_align_critic.hpp   # 路径对齐代价
│   │   ├── path_angle_critic.hpp   # 路径角度代价
│   │   ├── goal_critic.hpp         # 目标点代价
│   │   ├── goal_angle_critic.hpp   # 目标朝向代价
│   │   ├── prefer_forward_critic.hpp # 偏好前进代价
│   │   ├── constraint_critic.hpp   # 运动学约束代价
│   │   ├── twirling_critic.hpp     # 原地打转惩罚代价
│   │   └── velocity_deadband_critic.hpp # 速度死区代价
│   └── tools/
│       ├── math_utils.hpp          # 数学工具：角度规范化、clamp
│       └── noise_generator.hpp     # 异步高斯噪声生成器
├── src/
│   └── mppi_ros1_node.cpp          # ROS1 主节点
├── scripts/
│   ├── mppi_path_publisher.py      # 路径发布器（直线/矩形/S形）
│   ├── mppi_path_publisher_fz.py   # 增强版路径发布器（支持旋转）
├── config/
│   ├── mppi_params.yaml            # 参数配置（详细中文注释）
│   └── mppi_test.rviz              # RViz 可视化配置
├── CMakeLists.txt                  # Catkin 构建配置
├── package.xml                     # ROS1 包清单
├── xtensor/                        # 第三方库：多维数组
├── xsimd/                          # 第三方库：SIMD 向量化
└── xtl/                            # 第三方库：xtensor 基础库
```

***

## 5. 模块详解

### 5.1 数据模型层（models/）

#### types.hpp — 基础数据类型

| 类型                   | 字段                   | 说明                           |
| -------------------- | -------------------- | ---------------------------- |
| `Point2D`            | x, y                 | 二维点                          |
| `Pose2D`             | x, y, theta          | 二维位姿                         |
| `Twist2D`            | vx, vy, wz           | 二维速度                         |
| `Control`            | vx, vy, wz           | 单步控制量                        |
| `ObstaclePrediction` | x, y, vx, vy, radius | 动态障碍物预测，含 `positionAt(t)` 方法 |

`ObstaclePrediction::positionAt(t)` 根据障碍物速度进行线性预测：

$$\hat{x}(t) = x\_0 + v\_x \cdot t, \quad \hat{y}(t) = y\_0 + v\_y \cdot t$$

#### constraints.hpp — 控制约束

- `ControlConstraints`：速度限制（vx\_max/min, vy\_max, wz\_max）、加速度限制（ax/ay/az\_max）、碰撞代价阈值
- `SamplingStd`：采样噪声标准差（vx, vy, wz）

#### control\_sequence.hpp — 控制序列

`ControlSequence`：每个时间步的控制量（vx\[], vy\[], wz\[]），使用 `xt::xtensor<float, 1>` 存储。支持 `reset(time_steps)` 重置。

#### state.hpp — 状态容器

`State`：所有采样轨迹的控制量和速度，均为 `[batch, time]` 二维张量：

- `cvx, cvy, cwz`：含噪声的控制量（采样结果）
- `vx, vy, wz`：经过加速度约束后的速度
- `pose`：当前位姿（Pose2D）
- `speed`：当前速度（Twist2D）

#### trajectories.hpp — 轨迹容器

`Trajectories`：所有采样轨迹的位置和朝向，均为 `[batch, time]` 二维张量：

- `x, y`：位置
- `yaws`：朝向
- `times`：时间戳

#### path.hpp — 全局路径

`Path`：离散路径点序列（x\[], y\[], yaws\[]），含 `getGoal()` 方法获取终点，`reset(size)` 重置。

#### optimizer\_settings.hpp — 优化器配置

`OptimizerSettings`：包含所有可配置参数，详见[参数配置](#8-参数配置)章节。

### 5.2 工具层（tools/）

#### math\_utils.hpp — 基础数学工具

| 函数                                  | 功能                    |
| ----------------------------------- | --------------------- |
| `normalizeAngle(angle)`             | 角度规范化到 $\[-\pi, \pi]$ |
| `shortestAngularDistance(from, to)` | 最短有符号角度差              |
| `clamp(val, min_val, max_val)`      | 值截断                   |

常量定义：`PI = 3.14159265f`, `TWO_PI = 6.28318530f`, `EPSILON = 1e-6f`

#### noise\_generator.hpp — 异步高斯噪声生成器

`NoiseGenerator` 类使用独立线程异步预生成噪声：

```
主线程:  setNoisedControls() → 等待噪声就绪 → 读取噪声 → 触发下一轮生成
噪声线程:  等待触发 → 生成噪声 → 通知就绪 → 等待触发
```

- 使用 `xt::random::randn` 生成高斯噪声
- 使用 `std::mutex` + `std::condition_variable` 实现线程同步
- `setNoisedControls()`：将噪声叠加到控制序列上
- `generateNextNoises()`：触发异步预生成下一组噪声

### 5.3 代价函数层（critics/）

#### critic\_function.hpp — 代价函数基类

```cpp
class CriticFunction {
    virtual void initialize() = 0;
    virtual void score(CriticData & data) = 0;
    void setEnabled(bool enabled);
    bool isEnabled() const;
    void setName(const std::string& name);
    std::string getName() const;
};
```

#### critic\_data.hpp — 评价数据结构

`CriticData`：传递给代价函数的数据集合，包含：

- `state`：状态容器
- `trajectories`：轨迹容器
- `path`：全局路径
- `costs`：代价数组（各 Critic 累加）
- `model_dt`：时间步长
- `fail_flag`：失败标志（所有轨迹碰撞时置 true）
- `motion_model`：当前运动模型
- `furthest_reached_path_point`：最远可达路径点索引
- `dynamic_obstacles`：动态障碍物列表
- `path_pts_valid`：路径点有效性标志

#### critic\_manager.hpp — 代价函数管理器

`CriticManager`：管理所有代价函数，执行 `evalTrajectoriesScores(data)` 批量评分，遍历所有已注册的 Critic 并调用 `score()`。

#### obstacles\_critic.hpp — 障碍物代价（核心）

`ObstaclesCritic` 是最复杂的代价函数，核心流程：

1. **栅格重建**：以机器人位置为中心，将障碍物点映射到局部栅格，BFS 四邻域传播距离场
2. **轨迹评分**：对每条轨迹逐点查询距离场
3. **圆形模式**：查询轨迹点到最近障碍物距离，减去机器人半径
4. **足迹模式**：将足迹点变换到世界坐标，逐点查询距离场，取最小距离
5. **动态障碍物**：根据时间预测障碍物位置，计算与轨迹点的距离
6. **碰撞判定**：距离 < collision\_margin 时视为碰撞，赋予 collision\_cost
7. **排斥代价**：距离在 \[collision\_margin, inflation\_radius) 范围内时，按指数衰减计算排斥力
8. **全碰撞检测**：所有轨迹都碰撞时设置 fail\_flag

#### path\_follow\_critic.hpp — 路径跟随代价

计算轨迹终点与路径参考点之间的距离平方：

$$
C_{\mathrm{follow}} = w \cdot \left\| \mathbf{p}_{\mathrm{traj\_end}} - \mathbf{p}_{\mathrm{path\_ref}} \right\|^2
$$

参考点由 `furthest_reached_path_point + offset` 确定。支持跳过被障碍物阻挡的路径点。

#### path_align_critic.hpp — 路径对齐代价

计算轨迹上每个点与路径最近点距离的平均值：

$$
C_{\text{align}} = w \cdot \frac{1}{N} \sum_{j=0}^{N-1} \left| \mathbf{p}_{\text{traj}}(j) - \mathbf{p}_{\text{path}}(\text{closest}(j)) \right|
$$

关键特性：

- **路径累计距离匹配**：使用 Nav2 风格的累计距离匹配，避免暴力搜索
- **路径阻塞检测**：当路径被障碍物占据比例超过 `max_path_occupancy_ratio` 时，自动失效
- **朝向考虑**：可选地将朝向偏差纳入距离计算：

  $$
  d' = d \cdot (1 + 0.5 \cdot |\Delta\theta|)
  $$

#### path_angle_critic.hpp — 路径角度代价

惩罚轨迹终点与路径参考点之间的朝向偏差。三种模式：

| 模式 | 值 | 说明 |
| --- | --- | --- |
| `FORWARD_PREFERENCE` | 0 | 偏好前进方向，惩罚朝向与路径方向不一致 |
| `NO_DIRECTIONAL_PREFERENCE` | 1 | 允许倒车，仅惩罚朝向偏差大小 |
| `CONSIDER_FEASIBLE_PATH_ORIENTATIONS` | 2 | 根据速度方向选择参考朝向 |

#### goal_critic.hpp — 目标点代价

接近目标时惩罚轨迹终点与目标点的距离平方：

$$
C_{\mathrm{goal}} = w \cdot \left\| \mathbf{p}_{\mathrm{traj\_end}} - \mathbf{p}_{\mathrm{goal}} \right\|^2
$$

仅在距离目标 < `goal_threshold` 时激活。

#### goal_angle_critic.hpp — 目标朝向代价

接近目标时惩罚轨迹终点朝向与目标朝向的偏差平方：

$$
C_{\mathrm{goal\_angle}} = w \cdot (\theta_{\mathrm{traj\_end}} - \theta_{\mathrm{goal}})^2
$$

仅在距离目标 < `goal_angle_threshold` 时激活。

#### prefer\_forward\_critic.hpp — 偏好前进代价

惩罚控制序列中负速度（倒车）的出现：

$$C\_{\text{forward}} = w \cdot \sum\_{t} \max(0, -v\_x(t))$$

仅在距离目标 > `prefer_forward_threshold` 时激活。

#### constraint\_critic.hpp — 约束代价

激励机器人在运动学约束范围内移动，惩罚超出速度限制的行为。支持三种运动模型：

- 差速驱动：惩罚 vy 非零
- 全向移动：无额外约束
- 阿克曼：惩罚转弯半径小于最小值

#### twirling\_critic.hpp — 旋转惩罚代价

惩罚"低前进+高旋转"行为，阻止原地打转：

$$C\_{\text{twirl}} = w \cdot \sum\_{t} \max(0, |w\_z(t)| - |v\_x(t)|)$$

接近目标点 `twirling_threshold` 距离内自动关闭，允许旋转调整朝向。

#### velocity\_deadband\_critic.hpp — 速度死区代价

惩罚速度低于死区阈值的情况，避免低速不稳定：

$$C\_{\text{deadband}} = w \cdot \sum\_{t} \left\[\mathbb{1}(|v\_x| < \epsilon\_{vx}) + \mathbb{1}(|v\_y| < \epsilon\_{vy}) + \mathbb{1}(|w\_z| < \epsilon\_{wz})\right]$$

### 5.4 核心算法层

#### motion\_models.hpp — 运动模型

| 模型    | 类名                     | 全向 | 特殊约束   |
| ----- | ---------------------- | -- | ------ |
| 差速驱动  | `DiffDriveMotionModel` | 否  | 无      |
| 全向移动  | `OmniMotionModel`      | 是  | 无      |
| 阿克曼转向 | `AckermannMotionModel` | 否  | 最小转弯半径 |

所有模型共享基类 `MotionModel` 的加速度约束逻辑：

$$v\_{\text{new}} = v\_{\text{prev}} + \text{clamp}(u - v\_{\text{prev}}, -a\_{\max} \Delta t, a\_{\max} \Delta t)$$

阿克曼模型额外约束：

$$\text{if } R = \left|\frac{v\_x}{\omega\_z}\right| < R\_{\min}: \quad \omega\_z = \text{sign}(\omega\_z) \cdot \frac{|v\_x|}{R\_{\min}}$$

#### optimizer.hpp — 优化器核心

`Optimizer` 类是 MPPI 算法的核心，执行流程：

```
evalControl()
  ├── prepare()                    # 更新位姿、裁剪路径
  ├── [迭代循环]
  │   ├── generateNoisedTrajectories()  # 采样+前向仿真
  │   │   ├── setNoisedControls()       # 叠加噪声
  │   │   ├── applyControlConstraintsBatch()  # 速度边界约束
  │   │   ├── applyConstraints()        # 运动学约束（如阿克曼）
  │   │   ├── updateStateVelocities()   # 加速度约束
  │   │   ├── integrateStateVelocities() # 多线程轨迹积分
  │   │   └── generateNextNoises()      # 触发异步噪声生成
  │   ├── findFurthestReachedPathPoint() # 查找最远可达路径点
  │   ├── evalTrajectoriesScores()      # 各 Critic 评分
  │   ├── updateControlSequence()       # Softmax 加权更新
  │   └── fallback()                    # 碰撞恢复
  ├── isBlocked()                  # 阻塞检测
  ├── savitskyGolayFilter()        # SG 滤波
  └── getControlFromSequence()     # 取第一个控制量
```

#### controller.hpp — 控制器接口

`MPPIController` 类封装优化器和代价函数管理器，提供：

- `initialize()`：初始化控制器，创建所有 Critic
- `updateStaticObstacles()`：更新障碍物 + 路径有效性 + 运动模型切换
- `computeVelocityCommands()`：主循环调用入口
- 全向/差速自动切换逻辑

### 5.5 ROS1 集成层

#### mppi\_ros1\_node.cpp — ROS1 主节点

`MPPIRos1Node` 类实现与 ROS1 系统的交互：

- **订阅话题**：`plan`（路径）、`odom`（里程计）、`scan`（激光扫描）
- **发布话题**：`cmd_vel`（速度指令）、`debug_optimal_trajectory`（调试轨迹）
- **控制循环**：50ms 定时器（20Hz）
- **参数加载**：从 ROS 参数服务器读取所有 MPPI 参数
- **启动辅助**：`tryApplyStartupAssist()` — 解决零速度启动困境
- **坐标变换**：将激光点从机器人坐标系转换到全局坐标系
- **异常处理**：MPPI 计算失败时输出零速度并重置控制器
- **到达判断**：距目标点 < 0.20m 时停止

***

## 6. 安装与编译

### 6.1 前提条件

- ROS1 Melodic/Noetic
- C++17 兼容编译器（GCC ≥ 7）
- xtensor、xsimd、xtl 库

### 6.2 下载第三方库(仓库的无效就重新下载)

在 `mppi_laser_example` 目录下执行：

```bash
# 进入包目录
cd src/mppi_laser_example

# 下载 xtl（xtensor 基础库，需先安装）
git clone https://github.com/xtensor-stack/xtl.git
cd xtl && git checkout 0.7.0 && cd ..

# 下载 xsimd（SIMD 加速库）
git clone https://github.com/xtensor-stack/xsimd.git
cd xsimd && git checkout 7.4.10 && cd ..

# 下载 xtensor（主数值库，version 0.21.0）
git clone https://github.com/xtensor-stack/xtensor.git
cd xtensor && git checkout 0.21.0 && cd ..

# 返回工作空间根目录
cd ../..
```

三个库均为头文件-only 库，下载后直接通过 CMakeLists.txt 中的 `include_directories` 引用，无需额外编译安装。

### 6.3 编译步骤

```bash
# 1. 进入工作空间
cd ~/MPPI_ws_ros1sim

# 2. 编译
catkin_make

# 3. source 工作空间
source devel/setup.bash
```

### 6.4 依赖说明

ROS1 依赖通过 package.xml 声明：

- roscpp, rospy, nav\_msgs, geometry\_msgs, std\_msgs, sensor\_msgs, tf

***

## 7. 使用方法

### 7.1 启动控制器

```bash
# 终端1：启动 MPPI 控制器
rosrun mppi_laser_example mppi_laser_example_node _params_file:=src/mppi_laser_example/config/mppi_params.yaml

# 或使用 rosparam 加载参数后启动
rosparam load src/mppi_laser_example/config/mppi_params.yaml /mppi_ros1_node
rosrun mppi_laser_example mppi_laser_example_node
```

### 7.2 发布全局路径

```bash
# 终端2：发布路径（支持多种路径类型）
rosrun mppi_laser_example mppi_path_publisher.py
```

路径发布器命令：

- `s` — 切换到直线路径（25m）
- `t` — 切换到三边矩形路径（总长 40m）
- `a` — 切换到圆弧-直线组合路径
- `p` — 打印路径信息

### 7.3 可视化

```bash
# 终端3：在 RViz 中可视化
rviz -d src/mppi_laser_example/config/mppi_test.rviz
```

### 7.4 完整运行流程

```bash
# 1. 编译
cd ~/MPPI_ws_ros1sim && catkin_make && source devel/setup.bash

# 2. 启动仿真环境（根据你的机器人仿真配置）
roslaunch your_robot_sim launch_file.launch

# 3. 加载 MPPI 参数
rosparam load src/mppi_laser_example/config/mppi_params.yaml /mppi_ros1_node

# 4. 启动 MPPI 控制器
rosrun mppi_laser_example mppi_laser_example_node

# 5. 发布全局路径
rosrun mppi_laser_example mppi_path_publisher.py

# 6. 可视化
rviz -d src/mppi_laser_example/config/mppi_test.rviz

或者按照该项目的流程：
ROS1：MPPI
1、
source devel/setup.bash
# 步骤1：加载YAML到参数服务器
rosparam load $(rospack find mppi_laser_example)/config/mppi_params.yaml
# 步骤2：启动节点（节点会自动读取参数）
rosrun mppi_laser_example mppi_ros1_node

2、发布路径
rosrun mppi_laser_example mppi_path_publisher_fz.py _path_type:=three_side_rectangle _path_points:=300
或
rosrun mppi_laser_example JZJ_path.py _path_type:=arc_line
或
rosrun mppi_laser_example clean_path.py

3、rosrun rviz rviz -d /home/shaoyu/MPPI_ws_ros1sim/src/mppi_laser_example/config/mppi_test.rviz

4、（差速/全向）
roslaunch pioneer_utils bizhang.launch

静态TF变换或者启动SLAM节点
rosrun tf static_transform_publisher 0 0 0 0 0 0 map odom 100
```

***

## 8. 参数配置

所有参数在 `config/mppi_params.yaml` 中配置，每个参数均有详细中文注释。以下按类别列出：

### 8.1 MPPI 核心参数

| 参数                         | 默认值   | 描述                                    |
| -------------------------- | ----- | ------------------------------------- |
| `batch_size`               | 1500  | 轨迹样本数，影响优化精度和计算速度                     |
| `time_steps`               | 40    | 预测时间步数，预测时长 = time\_steps × model\_dt |
| `model_dt`                 | 0.05  | 时间步长（秒），控制轨迹预测的时间精度                   |
| `temperature`              | 0.2   | Softmax 温度，控制权重分布的尖锐程度                |
| `adaptive_temperature`     | True  | 是否启用自适应温度                             |
| `adaptive_temperature_min` | 0.1   | 自适应温度下限                               |
| `adaptive_temperature_max` | 1.0   | 自适应温度上限                               |
| `gamma`                    | 0.015 | 控制代价权重，平衡平滑性和轨迹质量                     |
| `iteration_count`          | 1     | 每周期优化迭代次数                             |
| `control_period_ms`        | 50    | 控制周期（毫秒），50ms = 20Hz                  |
| `thread_count`             | 4     | 并行计算线程数                               |
| `prune_distance`           | 3.5   | 路径裁剪距离（米）                             |

### 8.2 控制约束

| 参数       | 默认值   | 描述                     |
| -------- | ----- | ---------------------- |
| `vx_max` | 1.2   | 最大线速度（m/s）             |
| `vx_min` | -0.25 | 最小线速度（m/s），0.0 表示不允许倒车 |
| `vy_max` | 1.2   | 最大横向速度（m/s），仅全向模型有效    |
| `wz_max` | 2.0   | 最大角速度（rad/s）           |
| `ax_max` | 1.5   | 最大线加速度（m/s²）           |
| `ay_max` | 1.35  | 最大横向加速度（m/s²）          |
| `az_max` | 2.5   | 最大角加速度（rad/s²）         |

### 8.3 噪声采样标准差

| 参数       | 默认值  | 描述                        |
| -------- | ---- | ------------------------- |
| `vx_std` | 0.20 | 线速度噪声标准差（m/s）             |
| `vy_std` | 0.20 | 横向速度噪声标准差（m/s），差速模型设为 0.0 |
| `wz_std` | 0.30 | 角速度噪声标准差（rad/s）           |

### 8.4 障碍物代价参数

| 参数                            | 默认值      | 描述            |
| ----------------------------- | -------- | ------------- |
| `obstacle_repulsion_weight`   | 1.0      | 障碍物排斥权重       |
| `obstacle_collision_cost`     | 100000.0 | 碰撞轨迹代价        |
| `obstacle_collision_margin`   | 0.3      | 碰撞边界距离（m）     |
| `collision_cost_threshold`    | 50000.0  | 碰撞代价阈值        |
| `obstacle_inflation_radius`   | 0.8      | 障碍物膨胀半径（m）    |
| `obstacle_cost_scaling`       | 4.0      | 代价缩放因子        |
| `obstacle_near_goal_distance` | 0.3      | 接近目标时忽略障碍物的距离 |
| `robot_radius`                | 0.25     | 机器人半径（m）      |
| `consider_footprint`          | false    | 是否考虑机器人足迹     |
| `grid_resolution`             | 0.05     | 距离场分辨率（m/栅格）  |
| `grid_width`                  | 100      | 栅格地图宽度        |
| `grid_height`                 | 100      | 栅格地图高度        |

### 8.5 阻塞检测参数

| 参数                         | 默认值 | 描述                        |
| -------------------------- | --- | ------------------------- |
| `twirling_weight`          | 0.5 | 旋转惩罚权重（软约束）               |
| `twirling_threshold`       | 0.5 | 关闭 TwirlingCritic 的目标距离阈值 |
| `spinning_ratio_threshold` | 8.0 | wz/vx 比率阈值（硬约束）           |
| `spinning_detect_frames`   | 8   | 连续检测帧数                    |

### 8.6 启动辅助参数

| 参数                       | 默认值   | 描述          |
| ------------------------ | ----- | ----------- |
| `startup_assist_enabled` | false | 是否启用启动辅助    |
| `startup_boost_vx`       | 0.06  | 启动时的线速度增强值  |
| `startup_boost_wz_gain`  | 1.2   | 启动时的角速度增益系数 |
| `startup_boost_wz_limit` | 0.6   | 启动时的角速度限制   |
| `startup_cmd_vx_eps`     | 0.01  | 启动命令速度阈值    |
| `startup_speed_vx_eps`   | 0.02  | 启动实际速度阈值    |
| `startup_goal_distance`  | 0.30  | 启动目标距离阈值    |
| `startup_lookahead`      | 8     | 启动时的前瞻步数    |

### 8.7 路径对齐代价参数

| 参数                                 | 默认值   | 描述            |
| ---------------------------------- | ----- | ------------- |
| `path_align_weight`                | 15.0  | 路径对齐权重        |
| `path_align_offset`                | 4     | 从最远路径点向前偏移的步数 |
| `path_align_threshold`             | 0.3   | 距离阈值（m）       |
| `path_align_traj_step`             | 1     | 轨迹点采样步长       |
| `path_align_obstacle_check_radius` | 0.15  | 路径对齐障碍物检查半径   |
| `path_align_max_occupancy_ratio`   | 0.45  | 路径阻塞比例阈值      |
| `path_align_use_orientations`      | false | 是否考虑路径朝向      |

### 8.8 路径角度代价参数

| 参数                     | 默认值  | 描述                         |
| ---------------------- | ---- | -------------------------- |
| `path_angle_weight`    | 5.0  | 路径角度权重                     |
| `path_angle_offset`    | 4    | 从最远路径点向前偏移的步数              |
| `path_angle_threshold` | 0.3  | 距离阈值（m）                    |
| `path_angle_max`       | 0.15 | 最大允许角度偏差（rad）              |
| `path_angle_mode`      | 0    | 角度模式：0=偏好前进，1=无偏好，2=考虑路径朝向 |

### 8.9 路径跟随代价参数

| 参数                      | 默认值 | 描述            |
| ----------------------- | --- | ------------- |
| `path_follow_weight`    | 8.0 | 路径跟随权重        |
| `path_follow_offset`    | 4   | 从最远路径点向前偏移的步数 |
| `path_follow_threshold` | 0.4 | 距离阈值（m）       |

### 8.10 目标点代价参数

| 参数                     | 默认值 | 描述            |
| ---------------------- | --- | ------------- |
| `goal_weight`          | 5.0 | 目标点权重         |
| `goal_threshold`       | 0.8 | 激活目标点代价的距离阈值  |
| `goal_angle_weight`    | 3.0 | 目标角度权重        |
| `goal_angle_threshold` | 0.6 | 激活目标角度代价的距离阈值 |

### 8.11 其他代价参数

| 参数                         | 默认值  | 描述                     |
| -------------------------- | ---- | ---------------------- |
| `prefer_forward_weight`    | 3.0  | 偏好前进权重                 |
| `prefer_forward_threshold` | 0.5  | 距离阈值                   |
| `constraint_weight`        | 3.0  | 约束代价权重                 |
| `motion_model_type`        | 0    | 运动模型类型：0=差速，1=全向，2=阿克曼 |
| `velocity_deadband_weight` | 0.5  | 速度死区权重                 |
| `velocity_deadband_vx`     | 0.05 | 线速度死区阈值                |
| `velocity_deadband_vy`     | 0.05 | 横向速度死区阈值               |
| `velocity_deadband_wz`     | 0.1  | 角速度死区阈值                |

### 8.12 SG 滤波器参数

| 参数                       | 默认值   | 描述              |
| ------------------------ | ----- | --------------- |
| `use_sg_filter`          | true  | 是否启用 SG 滤波器     |
| `shift_control_sequence` | false | 是否启用控制序列时序平滑    |
| `retry_attempt_limit`    | 2     | fallback 重试次数限制 |
| `use_mean_normalization` | false | 是否使用均值归一化       |

### 8.13 运动模型参数

| 参数                             | 默认值         | 描述                            |
| ------------------------------ | ----------- | ----------------------------- |
| `motion_model`                 | "DiffDrive" | 运动模型：DiffDrive/Omni/Ackermann |
| `ackermann_min_turning_radius` | 0.3         | 阿克曼最小转弯半径（m）                  |

### 8.14 全向/差速自动切换参数

| 参数                            | 默认值  | 描述             |
| ----------------------------- | ---- | -------------- |
| `enable_omni_switching`       | true | 是否启用全向/差速自动切换  |
| `omni_trigger_obstacle_dist`  | 0.9  | 触发全向模式的障碍物距离阈值 |
| `omni_trigger_path_deviation` | 0.3  | 触发全向模式的路径偏离阈值  |
| `diff_restore_path_threshold` | 0.20 | 恢复差速模式的路径偏离阈值  |
| `omni_switch_delay_frames`    | 3    | 模式切换延迟帧数       |

### 8.15 差速模式配置要点

切换到差速模式时，需要修改以下关键参数：

```yaml
motion_model: "DiffDrive"          # 运动模型设为差速
motion_model_type: 0               # 约束代价中的模型类型
vx_min: -0.0                       # 不允许倒车（或设为负值允许）
vy_std: 0.0                        # 横向速度噪声设为 0
enable_omni_switching: false       # 关闭全向切换（纯差速模式）魔改部分
consider_footprint: true           # 建议开启足迹检测
robot_radius: 0.35                 # 根据实际机器人尺寸调整
```

***

## 9. 话题与接口

### 9.1 订阅的话题

| 话题      | 类型                      | 描述           |
| ------- | ----------------------- | ------------ |
| `/plan` | `nav_msgs/Path`         | 要跟随的全局路径     |
| `/odom` | `nav_msgs/Odometry`     | 机器人位姿和速度     |
| `/scan` | `sensor_msgs/LaserScan` | 用于障碍物检测的激光扫描 |

### 9.2 发布的话题

| 话题                          | 类型                    | 描述                |
| --------------------------- | --------------------- | ----------------- |
| `/cmd_vel`                  | `geometry_msgs/Twist` | 输出速度命令            |
| `/debug_optimal_trajectory` | `nav_msgs/Path`       | 最优轨迹（用于 RViz 可视化） |

### 9.3 核心 C++ 接口

```cpp
// 初始化控制器
mppi::MPPIController controller;
controller.initialize(settings, "DiffDrive", ackermann_radius);

// 设置各 Critic 参数
controller.getObstaclesCritic()->setParams(...);
controller.getPathAlignCritic()->setParams(...);
// ...

// 控制循环
controller.setPath(path_poses);
controller.updateStaticObstacles(laser_points, robot_pose);
mppi::Twist2D cmd = controller.computeVelocityCommands(robot_pose, robot_speed);
```

***

## 10. 算法细节

### 10.1 控制循环流程

```
1. 接收激光扫描 → 转换到全局坐标系
2. 接收里程计 → 更新机器人位姿和速度
3. 接收全局路径 → 存储路径点
4. 定时器触发（50ms）：
   a. 检查是否到达目标（< 0.20m）
   b. 更新障碍物 + 路径有效性 + 运动模型切换
   c. 计算速度指令：
      i.  裁剪路径（以机器人为中心，保留 prune_distance 内的点）
      ii. 迭代优化：
          - 叠加噪声生成 K 条候选轨迹
          - 多线程前向仿真
          - 各 Critic 评分
          - Softmax 加权更新控制序列
      iii. 碰撞检测与恢复
      iv.  阻塞检测
      v.   SG 滤波
      vi.  取第一个控制量
   d. 启动辅助（可选）
   e. 发布速度指令
   f. 发布调试轨迹
```

### 10.2 路径裁剪

Nav2 风格的路径裁剪，使索引 0 对应机器人当前位置：

1. 找到机器人当前位置在路径上的最近点
2. 从最近点开始，沿路径累计距离，保留 `prune_distance` 内的点
3. 创建裁剪后的路径

### 10.3 最远可达路径点

查找所有采样轨迹终点在路径上对应的最近点，取最大索引作为最远可达路径点。该索引用于确定 PathFollowCritic、PathAlignCritic、PathAngleCritic 的参考点位置。

### 10.4 碰撞恢复机制

当所有轨迹都碰撞时：

1. 设置 `fail_flag = true`
2. 尝试 `fallback()`：重置控制序列，重新采样
3. 最多重试 `retry_attempt_limit` 次
4. 超过重试次数后抛出异常，ROS1 节点捕获后输出零速度

### 10.5 全向/差速自动切换

切换条件：

| 方向    | 条件                                                                                | 动作                  |
| ----- | --------------------------------------------------------------------------------- | ------------------- |
| 差速→全向 | 最近障碍物 < `omni_trigger_obstacle_dist` 或 路径偏离 > `omni_trigger_path_deviation`       | 切换到 OmniMotionModel |
| 全向→差速 | 最近障碍物 > `omni_trigger_obstacle_dist × 1.2` 且 路径偏离 < `diff_restore_path_threshold` | 恢复基础运动模型            |

切换时执行：

1. `optimizer_->softReset()`：重置控制序列
2. 创建新的运动模型
3. `optimizer_->setMotionModel()`：更新运动模型
4. 更新 ConstraintCritic 参数

延迟机制：连续 `omni_switch_delay_frames` 帧满足条件才执行切换，防止频繁切换。

### 10.6 启动辅助

当机器人速度接近零且无障碍物时，根据路径方向施加初始速度：

$$v\_x = v\_{\text{boost}}, \quad \omega\_z = \text{clamp}(K\_p \cdot \Delta\theta, -\omega\_{\max}, \omega\_{\max})$$

***

## 11. 性能优化

### 11.1 异步噪声预生成

噪声生成使用独立线程，与主控制循环并行执行：

```
时间线:
主线程:   [使用噪声A] → [使用噪声A评分] → [使用噪声B] → [使用噪声B评分] → ...
噪声线程: [生成噪声A] → [生成噪声B] → [生成噪声C] → ...
```

当主线程使用当前噪声进行评分时，噪声线程已经在预生成下一组噪声，减少了主循环的等待时间。

### 11.2 多线程轨迹积分

轨迹积分使用 `std::thread` 多线程并行计算：

```cpp
// 将 batch_size 条轨迹分配到 thread_count 个线程
unsigned int batch_per_thread = (batch_size + thread_count - 1) / thread_count;
for (unsigned int t = 0; t < thread_count; ++t) {
    threads.emplace_back(worker, start, end);
}
```

每个线程独立计算一批轨迹的前向仿真，避免数据竞争。

### 11.3 栅格距离场 O(1) 查询

以机器人为中心构建局部 BFS 距离场后，轨迹评分时只需通过坐标映射直接查询距离，时间复杂度 O(1)，避免了暴力搜索所有障碍物点。

### 11.4 xt::noalias 优化

在噪声叠加等操作中使用 `xt::noalias` 跳过别名检查，提升 xtensor 的赋值性能：

```cpp
xt::noalias(state.cvx) = xt::view(control_sequence.vx, xt::newaxis(), xt::all()) + noises_vx_;
```

### 11.5 路径累计距离缓存

PathAlignCritic 使用路径累计距离匹配代替暴力搜索，通过单调递增的距离序列实现提前终止：

```cpp
for (size_t i = start_idx + 1; i < path_distances.size(); ++i) {
    float diff = std::abs(path_distances[i] - target_distance);
    if (diff < min_diff) { min_diff = diff; closest_idx = i; }
    else if (diff > min_diff) { break; }  // 提前终止
}
```

***

## 12. 与 Nav2 官方 MPPI 控制器的区别

### 12.1 框架依赖

| 特性      | 本项目               | Nav2 MPPI                                    |
| ------- | ----------------- | -------------------------------------------- |
| ROS2 依赖 | 无                 | 强依赖（rclcpp, nav2\_core, nav2\_costmap\_2d 等） |
| ROS1 支持 | 原生支持              | 暂未支持                                         |
| 参数系统    | 外部通过 setParams 设置 | ROS2 参数服务器 + ParametersHandler               |
| 插件机制    | 编译时静态注册           | pluginlib 动态加载                               |
| 生命周期管理  | 无                 | LifecycleNode 状态机                            |
| 代码组织    | 模块化多文件            | 插件内联实现                                       |

### 12.2 障碍物处理

| 特性    | 本项目                       | Nav2 MPPI                                    |
| ----- | ------------------------- | -------------------------------------------- |
| 障碍物来源 | 激光点云（直接输入）                | Costmap2D 全局代价地图                             |
| 距离查询  | 自建局部栅格 BFS 距离场，可接入 RC-ESDF 精确距离查询 | Costmap2D 膨胀层 + FootprintCollisionChecker    |
| 动态障碍物 | 内置线性预测支持                  | 无（依赖代价地图更新）                                  |
| 足迹检测  | 自实现坐标变换 + 栅格查询            | nav2\_costmap\_2d::FootprintCollisionChecker |

### 12.3 数值库

| 特性    | 本项目               | Nav2 MPPI                 |
| ----- | ----------------- | ------------------------- |
| 矩阵运算  | xtensor           | Eigen                     |
| 随机数生成 | xt::random::randn | std::normal\_distribution |

### 12.4 算法增强\*一些魔改

本项目在 Nav2 MPPI 基础上增加的功能：

| 增强功能          | 描述                               | Nav2 状态     |
| ------------- | -------------------------------- | ----------- |
| **阻塞检测机制**    | 通过 wz/vx 比率和连续帧检测，识别原地打转并输出零速度   | 无此机制        |
| **全向/差速自动切换** | 根据障碍物距离和路径偏离自动切换运动模型             | 需手动配置       |
| **自适应温度**     | 根据代价分布范围动态调整 softmax 温度          | 使用固定温度      |
| **均值归一化选项**   | 支持均值归一化替代 min 归一化                | 仅 min 归一化   |
| **路径点有效性检查**  | PathFollowCritic 可跳过被障碍物阻挡的路径点   | 基础实现        |
| **路径阻塞检测**    | PathAlignCritic 在路径被大量阻塞时自动失效    | 基础实现        |
| **动态障碍物预测**   | ObstaclesCritic 支持基于速度的动态障碍物位置预测 | 无（依赖代价地图更新） |
| **启动辅助**      | 解决零速度启动困境/代码冗余/无效                        | 无此功能        |
| **任意形状机器人避碰**      | 结合 RC-ESDF 构建分区碰撞检测，风险区用解析梯度进行采样偏置                        | 无此功能        |


### 12.5 Critic 差异

| Critic                 | 本项目           | Nav2 MPPI  |
| ---------------------- | ------------- | ---------- |
| CostCritic             | 无             | 有（基于代价地图值） |
| ObstaclesCritic        | BFS栅格距离场 + RC-ESDF分区碰撞检测 + 动态预测  | 代价地图膨胀层查询  |
| PathAlignCritic        | 路径阻塞检测 + 朝向考虑 | 基础实现       |
| PathAngleCritic        | 三种角度模式        | 三种角度模式     |
| VelocityDeadbandCritic | 有             | 有          |
| TwirlingCritic         | 有（含目标距离关闭逻辑）  | 有          |

### 12.6 接口差异

| 特性   | 本项目                                                     | Nav2 MPPI                                                             |
| ---- | ------------------------------------------------------- | --------------------------------------------------------------------- |
| 主接口  | `computeVelocityCommands(Pose2D, Twist2D)` 返回 `Twist2D` | `computeVelocityCommands(PoseStamped, Twist, Path)` 返回 `TwistStamped` |
| 路径处理 | 集成在 `Optimizer::prunePath()` 中                          | 由独立的 PathHandler 节点完成                                                 |
| 参数设置 | 通过 `setParams()` 方法                                     | 通过 ROS2 参数服务器                                                         |

### 12.7 线程模型

| 特性   | 本项目               | Nav2 MPPI                        |
| ---- | ----------------- | -------------------------------- |
| 噪声生成 | 独立线程 + xt::random | 独立线程 + std::normal\_distribution |
| 轨迹积分 | std::thread 多线程并行 | Eigen 向量化（单线程）                   |

### 12.8 代码组织差异

| 特性   | 本项目          | Nav2 MPPI       |
| ---- | ------------ | --------------- |
| 职责划分 | 每个文件单一职责     | 多个类内联在同一文件，插件制  |
| 可维护性 | 高（模块独立，接口清晰） | 中（看理解程度）     |
| 编译依赖 | 仅 xtensor    | Eigen + Nav2 全栈 |

***

## 13. 许可证

Copyright © 2025 MPPI ROS1 Project

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.

***

## 14. 参考内容

### 14.1 MPPI 算法论文

- Williams G, Drews P, Goldfain B, Rehg J M, Theodorou E A. **Information-Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving**\[J]. IEEE Transactions on Robotics, 2018, 34(6): 1603-1622.
- Williams G, Aldrich A, Theodorou E A. **Model Predictive Path Integral Control: From Theory to Parallel Computation**\[J]. Journal of Guidance, Control, and Dynamics, 2017, 40(2): 344-357.

### 14.2 Nav2 MPPI 控制器

- Nav2 官方文档: <https://docs.nav2.org/>
- Nav2 MPPI Controller 源码: <https://github.com/ros-planning/navigation2/tree/main/nav2_mppi_controller>

### 14.3 第三方库

- xtensor（头文件-only 多维数组库）: <https://github.com/xtensor-stack/xtensor>
- xsimd（C++ SIMD 加速库）: <https://github.com/xtensor-stack/xsimd>
- xtl（xtensor 基础模板库）: <https://github.com/xtensor-stack/xtl>

### 14.4 相关博客

- [MPPI 局部路径规划控制器 —— 从 Nav2 解耦的模块化 ROS1 实现](https://blog.csdn.net/qq_56908984/article/details/158774722?sharetype=blogdetail\&sharerId=158774722\&sharerefer=PC\&sharesource=qq_56908984\&spm=1011.2480.3001.8118)
