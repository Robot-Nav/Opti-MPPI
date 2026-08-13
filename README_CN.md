# DC-PMPPI 扰动补偿模型预测路径积分控制器

![License](https://img.shields.io/badge/License-Apache--2.0-blue.svg)
![Language](https://img.shields.io/badge/Language-C%2B%2B17-orange.svg)
![ROS](https://img.shields.io/badge/ROS-Noetic-22314E.svg)
![Build](https://img.shields.io/badge/Build-catkin-green.svg)
![Parallel](https://img.shields.io/badge/Parallel-OpenMP-red.svg)
![Vectorization](https://img.shields.io/badge/Vectorization-xtensor%20%2B%20xsimd-yellow.svg)
![Platform](https://img.shields.io/badge/Platform-Linux-lightgrey.svg)
![Status](https://img.shields.io/badge/Status-Release-brightgreen.svg)

> 一个面向湿滑路面、甲板晃动等低附着/外部扰动工况的 ROS1 局部路径跟踪控制器。
> 在标准 MPPI 采样优化框架之上引入等价扰动估计与控制有效性辨识，使预测模型与被控对象保持一致，从而在保持计算实时性的前提下显著改善滑移条件下的轨迹跟踪精度。

---

## 目录

- [一、项目含义](#一项目含义)
- [二、算法原理](#二算法原理)
  - [2.1 MPPI 控制框架回顾](#21-mppi-控制框架回顾)
  - [2.2 DC-PMPPI 的核心改进](#22-dc-pmppi-的核心改进)
  - [2.3 系统整体数据流](#23-系统整体数据流)
- [三、算法公式](#三算法公式)
  - [3.1 路径积分控制律](#31-路径积分控制律)
  - [3.2 含扰动的 Rollout 动力学](#32-含扰动的-rollout-动力学)
  - [3.3 在线扰动估计](#33-在线扰动估计)
  - [3.4 控制有效性辨识](#34-控制有效性辨识)
  - [3.5 MPCC 式跟踪代价](#35-mpcc-式跟踪代价)
  - [3.6 自适应温度与精英截断](#36-自适应温度与精英截断)
  - [3.7 运动学约束](#37-运动学约束)
- [四、项目内容](#四项目内容)
  - [4.1 目录结构](#41-目录结构)
  - [4.2 核心模块说明](#42-核心模块说明)
  - [4.3 关键参数解读](#43-关键参数解读)
- [五、运行步骤](#五运行步骤)
- [六、项目依赖库](#六项目依赖库)
- [七、扩展与消融实验](#七扩展与消融实验)

---

## 一、项目含义

`dc_pmppi` 是 **Disturbance-Compensated Model Predictive Path Integral Control** 的缩写，是面向轮式移动机器人（差速、全向、阿克曼）的局部路径跟踪控制器。

标准 MPPI 通过对控制序列进行高斯采样、Rollout、加权平均获得最优控制指令，具有无需代价函数可微、天然处理非线性模型与非凸约束的优势。但其隐含假设是 **预测模型与真实被控对象一致**。一旦该假设被打破（例如：

- 雨雪/湿滑路面下轮胎侧偏刚度下降，指令速度与实际速度不再相等；
- 舰载或船载机器人在晃动甲板上作业，车体存在外部强迫运动；
- 里程计与底盘存在系统性偏置，等效为一个持续作用的外部扰动；

），MPPI 的预测轨迹会与真实轨迹产生系统性偏差，控制律无法在 horizon 内主动修正，最终表现为路径偏置、超调甚至失稳。

DC-PMPPI 的工程目标是：

1. 在线辨识车体系下的等价侧向滑移速度 `v_y^dist`、横摆扰动角速度 `ω^dist`，以及纵向/横摆控制有效性系数 `η_v, η_ω`；
2. 将这些估计量注入 MPPI 的 Rollout 模型、参考控制生成与代价函数中，使预测轨迹更贴近真实被控对象；
3. 通过置信度加权与扰动衰减时间常数，保证估计瞬态、噪点、丢帧不会反向污染控制律。

`vy_dist = 0, wz_dist = 0` 时，DC-PMPPI 自动退化为标准 MPPI，便于做 A/B 对比实验。

---

## 二、算法原理

### 2.1 MPPI 控制框架回顾

MPPI 将最优控制问题转化为路径积分形式。设离散动力学：

```
x_{t+1} = F(x_t, u_t)
```

代价函数：

```
S(x_{0:T}, u_{0:T}) = Σ_{t=0}^{T-1} q(x_t, u_t) + φ(x_T)
```

控制律由采样轨迹的指数加权平均给出（Williams et al., 2017）：

```
u* = Σ_i w_i · u_i,   w_i = exp(-1/λ · (S_i - min S))
```

每个控制周期完成 **采样 → Rollout → 评分 → 加权更新 → 序列左移** 五个步骤，并通过热启动（warm start）保持时序平滑。

### 2.2 DC-PMPPI 的核心改进

本项目在标准 MPPI 之上引入五项关键改进：

**(1) 等价扰动估计**
通过比较里程计位姿差分速度与已执行指令，估计车体系下的侧向残差 `v_y^dist` 和横摆残差 `ω^dist`，作为模型失配的统一表征。这些残差融合了侧滑、外部强迫运动、定位偏置等多种物理因素，避免对每种扰动单独建模。

**(2) 控制有效性 η_v / η_ω**
湿滑路面下指令速度与实际响应之间存在增益衰减。引入：

```
v_actual = η_v · v_cmd,   ω_actual = η_ω · ω_cmd
```

通过指数加权 Welford 回归在线辨识 η，将控制指令的"打折扣"行为显式写入 Rollout 模型，避免在代价函数中重复惩罚。

**(3) 扰动衰减注入**
扰动不是常值，未来时刻的影响按时间常数 τ 衰减：

```
d_t = d_0 · exp(-t / τ)
```

使 Rollout 在 horizon 远端逐渐回归名义模型，避免长时扰动估计误差累积放大。

**(4) 自适应温度 + ESS 目标**
MPPI 的温度参数 λ 控制采样的探索-利用权衡。固定 λ 在不同代价尺度下表现差异巨大。本项目以 **目标有效样本量（Effective Sample Size, ESS）** 为目标，通过二分搜索调整 λ，使算法对代价函数尺度变化具有鲁棒性。

**(5) 精英截断（CEM-style）+ 反对称采样**
保留 MPPI 在精英集合内的相对代价加权，同时丢弃长尾弱样本；通过对称配对采样（antithetic sampling）降低蒙特卡洛方差。

### 2.3 系统整体数据流

```
┌────────────────┐   /plan    ┌──────────────────┐
│  全局路径规划   │ ─────────> │  pathCallback     │
└────────────────┘            │  · 重采样/去重     │
                              │  · 切线朝向计算     │
┌────────────────┐   /odom1   └────────┬─────────┘
│  里程计/SLAM    │ ─────────┐         │
└────────────────┘          ▼         │
                  ┌──────────────────┐ │
                  │  odomCallback     │ │
                  │  · 位姿/速度缓存   │ │
                  │  · 扰动估计器更新  │ │
                  └────────┬─────────┘ │
                           │           │
                           ▼           ▼
              ┌──────────────────────────────┐
              │      controlLoop (50 ms)      │
              │  1. 目标到达/重捕获判定         │
              │  2. 置信度加权扰动注入          │
              │  3. MPPI 优化器 evalControl    │
              │     · 路径裁剪 + 参考速度预览    │
              │     · 高斯采样 + 引导采样       │
              │     · Rollout（含扰动衰减）     │
              │     · Critic 评分              │
              │     · softmax 加权更新         │
              │     · Savitzky-Golay 平滑      │
              │  4. GoalDocking 终端稳定器     │
              │  5. enforceMotionConstraints  │
              └────────────┬─────────────────┘
                           │
                           ▼
                    /cmd_vel, /local_path,
                    /controller_solve_time_ms
```

---

## 三、算法公式

### 3.1 路径积分控制律

设当前控制序列为 `U = (u_0, u_1, ..., u_{T-1})`，对其添加高斯噪声 `ε_i,t ~ N(0, Σ)` 得到 N 条采样序列 `U_i = U + ε_i`。每条序列的累计代价：

```
S_i = φ(x_T^i) + Σ_{t=0}^{T-1} [ q(x_t^i, u_t^i)
        + γ/2 · u_t^iᵀ · Σ⁻¹ · ε_{i,t} ]
```

其中 γ 是控制代价折扣因子，第二项是 MPPI 的"控制能量"正则项，避免无谓的高频抖动。

控制序列按信息论最优更新律更新：

```
U ← U + Σ_i w_i · ε_i,   w_i = (1/Z) · exp(-1/λ · (S_i - S_min))
```

`Z = Σ_i exp(-1/λ · (S_i - S_min))` 为归一化常数。本项目支持：

- `use_mean_normalization`：用代价均值代替 min 做参考点；
- `elite_sample_count > 0`：仅保留 Top-K 样本参与加权（CEM 截断）；
- `elite_update_rate`：精英均值更新步长，控制收敛速度。

### 3.2 含扰动的 Rollout 动力学

对差速/阿克曼等非完整约束模型，标准 Rollout 为：

```
x_{t+1} = x_t + (v·cosθ)·dt
y_{t+1} = y_t + (v·sinθ)·dt
θ_{t+1} = θ_t + ω·dt
```

DC-PMPPI 注入侧向滑移与横摆扰动：

```
θ_{t+1} = θ_t + (ω_cmd·η_ω + ω^dist·e^{-t/τ})·dt
x_{t+1} = x_t + (v·cosθ - v_y^dist·e^{-t/τ}·sinθ)·dt
y_{t+1} = y_t + (v·sinθ + v_y^dist·e^{-t/τ}·cosθ)·dt
```

控制有效性 η 在运动模型预测层引入：

```
v_realized = η_v · v_cmd,   ω_realized = η_ω · ω_cmd
```

随后仍施加加速度约束 `|v_realized - v_prev| ≤ a_max·dt`，保证物理可行性。

### 3.3 在线扰动估计

设位姿差分得到的车体系纵向/侧向/横摆速度为 `v_x^meas, v_y^meas, ω^meas`，已执行指令经一阶滞后补偿后为 `v_x^cmd, ω^cmd`。横摆预期值：

```
ω^exp = η_ω · ω^cmd
```

扰动残差：

```
ω^dist_obs = ω^meas - ω^exp
v_y^dist_obs = v_y^meas
```

通过指数低通滤波（时间常数 `τ_f`）：

```
v_y^dist ← β·v_y^dist + (1-β)·clip(v_y^dist_obs - v_y^dist, [-Δ_max, Δ_max])
```

其中 `β = exp(-dt/τ_f)`，`Δ_max` 是单步创新限幅，防止定位跳变污染估计。

方差同步递推：

```
σ²_vy ← β·σ²_vy + (1-β)·(Δ_vy)²
```

用于可选的扰动不确定性 sigma-point 采样。

### 3.4 控制有效性辨识

采用指数加权 Welford 中心化回归（可选）或简单比值法：

**比值法**（默认）：

```
η_v_obs = clip(v_x^meas / v_x^cmd, 0.3, 1.0)   if |v_x^cmd| > deadzone
η_ω_obs = clip(ω^meas / ω^cmd, 0.3, 1.0)         if |ω^cmd| > deadzone
```

**中心化回归法**：

```
cov(v_cmd, v_meas) / var(v_cmd)
```

中心化可剔除慢变加性扰动对斜率估计的污染。

η 经慢速低通（时间常数 `τ_η`，约 1.5s）融合，避免短时漂移误判为附着力下降。

**附着能力指标** `traction_scale`：

```
severity = k_1·|v_y^dist| + k_2·|ω^dist| + 0.2·(σ_vy + σ_ω)
traction_scale = clip(1 - severity, τ_min, 1.0)
```

`traction_scale` 不是物理摩擦系数，而是 [0,1] 的保守能力指标，用于动态收紧加速度与速度上限：

```
v_max ← v_max · traction_scale^α_v
a_max ← a_max · traction_scale^α_a
```

### 3.5 MPCC 式跟踪代价

采用 Model Predictive Contouring Control 思想，将路径弧长 s 作为虚拟进度状态，对每条 Rollout 轨迹计算：

```
e_c = signed_lateral_error        # 轮廓误差（横向距离）
e_l = s_exp - s_proj              # 滞后误差（进度差）
e_h = wrap(θ_traj - (θ_path - β_slip))   # 航向误差（含滑移角补偿）
e_v = v_traj - v_ref(s)           # 速度跟踪误差
e_ω = (ω_traj + ω^dist·decay) - v_ref·κ  # 横摆角速度误差
```

其中 `β_slip = atan2(v_y^dist·decay, max(v_ref, 0.1))` 是车体侧偏角补偿，让目标航向在滑移下仍指向真实运动方向。

代价：

```
J_i = Σ_t w(t) · (q_c·e_c² + q_l·e_l² + q_h·e_h² + q_v·e_v² + q_ω·e_ω²)
     + w_term · (q_pos·|p_T - p_goal|² + q_yaw·Δθ_goal² + q_vel·v_T²)
```

`w(t) = 1 + t/(T-1)` 为时间加权，强化终端代价。终端项仅在 horizon 覆盖到全局目标时按 `goal_blend` 比例激活。

**参考速度预览**：考虑曲率限速、弯道前视减速、终点制动三重约束：

```
v_ref(s) = smooth_min(v_cruise,
                      sqrt(a_lat_max / |κ|),
                      sqrt(2·a_goal·d_goal),
                      sqrt(v_corner² + 2·a_corner·d_corner))
```

`smooth_min` 是带过渡区的光滑最小算子，避免硬切换造成控制抖动。

### 3.6 自适应温度与精英截断

**ESS 目标温度搜索**：

```
ESS(T) = (Σ w_i)² / Σ w_i²,   w_i = exp(-(S_i - S_min)/T)
```

通过二分搜索找到使 `ESS(T) = target_ess_ratio · N` 的温度 T，约束在 `[T_min, T_max]`。代价尺度变化时温度自动跟随，无需手工调参。

**CEM 精英截断**：保留 Top-K 样本，丢弃长尾；精英集合内仍保留 MPPI 的相对代价加权（不是 CEM 的等权平均）：

```
w_i = 0                      if i not in Top-K
w_i = exp(-(S_i - S_min)/T) / Z   otherwise
```

### 3.7 运动学约束

**差速模型**：直接限制 `|v_x| ≤ v_max`、`|ω| ≤ ω_max`，独立加速度约束 `|Δv_x| ≤ a_x·dt`、`|Δω| ≤ a_z·dt`。

**全向模型**：合成速度 `v = hypot(v_x, v_y)` 受限，`v_y` 独立通道。

**阿克曼模型**：在加速度传播后再次投影到可行域：

```
|ω| ≤ min(|v| / R_min, a_lat_max / |v|)
```

保证预测状态本身也满足最小转弯半径与横向加速度约束，避免启动/制动阶段产生瞬时无限曲率。

---

## 四、项目内容

### 4.1 目录结构

```
dc_pmppi/
├── CMakeLists.txt                      # catkin 构建配置
├── package.xml                         # ROS 包描述与依赖声明
├── config/
│   ├── dc_pmppi_params.yaml            # 控制器全部可调参数
│   └── dc_pmppi_test.rviz              # RViz 可视化配置
├── include/
│   ├── controller.hpp                  # 控制器对外接口类
│   ├── optimizer.hpp                   # MPPI 优化器核心
│   ├── motion_models.hpp               # 三种运动学模型
│   ├── critics/                        # 代价函数集合
│   │   ├── critic_function.hpp         #   代价基类
│   │   ├── critic_manager.hpp          #   代价管理器（含统计）
│   │   ├── critic_data.hpp             #   评分数据传递结构
│   │   ├── mpcc_tracking_critic.hpp    #   MPCC 式跟踪代价（核心）
│   │   ├── control_rate_critic.hpp     #   控制变化率平滑代价
│   │   ├── constraint_critic.hpp       #   运动学约束违反代价
│   │   ├── stability_critic.hpp        #   低附着稳定性代价
│   │   ├── path_align_critic.hpp       #   路径对齐代价
│   │   ├── path_angle_critic.hpp       #   路径角度代价
│   │   ├── path_follow_critic.hpp      #   路径跟随代价
│   │   ├── lateral_error_critic.hpp    #   LQR 最优横向误差代价
│   │   ├── prefer_forward_critic.hpp   #   偏好前进代价
│   │   ├── twirling_critic.hpp         #   原地旋转抑制代价
│   │   └── velocity_deadband_critic.hpp#   速度死区代价
│   ├── models/                         # 数据结构
│   │   ├── types.hpp                   #   Pose2D / Twist2D / Control / DisturbanceEstimate
│   │   ├── constraints.hpp             #   控制约束与采样噪声参数
│   │   ├── state.hpp                   #   采样状态容器
│   │   ├── trajectories.hpp            #   轨迹容器
│   │   ├── control_sequence.hpp        #   控制序列
│   │   ├── path.hpp                    #   全局路径
│   │   └── optimizer_settings.hpp      #   优化器全部设置
│   └── tools/                          # 工具模块
│       ├── math_utils.hpp              #   数学常量与角度/clamp 工具
│       ├── noise_generator.hpp         #   高斯噪声生成（含 AR/KMPPI/反对称）
│       ├── disturbance_estimator.hpp   #   在线扰动与有效性估计器
│       ├── path_projection.hpp         #   点到路径投影与弧长计算
│       ├── path_reference.hpp          #   路径参考状态与速度预览
│       └── goal_docking.hpp            #   终点位姿稳定器
├── launch/
│   ├── j15_dc_pmppi.launch             # 顶层启动（含 RViz/DOT 选项）
│   ├── j15_dc_pmppi_node.launch        # 节点启动
│   ├── j15_path.launch                 # 路径发布
│   └── j15_sim.launch                  # 仿真启动
├── scripts/
│   ├── JZJ_path.py                     # 路径生成脚本
│   └── twist_to_ackermann.py           # Twist → Ackermann 转换
├── src/
│   └── dc_pmppi_node.cpp               # ROS 节点入口
├── test/
│   └── disturbance_estimator_test.cpp  # 扰动估计器单元测试
├── xsimd/                              # 内嵌 SIMD 加速库
└── xtensor/                            # 内嵌张量运算库
```

### 4.2 核心模块说明

#### `controller.hpp` — 控制器对外接口

封装优化器、运动模型、代价管理器三者，对外暴露：

- `initialize(settings, motion_model_type, ackermann_radius)`：根据字符串选择运动模型，注册默认 Critic；
- `setPath(path)`：更新全局路径，自动检测路径变化并触发优化器重置；
- `setDisturbanceEstimate(...)`：注入外部扰动估计（DC-PMPPI 核心）；
- `computeVelocityCommands(pose, speed)`：主循环调用，返回最优 `Twist2D`；
- `enforceMotionConstraints(cmd)`：对终端稳定器输出再次施加运动学约束；
- 一系列 Getter：`getOptimizedTrajectory`、`getEffectiveSampleSize`、`getLastMinCost/MaxCost`、`getCriticStatistics`。

#### `optimizer.hpp` — MPPI 优化器核心

实现完整的优化主循环：

1. `prepare`：路径裁剪、参考控制序列生成、约束缩放；
2. `optimize`：生成噪声轨迹 → Critic 评分 → 加权更新 → 施加约束；
3. `evalControl`：多轮迭代 + 早停（代价变化小于阈值）+ Savitzky-Golay 滤波；
4. `getControlFromSequence`：从控制序列首项提取指令，附加航向对齐门控、弯道对准、目标制动、曲率一致性保持等后处理。

路径裁剪采用"局部搜索窗口 + 物理距离 + 航向匹配"三重策略，发卡弯等大转角下不会误判最近点。`furthest_reached_quantile` 用分位数而非均值确定轨迹终点对应的路径进度，对个别异常 rollout 鲁棒。

#### `motion_models.hpp` — 三种运动学模型

- `DiffDriveMotionModel`：差速驱动，非完整约束；
- `OmniMotionModel`：全向移动，`v_y` 主动通道；
- `AckermannMotionModel`：阿克曼，含最小转弯半径与横向加速度双约束，在 `predict` 阶段对每个状态点重新投影到可行域。

所有模型共享 `predict` 接口：第一时刻用当前测量速度，后续时刻按"指令有效性 × 含噪声控制量"传播并施加加速度约束。

#### `critics/` — 代价函数集合

每个 Critic 继承 `CriticFunction`，实现 `score(CriticData&)` 将代价累加到 `data.costs`。当前默认启用的两个核心 Critic：

- **`MPCCTrackingCritic`**：MPCC 式综合跟踪代价，包含轮廓、滞后、航向、速度、横摆角速度、终端六项。`thread_count` 控制 OpenMP 并行评分。
- **`ControlRateCritic`**：控制变化率代价，惩罚 `(Δv_x/Δt)²`、`(Δω/Δt)²`，平滑指令序列。

其余 Critic 作为可选模块，可根据场景启用：

- `ConstraintCritic`：硬约束越界惩罚（含阿克曼转弯半径）；
- `StabilityCritic`：低附着稳定性，限制合加速度与横摆加速度；
- `PathAlignCritic` / `PathFollowCritic` / `PathAngleCritic`：经典 Nav2 风格的路径对齐/跟随/角度代价；
- `LateralErrorCritic`：基于 LQR Riccati 解的横向误差最优加权代价；
- `PreferForwardCritic` / `TwirlingCritic` / `VelocityDeadbandCritic`：偏好前进、抑制原地旋转、速度死区。

`CriticManager` 支持每个 Critic 的代价贡献与耗时统计（`collect_critic_statistics` 开关），便于调参定位瓶颈。

#### `tools/disturbance_estimator.hpp` — 在线扰动与有效性估计器

DC-PMPPI 的核心模块。实现：

- 位姿差分速度计算（可选直接读取里程计 twist）；
- 一阶滞后执行器模型补偿（避免将电机响应滞后误判为附着力下降）；
- 中心化 Welford 回归或稳健比值法辨识 `η_v, η_ω`；
- 指数低通滤波 + 创新限幅的扰动估计；
- 同步方差递推（用于可选 sigma-point 扰动采样）；
- 置信度估计（基于创新幅值与持续样本数）。

#### `tools/noise_generator.hpp` — 噪声生成器

支持多种采样策略，互斥保护：

- **白噪声 + AR(1) 时间相关**：`noise_correlation` β 控制时序相关性，β=0 为白噪声，β→1 为平滑序列；
- **KMPPI 风格 RBF 核插值**：在少量支持点上优化噪声，再通过 RBF 核扩展到完整 horizon，降低优化维度；
- **控制导数采样**（Smooth-MPPI）：噪声作用在控制增量上并积分，得到平滑绝对控制；
- **反对称采样**：成对正负扰动，方差减半；
- **名义样本保留**：`nominal_sample_count` 份零噪声样本，保证当前控制序列不被采样集合完全排除；
- **跨周期左移**：`reuse_noise_sequence` 复用上周期噪声并仅注入正交新息，抑制相邻周期的随机跳变。

#### `tools/path_projection.hpp` / `path_reference.hpp` — 路径几何工具

- `computePathArcLengths`：路径弧长累积；
- `projectPointToPath`：点到分段线性路径的投影，返回最近点、有符号横向误差、弧长、切向朝向；
- `samplePathReference`：按弧长采样路径参考状态（位置、朝向、曲率）；
- `previewReferenceSpeed`：考虑曲率限速、弯道前视减速、终点制动三重约束的参考速度；
- `smoothMinimum`：带过渡区的光滑最小算子。

#### `tools/goal_docking.hpp` — 终端位姿稳定器

在距离目标 `activation_distance` 内逐步接管控制，融合 MPPI 名义指令与位姿稳定反馈：

- 距离 - 速度模型：`v = min(v_max, k_d·d, sqrt(2·a_goal·d))`；
- 方位反馈 + 终端朝向耦合；
- 速度低 + 距离未到时触发"重捕获"模式，允许更大速度避免在目标附近停滞；
- 滑移角补偿与 η 缩放，保证湿滑工况下仍能精确靠站。

### 4.3 关键参数解读

完整参数见 [`config/dc_pmppi_params.yaml`](config/dc_pmppi_params.yaml)，下表列出对性能影响最大的几组：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `batch_size` | 400 | 单周期采样轨迹数；Gazebo 稳健基线，超时后按比例下调 |
| `time_steps` | 40 | 预测时域步数，40×0.05s = 2.0s horizon |
| `model_dt` | 0.05 s | 模型离散步长 |
| `temperature` | 0.60 | softmax 温度 λ，开启 `adaptive_temperature` 后为初值 |
| `noise_correlation` | 0.85 | AR(1) 噪声时序相关性，0=白噪声 |
| `antithetic_sampling` | true | 反对称采样，降低蒙特卡洛方差 |
| `guided_sampling_ratio` | 0.25 | 25% rollout 使用曲率前馈引导分布 |
| `reuse_noise_sequence` | true | 跨周期左移低频噪声，抑制周期跳变 |
| `prune_distance` | 3.5 m | 路径裁剪距离，仅保留车前方此范围内的路径点 |
| `motion_model` | "Ackermann" | 运动模型选择：DiffDrive / Omni / Ackermann |
| `ackermann_min_turning_radius` | 0.50 m | 阿克曼最小转弯半径 |
| `path_reference_speed` | 0.60 m/s | 标称巡航速度 |
| `path_max_lateral_acceleration` | 0.60 m/s² | 曲率限速的横向加速度上限 |
| `path_corner_preview_distance` | 1.20 m | 弯道前视距离 |
| `mpcc_contour_weight` | 2.2 | 轮廓误差权重，强化几何跟踪 |
| `dc_pmppi_enabled` | false | DC-PMPPI 总开关；false 时退化为标准 MPPI |
| `dc_pmppi_filter_time_constant` | 0.20 s | 扰动估计低通时间常数 |
| `dc_pmppi_effectiveness_time_constant` | 1.50 s | 控制有效性 η 辨识时间常数 |
| `dc_pmppi_confidence_time_constant` | 0.50 s | 置信度渐进启用时间常数 |
| `dc_pmppi_compensation_gain` | 0.85 | 总补偿增益，抑制多处注入导致的过补偿 |
| `disturbance_decay_time_constant` | 0.8 s | Rollout 中扰动衰减时间常数 τ |
| `min_traction_scale` | 0.35 | 附着能力指标下限 |

---

## 五、运行步骤

### 5.1 编译

在 catkin 工作空间根目录执行：

```bash
cd ~/catkin_ws
catkin_make --pkg dc_pmppi
source devel/setup.bash
```

如需启用 OpenMP 加速，确认系统已安装 `libomp-dev`；CMake 会自动检测并链接。

### 5.2 启动控制器

```bash
# 1) 启动仿真与里程计（项目内已有 j15_sim.launch）
roslaunch dc_pmppi j15_sim.launch

# 2) 启动 DC-PMPPI 控制器（默认 dc_pmppi_enabled=false，即标准 MPPI 基线）
roslaunch dc_pmppi j15_dc_pmppi.launch

# 3) 启用 DC-PMPPI 扰动补偿（A/B 实验时使用）
roslaunch dc_pmppi j15_dc_pmppi.launch dc_pmppi_enabled:=true

# 4) 发布全局路径
roslaunch dc_pmppi j15_path.launch
```

### 5.3 话题接口

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 订阅 | `/plan` | `nav_msgs/Path` | 全局路径，frame 须与 `planning_frame` 一致 |
| 订阅 | `/odom1` | `nav_msgs/Odometry` | 机器人位姿与速度 |
| 订阅 | `/constrained_cmd_vel_stamped` | `geometry_msgs/TwistStamped` | 实际执行的指令（用于扰动估计，可选） |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` | 控制指令 |
| 发布 | `/local_path` | `nav_msgs/Path` | 优化器预测轨迹（用于可视化） |
| 发布 | `/controller_solve_time_ms` | `std_msgs/Float64` | 单周期求解耗时 |

### 5.4 实时调参

运行时通过 `rqt_reconfigure` 或直接 `rosparam set` 调整：

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

重点关注：

- 求解耗时 `/controller_solve_time_ms` 应稳定低于 `control_period_ms`（默认 50 ms）；
- ESS（日志中的 `ESS=`）建议保持在 batch_size 的 5%~30%；
- 启用 `collect_critic_statistics` 后日志会打印每个 Critic 的均值与耗时。

### 5.5 单元测试

```bash
catkin_make run_tests_dc_pmppi
```

主要测试 `DisturbanceEstimator` 在阶跃/正弦扰动下的收敛性、方差递推正确性以及置信度过渡行为。

---

## 六、项目依赖库

### 6.1 系统依赖

| 依赖 | 版本要求 | 用途 | 安装方式 |
|------|----------|------|----------|
| ROS | Noetic | 节点框架、消息接口 | apt |
| catkin | - | 构建系统 | 随 ROS 安装 |
| C++ | C++17 | 语言标准 | 编译器自带 |
| CMake | ≥ 3.1 | 构建配置 | apt |
| OpenMP | ≥ 4.5 | 并行采样与评分 | apt `libomp-dev` |
| Eigen3 | ≥ 3.3 | 几何运算 | apt `libeigen3-dev` |

### 6.2 ROS 包依赖

`package.xml` 中声明：

- `roscpp` / `rospy` — ROS 通信
- `nav_msgs` — Path / Odometry 消息
- `geometry_msgs` — Twist / TwistStamped / Pose
- `std_msgs` — Float64 等基础类型
- `sensor_msgs` — 传感器消息（间接依赖）
- `tf` — 坐标变换
- `ackermann_msgs` — Ackermann 指令消息
- `onboard_detector` — 可选的动静态障碍检测器（exec_depend）

### 6.3 内嵌第三方库

为避免外部依赖管理负担，以下库以源码方式内嵌在包内：

- **`xtensor/`** — C++ 张量运算库，提供 NumPy 风格的 `xtensor<float, N>` 多维数组与向量化数学运算。所有批量采样、Rollout、代价计算均基于 xtensor 实现，可在编译期开启 SIMD 优化。
- **`xsimd/`** — C++ SIMD 抽象库，被 xtensor 用于 AVX/SSE/NEON 指令集的底层向量化加速。

内嵌路径在 `CMakeLists.txt` 中通过 `XTENSOR_INCLUDE_DIRS` 显式包含：

```cmake
set(XTENSOR_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/xtensor/include
    ${CMAKE_CURRENT_SOURCE_DIR}/xsimd/include
    ${CMAKE_CURRENT_SOURCE_DIR}/xtl/include
)
```

无需单独安装 xtensor/xsimd，编译时直接使用。

### 6.4 Python 依赖（脚本端）

`scripts/JZJ_path.py` 与 `scripts/twist_to_ackermann.py` 仅依赖 `rospy` 与 `numpy`，随 ROS Noetic 默认可用。

---

## 七、扩展与消融实验

项目内保留了多个用于消融对比的可选模块，便于科研复现：

| 配置项 | 关闭后行为 | 适用场景 |
|--------|------------|----------|
| `dc_pmppi_enabled` | 退化为标准 MPPI | A/B 对比扰动补偿效果 |
| `adaptive_temperature` | 固定温度 λ | 验证 ESS 目标温度鲁棒性 |
| `elite_sample_count` | 全样本 MPPI 加权 | CEM 截断消融 |
| `antithetic_sampling` | 独立同分布高斯采样 | 方差缩减效果评估 |
| `noise_correlation` | 白噪声采样 | 时序相关性消融 |
| `sampling_support_points` | 全 horizon 独立噪声 | KMPPI 核插值消融 |
| `reuse_noise_sequence` | 每周期重新采样 | 跨周期相关性消融 |
| `use_sg_filter` | 不做控制序列后处理平滑 | Savitzky-Golay 滤波消融 |
| `guided_sampling_ratio` | 纯高斯采样 | 曲率前馈引导消融 |
| `sample_disturbance_uncertainty` | 扰动均值注入 | sigma-point 扰动采样消融 |
| `dc_pmppi_use_centered_effectiveness_regression` | 稳健比值法 | η 辨识方法对比 |

通过逐项切换上述参数，可以复现标准 MPPI、CEM-MPPI、Smooth-MPPI、KMPPI、RMPPI 等多个变体的行为，方便做算法对比实验。

---

## 许可证

本项目采用 [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0) 开源。

内嵌的 `xtensor` 与 `xsimd` 各自保留其原始许可证（BSD-3-Clause），详见对应目录下的 `LICENSE` 文件。
