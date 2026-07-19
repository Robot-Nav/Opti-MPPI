# SA MPPI (Shape Aware MPPI) Local Planner: Real-time Trajectory Planning for Arbitrarily Shaped Robots

<p align="center">
  <a href="https://en.cppreference.com/"><img src="https://img.shields.io/badge/C++-17-blue.svg" alt="C++17"></a>
  <a href="https://docs.nav2.org/configuration/packages/configuring-mppic.html"><img src="https://img.shields.io/badge/MPPI-Nav2-ff69b4.svg" alt="MPPI"></a>
  <img src="https://img.shields.io/badge/Platform-Cross--Platform-orange.svg" alt="Cross Platform">
  <a href="https://github.com/xtensor-stack/xtensor"><img src="https://img.shields.io/badge/xtensor-0.21-purple.svg" alt="xtensor"></a>
  <img src="https://img.shields.io/badge/License-GPLv3-blue.svg" alt="License">
  <img src="https://img.shields.io/badge/Status-Active-brightgreen.svg" alt="Status">
</p>

> **Declaration**: [👉 README_CN ](https://github.com/Robot-Nav/SA-MPPI/blob/main/README_CN.md)This repository is a decoupled version of the MPPI local planner from Navigation2. Due to limited technical expertise, there may be some "hacky" modifications and non-standard code. Some implementation logic may differ from the original version. **The code is under continuous development**. If there are any inaccuracies, please feel free to point them out — your understanding is greatly appreciated! Issues and PRs are welcome.

> [Friendly reminder](https://github.com/Robot-Nav/SA-MPPI/issues/2)：Some suggestions

> **Highly recommend website**：[👉 Different MPPI ](http://robotics-tutorial.dmbot.cn/04_%E7%A7%BB%E5%8A%A8%E6%9C%BA%E5%99%A8%E4%BA%BA%E8%A7%84%E6%8E%A7/20_%E9%87%87%E6%A0%B7%E5%BC%8FMPC/40_MPPI%E5%85%AD%E5%A4%A7%E5%8F%98%E4%BD%93/#rmppi)
> <img width="1056" height="622" alt="image" src="https://github.com/user-attachments/assets/de6184a6-7b1b-442a-9997-b3f97421c26e" />

> **Reference Blogs**:
>
> 1. [Shape-Aware MPPI (SA MPPI) Algorithm: Real-time Trajectory Optimization for Arbitrarily Shaped Robots Based on RC-ESDF](https://blog.csdn.net/qq_56908984/article/details/160439414)
>
> 2. [RC-ESDF Explained: Robot-Centric Euclidean Signed Distance Field](https://blog.csdn.net/qq_56908984/article/details/160087544)
>
> 3. [MPPI_C++: Decoupled MPPI Local Planner from Nav2 (Hackable)](https://blog.csdn.net/qq_56908984/article/details/158774722)
>
> 4. [SA-MPPI Cost Calculator Online Interactive Webpage](https://opti-mppi-cost-calculator.vercel.app/critic_cost_calculator.html)

## Table of Contents

- [Project Overview](#1-project-overview)
- [Features](#2-features)
- [Mathematical Principles](#3-mathematical-principles)
- [Architecture Design](#4-architecture-design)
- [Module Details](#5-module-details)
- [Installation and Compilation](#6-installation-and-compilation)
- [Usage](#7-usage)
- [Parameter Configuration](#8-parameter-configuration)
- [Topics and Interfaces](#9-topics-and-interfaces)
- [Algorithm Details](#10-algorithm-details)
- [Performance Optimization](#11-performance-optimization)
- [Differences from Nav2 Official MPPI Controller](#12-differences-from-nav2-official-mppi-controller)
- [License](#13-license)
- [References](#14-references)

***

## 1. Project Overview

A decoupled implementation of the MPPI (Model Predictive Path Integral) controller extracted from the ROS2 Nav2 framework, written in **C++17** and adapted for the ROS1 environment. The core controller has **no dependencies** on Nav2 infrastructure (rclcpp, nav2\_core, nav2\_costmap\_2d, pluginlib, etc.), requiring only standard C++17 and the header-only `xtensor` numerical library. Combined with the RC-ESDF algorithm, it builds a zoned collision detection mechanism based on RC-ESDF's precise distance queries. In the risk zone, RC-ESDF's analytical gradient is used to bias MPPI sampling, guiding candidate trajectories to expand toward safer regions and improving the flexibility and smoothness of dynamic obstacle avoidance for arbitrarily shaped robots.

### 1.1 Core Positioning

- **Framework Agnostic**: No dependency on ROS2/Nav2 lifecycle nodes, Costmap2D, pluginlib, or other infrastructure — embeddable into any C++ project
- **LIDAR Native**: Directly uses laser point clouds as obstacle input, building a local BFS grid distance field for fast distance queries without relying on global costmaps. The local distance field replaces Nav2-MPPI's costmap dependency, decoupling obstacle detection from centralized global layers to a robot-centric perception loop. This distance field module can further integrate RC-ESDF (Robot-Centric Euclidean Signed Distance Field), which precomputes signed distance fields offline centered on the robot, supporting precise distance queries and zoned collision detection for arbitrary polygon footprints. In the risk zone, its analytical gradient can be used to bias MPPI sampling and guide candidate trajectories toward safer regions. It offers significant advantages in narrow-space navigation and non-circular robot obstacle avoidance. See [SA-MPPI: Real-time Trajectory Optimization for Arbitrarily Shaped Robots Based on RC-ESDF](https://blog.csdn.net/qq_56908984/article/details/160439414) for details.
- **Lightweight Deployment**: Depends only on xtensor, simple compilation, suitable for embedded and real-time scenarios
- **Feature Complete**: Retains the core algorithm logic of Nav2 MPPI with several practical enhancements

### 1.2 Core Capabilities

- **Real-time Trajectory Optimization**: Optimizes control sequences at 20Hz (50ms control period)
- **LIDAR Obstacle Avoidance**: Efficient collision detection using BFS-based local grid distance field
- **Multi-threaded Sampling**: Configurable thread pool for parallel trajectory generation
- **Dynamic Obstacle Support**: Predictive collision avoidance for moving obstacles, extensible
- **Blocking Detection**: Prevents spinning-in-place behavior when blocked by obstacles
- **Multiple Motion Models**: Supports differential drive, omnidirectional, and Ackermann steering models
- **Omni/Diff Automatic Switching**: Automatically switches motion models based on obstacle distance and path deviation

### 1.3 Basic Demo



https://github.com/user-attachments/assets/575febab-0bb6-4e82-a43d-874637169713



***

## 2. Features

### 2.1 Core Features

| Feature | Description |
| --- | --- |
| **Batch Sampling** | 1000+ trajectory samples per control cycle |
| **Grid Distance Field** | BFS-based local distance field for O(1) obstacle distance queries |
| **Multi-threading** | Configurable thread count for parallel trajectory integration |
| **Adaptive Temperature** | Dynamic temperature adjustment based on cost distribution |
| **SG Filtering** | Savitzky-Golay filter for smooth control sequences |
| **Footprint Support** | Collision detection for non-circular robots (polygon footprint) |
| **Dynamic Obstacles** | Velocity-based linear prediction for collision avoidance |
| **Blocking Detection** | Detects spinning-in-place and outputs zero velocity |
| **Omni/Diff Switching** | Automatically switches motion models based on environment |
| **Path Blocking Detection** | Automatically disables alignment cost when path is heavily occupied |
| **Path Point Validity** | Skips path points blocked by obstacles |
| **Startup Assist** | Resolves zero-velocity startup issues / redundant, removable |

### 2.2 Cost Function Components

| Cost Function | Purpose | Activation Condition |
| --- | --- | --- |
| `ObstaclesCritic` | Exponential repulsion-based collision avoidance (grid distance field + dynamic prediction) | When away from goal |
| `PathFollowCritic` | Endpoint tracking along global path | When away from goal |
| `PathAlignCritic` | Trajectory alignment with path geometry (includes path blocking detection) | When path is not blocked |
| `PathAngleCritic` | Heading alignment with path direction (three modes) | When away from goal |
| `GoalCritic` | Final position convergence | When near goal |
| `GoalAngleCritic` | Final heading alignment | When near goal |
| `PreferForwardCritic` | Penalizes backward motion | When away from goal |
| `ConstraintCritic` | Kinematic constraint enforcement (diff/omni/ackermann) | Always |
| `TwirlingCritic` | Suppresses spinning-in-place behavior | When away from goal |
| `VelocityDeadbandCritic` | Low-speed stability improvement | Always |

### 2.3 Cross-Platform Portability

| Platform | Support | Notes |
| --- | --- | --- |
| **Linux (x86/ARM)** | Full | Primary development platform |
| **ROS1** | Full | Reference implementation provided |
| **ROS2** | Compatible | Uses ROS2 message type wrappers |
| **Embedded Linux** | Full | Self-testable |
| **Real-time OS** | Compatible | No dynamic memory allocation in hot path |

**Dependencies (Core Controller Only)**:

- C++17 compiler
- [xtensor 0.21](https://github.com/xtensor-stack/xtensor) (header-only numerical library)
- Standard C++ library only

***

## 3. Mathematical Principles

### 3.1 MPPI Core Algorithm

The MPPI algorithm solves stochastic optimal control problems by sampling trajectories and computing weighted averages of control sequences. Unlike traditional MPC that requires solving constrained optimization problems, MPPI transforms the optimal control problem into a **path integral** estimation problem, solved via Monte Carlo sampling without complex gradient computations or constraint handling.

#### State Dynamics (Differential Drive)

$$\dot{x} = v\_x \cos\theta$$

$$\dot{y} = v\_x \sin\theta$$

$$\dot{\theta} = \omega\_z$$

Where:

- $(x, y)$: Robot position in world coordinates
- $\theta$: Robot heading
- $v\_x$: Linear velocity
- $\omega\_z$: Angular velocity

#### Discretized Motion Model

$$x\_{t+1} = x\_t + (v\_x \cos\theta - v\_y \sin\theta) \cdot \Delta t$$

$$y\_{t+1} = y\_t + (v\_x \sin\theta + v\_y \cos\theta) \cdot \Delta t$$

$$\theta\_{t+1} = \theta\_t + \omega \cdot \Delta t$$

Where $v\_y$ is non-zero only in the omnidirectional motion model.

#### Trajectory Sampling

For each batch $i$ and time step $t$:

$$u\_t^{(i)} = \bar{\nu}\_t + \epsilon\_t^{(i)}, \quad \epsilon\_t^{(i)} \sim \mathcal{N}(0, \Sigma)$$

Where:

- $\bar{\nu}\_t$: Mean control from previous iteration (current control sequence)
- $\epsilon\_t^{(i)}$: Gaussian noise with standard deviation $\sigma$

#### Trajectory Cost Function

Total cost for trajectory $i$:

$$S(\tau^{(i)}) = \sum\_{t=0}^{T-1} \left\[ q(x\_t^{(i)}) + \frac{\gamma}{2\sigma^2} \bar{\nu}\_t^2 + \frac{\gamma}{\sigma^2} \bar{\nu}\_t \cdot \epsilon\_t^{(i)} \right] \cdot \Delta t$$

Where:

- $q(x\_t^{(i)})$: State cost (computed by each Critic function)
- $\gamma$: Control cost weight parameter
- $\sigma$: Sampling noise standard deviation
- $\bar{\nu}\_t$: Current control sequence
- $\epsilon\_t^{(i)}$: Noise for trajectory $i$
- $T$: Time horizon (number of steps)

The cost function consists of three parts:

1. **State cost** $q(x\_t^{(i)})$: Independently computed and accumulated by 10 Critic functions
2. **Control regularization** $\frac{\gamma}{2\sigma^2}\bar{\nu}\_t^2$: Penalizes excessive control magnitudes
3. **Control-noise cross term** $\frac{\gamma}{\sigma^2}\bar{\nu}\_t \cdot \epsilon\_t^{(i)}$: Ensures unbiased optimal control update

#### Softmax Weighting

The optimal control update uses softmax weights:

$$w^{(i)} = \frac{\exp\left(-\frac{1}{\lambda}(S(\tau^{(i)}) - S\_{\min})\right)}{\sum\_{j=1}^{K} \exp\left(-\frac{1}{\lambda}(S(\tau^{(j)}) - S\_{\min})\right)}$$

Where:

- $\lambda$: Temperature parameter controlling weight distribution sharpness
- $S\_{\min}$: Minimum cost among all trajectories (for numerical stability)
- $K$: Batch size (number of samples)

Mean normalization mode is also supported:

$$w^{(i)} = \frac{\exp\left(-\frac{1}{\lambda}(S(\tau^{(i)}) - \bar{S})\right)}{\sum\_{j=1}^{K} \exp\left(-\frac{1}{\lambda}(S(\tau^{(j)}) - \bar{S})\right)}$$

Where $\bar{S}$ is the mean cost across all trajectories.

#### Control Update

Updated control sequence:

$$\bar{\nu}_t^{\text{new}} = \sum_{i=1}^{K} w^{(i)} \cdot u\_t^{(i)}$$

### 3.2 Obstacle Cost Function

The obstacle cost function uses **exponential repulsion cost**:

$$C\_{\text{obs}}(d) = \begin{cases} C\_{\text{collision}} & \text{if } d < d\_{\text{margin}} \ w\_{\text{repulsion}} \cdot \exp(-\alpha \cdot (d - d\_{\text{margin}})) & \text{if } d\_{\text{margin}} \le d < r\_{\text{inflation}} \ 0 & \text{otherwise} \end{cases}$$

Where:

- $d$: Distance to nearest obstacle
- $d\_{\text{margin}}$: Collision boundary distance (`collision_margin`)
- $r\_{\text{inflation}}$: Inflation radius (`inflation_radius`)
- $\alpha$: Cost scaling factor (`cost_scaling`)
- $w\_{\text{repulsion}}$: Repulsion weight (`repulsion_weight`)
- $C\_{\text{collision}}$: Collision cost (very large, e.g., 100000)

### 3.3 Grid Distance Field

The local distance field is computed using BFS (Breadth-First Search) centered on the robot:

```
For each grid cell (i, j):
  D[i,j] = min_{obstacle cells} ||(i,j) - (i_obs, j_obs)||
```

Construction process:

1. Map all obstacle points to grid coordinates with distance 0
2. BFS four-neighbor propagation from all obstacle cells
3. Each neighbor's distance = current distance + grid resolution

Querying is O(1) via direct coordinate mapping.

Default parameters: 0.05m resolution, 100×100 grid, covering 5m×5m local area.

### 3.4 Acceleration Constraints

Velocity updates obey acceleration limits:

$$v\_{t+1} = v\_t + \text{clamp}(v\_{\text{desired}} - v\_t, -a\_{\max} \Delta t, a\_{\max} \Delta t)$$

Where:

- $a\_{\max}$: Maximum acceleration
- $\Delta t$: Time step

This constraint is implemented in the motion model's `predict()` method, ensuring generated trajectories satisfy physical acceleration limits.

### 3.5 Blocking Detection

The controller uses rotation ratio to detect blocking:

$$\text{ratio} = \frac{|\omega\_z|}{|v\_x| + \epsilon}$$

If $\text{ratio} > \theta\_{\text{threshold}}$ for $N$ consecutive frames, the robot is considered blocked and commanded to zero velocity.

Blocking detection has two layers:

1. **TwirlingCritic**: Soft constraint via cost function penalizing "low forward + high rotation" behavior
2. **Optimizer::isBlocked()**: Hard constraint detecting average wz/vx ratio in control sequence, outputting zero velocity when exceeding threshold for consecutive frames

### 3.6 Savitzky-Golay Filtering

Control sequence smoothing uses SG filter coefficients:

$$\nu\_t^{\text{filtered}} = \sum\_{k=-2}^{2} c\_{k+2} \cdot \nu\_{t+k}$$

Default coefficients: $c = \[-0.085714, 0.342857, 0.485714, 0.342857, -0.085714]$ (5-point, 2nd-order fit).

Boundary handling: When indices exceed control sequence range, control history (previous frames' actual control) is used for padding.

### 3.7 Adaptive Temperature

When adaptive temperature is enabled, the temperature parameter adjusts dynamically based on cost distribution:

$$T = \text{clamp}\left(T\_{\text{base}} \cdot \left(1 + \frac{\ln(S\_{\max} - S\_{\min} + 1)}{5}\right), T\_{\min}, T\_{\max}\right)$$

Automatically increases temperature to smooth weights when cost distribution range is large, and decreases temperature to highlight optimal trajectories when cost distribution is concentrated.

***

## 4. Architecture Design

### 4.1 Layered Architecture

**Layered architecture** separates core algorithms from framework-specific implementations:

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROS1 Application Layer                    │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              mppi_ros1_node.cpp                          │   │
│  │  Subscribes: /plan, /odom, /scan  |  Publishes: /cmd_vel, debug │   │
│  │  Parameter loading | Startup assist | TF | Exception handling │   │
│  └──────────────────────┬───────────────────────────────────┘   │
└─────────────────────────┼───────────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────────┐
│                     Controller Interface Layer                   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              MPPIController (controller.hpp)              │   │
│  │  Init | Cost param setting | Obstacle update | Motion model switch │   │
│  │  Path point validity | Omni/diff auto-switching          │   │
│  └──────────┬──────────────────────────┬────────────────────┘   │
└─────────────┼──────────────────────────┼────────────────────────┘
              │                          │
┌─────────────▼──────────┐  ┌────────────▼────────────────────────┐
│      Optimizer Core     │  │         Cost Function Layer         │
│  ┌──────────────────┐  │  │  ┌─────────────────────────────┐    │
│  │  Optimizer       │  │  │  │     CriticManager           │    │
│  │  Sampling | Scoring│  │  │  ┌───────────────────────┐  │    │
│  │  Weighted update │  │  │  │ ObstaclesCritic       │  │    │
│  │  Filtering       │  │  │  │ PathFollowCritic      │  │    │
│  │  Blocking detect │  │  │  │ PathAlignCritic       │  │    │
│  │  Path pruning    │  │  │  │ PathAngleCritic       │  │    │
│  └───────┬──────────┘  │  │  │ GoalCritic            │  │    │
│          │              │  │  │ GoalAngleCritic       │  │    │
│  ┌───────▼──────────┐  │  │  │ PreferForwardCritic   │  │    │
│  │  NoiseGenerator  │  │  │  │ ConstraintCritic      │  │    │
│  │  Async noise     │  │  │  │ TwirlingCritic        │  │    │
│  └──────────────────┘  │  │  │ VelocityDeadbandCrit  │  │    │
│  ┌──────────────────┐  │  │  └───────────────────────┘  │    │
│  │  MotionModel     │  │  └─────────────────────────────┘    │
│  │  Diff | Omni     │  │                                      │
│  │  Ackermann       │  │                                      │
│  └──────────────────┘  │                                      │
└────────────────────────┘  └──────────────────────────────────────┘
              │                          │
┌─────────────▼──────────────────────────▼──────────────────────────┐
│                        Data Model Layer                            │
│  types | constraints | control_sequence | state | trajectories   │
│  path  | optimizer_settings                                        │
├───────────────────────────────────────────────────────────────────┤
│                        Utilities Layer                             │
│  math_utils (angle normalization, clamp) | noise_generator       │
├───────────────────────────────────────────────────────────────────┤
│                        External Dependencies (Minimal)            │
│  xtensor (header-only) | xsimd (SIMD) | xtl (base) | C++17       │
└───────────────────────────────────────────────────────────────────┘
```

### 4.2 Key Design Principles

1. **Zero ROS Dependency in Core**: All headers under `include/` use only standard C++17 and xtensor, containing no ROS headers
2. **Modular Separation**: Split into 20+ header files by functional responsibility, each focused on a single concern
3. **Inheritance-based Design**: Motion models and cost functions define interfaces via base classes with derived implementations
4. **Asynchronous Noise Generation**: Independent thread pre-generates noise to reduce main loop latency
5. **Multi-threaded Trajectory Integration**: Configurable thread count for parallel computation
6. **Deterministic Execution**: No non-deterministic operations in the control loop

### 4.3 File Organization

```
mppi_laser_example/
├── include/
│   ├── controller.hpp              # Public interface: MPPIController class
│   ├── optimizer.hpp               # Optimizer core: sampling, scoring, weighted update
│   ├── motion_models.hpp           # Motion models: DiffDrive/Omni/Ackermann
│   ├── models/
│   │   ├── types.hpp               # Basic types: Point2D, Pose2D, Twist2D, Control, ObstaclePrediction
│   │   ├── constraints.hpp         # Constraint parameters: ControlConstraints, SamplingStd
│   │   ├── control_sequence.hpp    # Control sequence data structure
│   │   ├── state.hpp               # State container (batch velocities + controls)
│   │   ├── trajectories.hpp        # Trajectory container (batch positions + headings)
│   │   ├── path.hpp                # Global path data structure
│   │   └── optimizer_settings.hpp  # Optimizer configuration parameters
│   ├── critics/
│   │   ├── critic_function.hpp     # Cost function base class
│   │   ├── critic_data.hpp         # Evaluation data structure
│   │   ├── critic_manager.hpp      # Cost function manager
│   │   ├── obstacles_critic.hpp    # Obstacle cost (grid distance field)
│   │   ├── path_follow_critic.hpp  # Path following cost
│   │   ├── path_align_critic.hpp   # Path alignment cost
│   │   ├── path_angle_critic.hpp   # Path angle cost
│   │   ├── goal_critic.hpp         # Goal point cost
│   │   ├── goal_angle_critic.hpp   # Goal heading cost
│   │   ├── prefer_forward_critic.hpp # Prefer forward cost
│   │   ├── constraint_critic.hpp   # Kinematic constraint cost
│   │   ├── twirling_critic.hpp     # Spinning penalty cost
│   │   └── velocity_deadband_critic.hpp # Velocity deadband cost
│   └── tools/
│       ├── math_utils.hpp          # Math utilities: angle normalization, clamp
│       └── noise_generator.hpp     # Asynchronous Gaussian noise generator
├── src/
│   └── mppi_ros1_node.cpp          # ROS1 main node
├── scripts/
│   ├── mppi_path_publisher.py      # Path publisher (straight/rectangle/S-curve)
│   ├── mppi_path_publisher_fz.py   # Enhanced path publisher (with rotation support)
├── config/
│   ├── mppi_params.yaml            # Parameter config (detailed Chinese comments)
│   └── mppi_test.rviz              # RViz visualization config
├── CMakeLists.txt                  # Catkin build config
├── package.xml                     # ROS1 package manifest
├── xtensor/                        # Third-party: multi-dimensional arrays
├── xsimd/                          # Third-party: SIMD vectorization
└── xtl/                            # Third-party: xtensor base library
```

***

## 5. Module Details

### 5.1 Data Model Layer (models/)

#### types.hpp — Basic Data Types

| Type | Fields | Description |
| --- | --- | --- |
| `Point2D` | x, y | 2D point |
| `Pose2D` | x, y, theta | 2D pose |
| `Twist2D` | vx, vy, wz | 2D velocity |
| `Control` | vx, vy, wz | Single-step control |
| `ObstaclePrediction` | x, y, vx, vy, radius | Dynamic obstacle prediction with `positionAt(t)` method |

`ObstaclePrediction::positionAt(t)` performs linear prediction based on obstacle velocity:

$$\hat{x}(t) = x\_0 + v\_x \cdot t, \quad \hat{y}(t) = y\_0 + v\_y \cdot t$$

#### constraints.hpp — Control Constraints

- `ControlConstraints`: Velocity limits (vx\_max/min, vy\_max, wz\_max), acceleration limits (ax/ay/az\_max), collision cost threshold
- `SamplingStd`: Sampling noise standard deviations (vx, vy, wz)

#### control\_sequence.hpp — Control Sequence

`ControlSequence`: Control per time step (vx[], vy[], wz[]) stored as `xt::xtensor<float, 1>`. Supports `reset(time_steps)`.

#### state.hpp — State Container

`State`: Controls and velocities for all sampled trajectories as `[batch, time]` 2D tensors:

- `cvx, cvy, cwz`: Noised controls (sampling results)
- `vx, vy, wz`: Velocities after acceleration constraints
- `pose`: Current pose (Pose2D)
- `speed`: Current velocity (Twist2D)

#### trajectories.hpp — Trajectory Container

`Trajectories`: Positions and headings for all sampled trajectories as `[batch, time]` 2D tensors:

- `x, y`: Positions
- `yaws`: Headings
- `times`: Timestamps

#### path.hpp — Global Path

`Path`: Discrete path point sequence (x[], y[], yaws[]) with `getGoal()` for endpoint and `reset(size)`.

#### optimizer\_settings.hpp — Optimizer Settings

`OptimizerSettings`: All configurable parameters — see [Parameter Configuration](#8-parameter-configuration) section.

### 5.2 Utilities Layer (tools/)

#### math\_utils.hpp — Basic Math Utilities

| Function | Purpose |
| --- | --- |
| `normalizeAngle(angle)` | Normalize angle to $\[-\pi, \pi]$ |
| `shortestAngularDistance(from, to)` | Shortest signed angular difference |
| `clamp(val, min_val, max_val)` | Value clamping |

Constants: `PI = 3.14159265f`, `TWO_PI = 6.28318530f`, `EPSILON = 1e-6f`

#### noise\_generator.hpp — Asynchronous Gaussian Noise Generator

`NoiseGenerator` uses a separate thread for asynchronous pre-generation:

```
Main thread:   setNoisedControls() → wait for noise ready → read noise → trigger next generation
Noise thread:  wait for trigger → generate noise → notify ready → wait for trigger
```

- Uses `xt::random::randn` for Gaussian noise generation
- `std::mutex` + `std::condition_variable` for thread synchronization
- `setNoisedControls()`: Applies noise to control sequence
- `generateNextNoises()`: Triggers asynchronous pre-generation of next noise batch

### 5.3 Cost Function Layer (critics/)

#### critic\_function.hpp — Cost Function Base Class

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

#### critic\_data.hpp — Evaluation Data Structure

`CriticData`: Data collection passed to cost functions, containing:

- `state`: State container
- `trajectories`: Trajectory container
- `path`: Global path
- `costs`: Cost array (accumulated by each Critic)
- `model_dt`: Time step
- `fail_flag`: Failure flag (set true when all trajectories collide)
- `motion_model`: Current motion model
- `furthest_reached_path_point`: Furthest reachable path point index
- `dynamic_obstacles`: List of dynamic obstacles
- `path_pts_valid`: Path point validity flags

#### critic\_manager.hpp — Cost Function Manager

`CriticManager`: Manages all cost functions, executes `evalTrajectoriesScores(data)` for batch scoring, iterating through all registered Critics and calling `score()`.

#### obstacles\_critic.hpp — Obstacle Cost (Core)

`ObstaclesCritic` is the most complex cost function with core flow:

1. **Grid Reconstruction**: Build local grid centered at robot position, map obstacle points, BFS four-neighbor distance field propagation
2. **Trajectory Scoring**: Query distance field point-by-point for each trajectory
3. **Circle Mode**: Query distance from trajectory point to nearest obstacle, subtract robot radius
4. **Footprint Mode**: Transform footprint points to world coordinates, query distance field per point, take minimum distance
5. **Dynamic Obstacles**: Predict obstacle positions over time based on velocity, compute distances to trajectory points
6. **Collision Detection**: Distance < collision_margin → collision, assign collision_cost
7. **Repulsion Cost**: Distance in [collision_margin, inflation_radius) → exponential decay repulsion
8. **Total Collision Detection**: Set fail_flag when all trajectories collide

#### path\_follow\_critic.hpp — Path Following Cost

Computes squared distance between trajectory endpoint and path reference point:

$$
C_{\mathrm{follow}} = w \cdot \left\| \mathbf{p}_{\mathrm{traj\_end}} - \mathbf{p}_{\mathrm{path\_ref}} \right\|^2
$$

Reference point determined by `furthest_reached_path_point + offset`. Supports skipping path points blocked by obstacles.

#### path_align_critic.hpp — Path Alignment Cost

Computes average distance between each trajectory point and its nearest path point:

$$
C_{\text{align}} = w \cdot \frac{1}{N} \sum_{j=0}^{N-1} \left| \mathbf{p}_{\text{traj}}(j) - \mathbf{p}_{\text{path}}(\text{closest}(j)) \right|
$$

Key features:

- **Path Cumulative Distance Matching**: Nav2-style cumulative distance matching to avoid brute force search
- **Path Blocking Detection**: Automatically disabled when path occupancy ratio exceeds `max_path_occupancy_ratio`
- **Heading Consideration**: Optionally incorporates heading deviation into distance:

  $$
  d' = d \cdot (1 + 0.5 \cdot |\Delta\theta|)
  $$

#### path_angle_critic.hpp — Path Angle Cost

Penalizes heading deviation between trajectory endpoint and path reference point. Three modes:

| Mode | Value | Description |
| --- | --- | --- |
| `FORWARD_PREFERENCE` | 0 | Prefers forward direction, penalizes heading misalignment |
| `NO_DIRECTIONAL_PREFERENCE` | 1 | Allows reverse motion, penalizes heading deviation magnitude only |
| `CONSIDER_FEASIBLE_PATH_ORIENTATIONS` | 2 | Selects reference heading based on velocity direction |

#### goal_critic.hpp — Goal Point Cost

Penalizes squared distance between trajectory endpoint and goal point when near goal:

$$
C_{\mathrm{goal}} = w \cdot \left\| \mathbf{p}_{\mathrm{traj\_end}} - \mathbf{p}_{\mathrm{goal}} \right\|^2
$$

Active only when within `goal_threshold` distance of goal.

#### goal_angle_critic.hpp — Goal Heading Cost

Penalizes squared heading deviation between trajectory endpoint and goal heading when near goal:

$$
C_{\mathrm{goal\_angle}} = w \cdot (\theta_{\mathrm{traj\_end}} - \theta_{\mathrm{goal}})^2
$$

Active only when within `goal_angle_threshold` distance of goal.

#### prefer\_forward\_critic.hpp — Prefer Forward Cost

Penalizes negative (reverse) velocities in control sequence:

$$C\_{\text{forward}} = w \cdot \sum\_{t} \max(0, -v\_x(t))$$

Active only when distance to goal > `prefer_forward_threshold`.

#### constraint\_critic.hpp — Constraint Cost

Encourages robot to stay within kinematic constraints, penalizing velocity limit violations. Supports three motion models:

- Differential drive: Penalizes non-zero vy
- Omnidirectional: No additional constraints
- Ackermann: Penalizes turning radius below minimum

#### twirling\_critic.hpp — Spinning Penalty Cost

Penalizes "low forward + high rotation" behavior to prevent spinning-in-place:

$$C\_{\text{twirl}} = w \cdot \sum\_{t} \max(0, |w\_z(t)| - |v\_x(t)|)$$

Automatically disabled within `twirling_threshold` distance of goal to allow rotational alignment.

#### velocity\_deadband\_critic.hpp — Velocity Deadband Cost

Penalizes velocities below deadband thresholds to avoid low-speed instability:

$$C\_{\text{deadband}} = w \cdot \sum\_{t} \left\[\mathbb{1}(|v\_x| < \epsilon\_{vx}) + \mathbb{1}(|v\_y| < \epsilon\_{vy}) + \mathbb{1}(|w\_z| < \epsilon\_{wz})\right]$$

### 5.4 Core Algorithm Layer

#### motion_models.hpp — Motion Models

| Model | Class | Omnidirectional | Special Constraints |
| --- | --- | --- | --- |
| Differential Drive | `DiffDriveMotionModel` | No | None |
| Omnidirectional | `OmniMotionModel` | Yes | None |
| Ackermann Steering | `AckermannMotionModel` | No | Minimum turning radius |

All models share base class `MotionModel` acceleration constraint logic:

$$v\_{\text{new}} = v\_{\text{prev}} + \text{clamp}(u - v\_{\text{prev}}, -a\_{\max} \Delta t, a\_{\max} \Delta t)$$

Ackermann model additional constraint:

$$\text{if } R = \left|\frac{v\_x}{\omega\_z}\right| < R\_{\min}: \quad \omega\_z = \text{sign}(\omega\_z) \cdot \frac{|v\_x|}{R\_{\min}}$$

#### optimizer.hpp — Optimizer Core

`Optimizer` is the core of MPPI algorithm with execution flow:

```
evalControl()
  ├── prepare()                    # Update pose, prune path
  ├── [Iteration loop]
  │   ├── generateNoisedTrajectories()  # Sampling + forward simulation
  │   │   ├── setNoisedControls()       # Apply noise
  │   │   ├── applyControlConstraintsBatch()  # Velocity boundary constraints
  │   │   ├── applyConstraints()        # Kinematic constraints (e.g., Ackermann)
  │   │   ├── updateStateVelocities()   # Acceleration constraints
  │   │   ├── integrateStateVelocities() # Multi-threaded trajectory integration
  │   │   └── generateNextNoises()      # Trigger async noise generation
  │   ├── findFurthestReachedPathPoint() # Find furthest reachable path point
  │   ├── evalTrajectoriesScores()      # All Critics scoring
  │   ├── updateControlSequence()       # Softmax weighted update
  │   └── fallback()                    # Collision recovery
  ├── isBlocked()                  # Blocking detection
  ├── savitskyGolayFilter()        # SG filtering
  └── getControlFromSequence()     # Take first control
```

#### controller.hpp — Controller Interface

`MPPIController` class encapsulates optimizer and cost function manager, providing:

- `initialize()`: Initialize controller, create all Critics
- `updateStaticObstacles()`: Update obstacles + path validity + motion model switching
- `computeVelocityCommands()`: Main loop entry point
- Omni/diff auto-switching logic

### 5.5 ROS1 Integration Layer

#### mppi_ros1_node.cpp — ROS1 Main Node

`MPPIRos1Node` class implements ROS1 system interaction:

- **Subscribed Topics**: `plan` (path), `odom` (odometry), `scan` (laser scan)
- **Published Topics**: `cmd_vel` (velocity command), `debug_optimal_trajectory` (debug trajectory)
- **Control Loop**: 50ms timer (20Hz)
- **Parameter Loading**: Reads all MPPI parameters from ROS parameter server
- **Startup Assist**: `tryApplyStartupAssist()` — resolves zero-velocity startup issues
- **Coordinate Transform**: Converts laser points from robot frame to global frame
- **Exception Handling**: Outputs zero velocity and resets controller on MPPI failure
- **Arrival Detection**: Stops when within 0.20m of goal point

***

## 6. Installation and Compilation

### 6.1 Prerequisites

- ROS1 Melodic/Noetic
- C++17 compatible compiler (GCC ≥ 7)
- xtensor, xsimd, xtl libraries

### 6.2 Download Third-party Libraries (Re-download if repository ones are invalid)

Execute in the `mppi_laser_example` directory:

```bash
# Enter package directory
cd src/mppi_laser_example

# Download xtl (xtensor base library, install first)
git clone https://github.com/xtensor-stack/xtl.git
cd xtl && git checkout 0.7.0 && cd ..

# Download xsimd (SIMD acceleration library)
git clone https://github.com/xtensor-stack/xsimd.git
cd xsimd && git checkout 7.4.10 && cd ..

# Download xtensor (main numerical library, version 0.21.0)
git clone https://github.com/xtensor-stack/xtensor.git
cd xtensor && git checkout 0.21.0 && cd ..

# Return to workspace root
cd ../..
```

All three libraries are header-only — simply reference via `include_directories` in CMakeLists.txt, no separate compilation/installation required.

### 6.3 Build Steps

```bash
# 1. Enter workspace
cd ~/MPPI_ws_ros1sim

# 2. Build
catkin_make

# 3. Source workspace
source devel/setup.bash
```

### 6.4 Dependency Description

ROS1 dependencies declared via package.xml:

- roscpp, rospy, nav_msgs, geometry_msgs, std_msgs, sensor_msgs, tf

***

## 7. Usage

### 7.1 Start Controller

```bash
# Terminal 1: Start MPPI controller
rosrun mppi_laser_example mppi_laser_example_node _params_file:=src/mppi_laser_example/config/mppi_params.yaml

# Or load parameters via rosparam then start
rosparam load src/mppi_laser_example/config/mppi_params.yaml /mppi_ros1_node
rosrun mppi_laser_example mppi_laser_example_node
```

### 7.2 Publish Global Path

```bash
# Terminal 2: Publish path (supports multiple path types)
rosrun mppi_laser_example mppi_path_publisher.py
```

Path publisher commands:

- `s` — Switch to straight line path (25m)
- `t` — Switch to three-sided rectangle path (total 40m)
- `a` — Switch to arc-line combination path
- `p` — Print path information

### 7.3 Visualization

```bash
# Terminal 3: Visualize in RViz
rviz -d src/mppi_laser_example/config/mppi_test.rviz
```

### 7.4 Complete Run Flow

```bash
# 1. Build
cd ~/MPPI_ws_ros1sim && catkin_make && source devel/setup.bash

# 2. Start simulation environment (configure according to your robot simulation)
roslaunch your_robot_sim launch_file.launch

# 3. Load MPPI parameters
rosparam load src/mppi_laser_example/config/mppi_params.yaml /mppi_ros1_node

# 4. Start MPPI controller
rosrun mppi_laser_example mppi_laser_example_node

# 5. Publish global path
rosrun mppi_laser_example mppi_path_publisher.py

# 6. Visualize
rviz -d src/mppi_laser_example/config/mppi_test.rviz

# Or follow this project's flow:
# ROS1: MPPI
# 1.
source devel/setup.bash
# Step 1: Load YAML to parameter server
rosparam load $(rospack find mppi_laser_example)/config/mppi_params.yaml
# Step 2: Start node (node will automatically read parameters)
rosrun mppi_laser_example mppi_ros1_node

# 2. Publish path
rosrun mppi_laser_example mppi_path_publisher_fz.py _path_type:=three_side_rectangle _path_points:=300
# or
rosrun mppi_laser_example JZJ_path.py _path_type:=arc_line
# or
rosrun mppi_laser_example clean_path.py

# 3. rviz
rosrun rviz rviz -d /home/shaoyu/MPPI_ws_ros1sim/src/mppi_laser_example/config/mppi_test.rviz

# 4. (Diff/Omni)
roslaunch pioneer_utils bizhang.launch

# Static TF transform or start SLAM node
rosrun tf static_transform_publisher 0 0 0 0 0 0 map odom 100
```

***

## 8. Parameter Configuration

All parameters are configured in `config/mppi_params.yaml`, each with detailed Chinese comments. Listed by category below:

### 8.1 MPPI Core Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `batch_size` | 1500 | Number of trajectory samples, affects optimization accuracy and computation speed |
| `time_steps` | 40 | Prediction time steps, horizon = time_steps × model_dt |
| `model_dt` | 0.05 | Time step (seconds), controls trajectory prediction temporal resolution |
| `temperature` | 0.2 | Softmax temperature, controls weight distribution sharpness |
| `adaptive_temperature` | True | Enable adaptive temperature |
| `adaptive_temperature_min` | 0.1 | Adaptive temperature lower bound |
| `adaptive_temperature_max` | 1.0 | Adaptive temperature upper bound |
| `gamma` | 0.015 | Control cost weight, balances smoothness and trajectory quality |
| `iteration_count` | 1 | Optimization iterations per cycle |
| `control_period_ms` | 50 | Control period (ms), 50ms = 20Hz |
| `thread_count` | 4 | Parallel computation thread count |
| `prune_distance` | 3.5 | Path pruning distance (meters) |

### 8.2 Control Constraints

| Parameter | Default | Description |
| --- | --- | --- |
| `vx_max` | 1.2 | Maximum linear velocity (m/s) |
| `vx_min` | -0.25 | Minimum linear velocity (m/s), 0.0 disallows reverse |
| `vy_max` | 1.2 | Maximum lateral velocity (m/s), omnidirectional only |
| `wz_max` | 2.0 | Maximum angular velocity (rad/s) |
| `ax_max` | 1.5 | Maximum linear acceleration (m/s²) |
| `ay_max` | 1.35 | Maximum lateral acceleration (m/s²) |
| `az_max` | 2.5 | Maximum angular acceleration (rad/s²) |

### 8.3 Noise Sampling Standard Deviations

| Parameter | Default | Description |
| --- | --- | --- |
| `vx_std` | 0.20 | Linear velocity noise std dev (m/s) |
| `vy_std` | 0.20 | Lateral velocity noise std dev (m/s), set 0.0 for diff drive |
| `wz_std` | 0.30 | Angular velocity noise std dev (rad/s) |

### 8.4 Obstacle Cost Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `obstacle_repulsion_weight` | 1.0 | Obstacle repulsion weight |
| `obstacle_collision_cost` | 100000.0 | Collision trajectory cost |
| `obstacle_collision_margin` | 0.3 | Collision boundary distance (m) |
| `collision_cost_threshold` | 50000.0 | Collision cost threshold |
| `obstacle_inflation_radius` | 0.8 | Obstacle inflation radius (m) |
| `obstacle_cost_scaling` | 4.0 | Cost scaling factor |
| `obstacle_near_goal_distance` | 0.3 | Distance to ignore obstacles near goal |
| `robot_radius` | 0.25 | Robot radius (m) |
| `consider_footprint` | false | Whether to consider robot footprint |
| `grid_resolution` | 0.05 | Distance field resolution (m/cell) |
| `grid_width` | 100 | Grid map width |
| `grid_height` | 100 | Grid map height |

### 8.5 Blocking Detection Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `twirling_weight` | 0.5 | Spinning penalty weight (soft constraint) |
| `twirling_threshold` | 0.5 | Goal distance threshold to disable TwirlingCritic |
| `spinning_ratio_threshold` | 8.0 | wz/vx ratio threshold (hard constraint) |
| `spinning_detect_frames` | 8 | Consecutive detection frames |

### 8.6 Startup Assist Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `startup_assist_enabled` | false | Enable startup assist |
| `startup_boost_vx` | 0.06 | Startup linear velocity boost value |
| `startup_boost_wz_gain` | 1.2 | Startup angular velocity gain coefficient |
| `startup_boost_wz_limit` | 0.6 | Startup angular velocity limit |
| `startup_cmd_vx_eps` | 0.01 | Startup command velocity threshold |
| `startup_speed_vx_eps` | 0.02 | Startup actual speed threshold |
| `startup_goal_distance` | 0.30 | Startup goal distance threshold |
| `startup_lookahead` | 8 | Startup lookahead steps |

### 8.7 Path Alignment Cost Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `path_align_weight` | 15.0 | Path alignment weight |
| `path_align_offset` | 4 | Steps to offset forward from furthest path point |
| `path_align_threshold` | 0.3 | Distance threshold (m) |
| `path_align_traj_step` | 1 | Trajectory point sampling step |
| `path_align_obstacle_check_radius` | 0.15 | Path alignment obstacle check radius |
| `path_align_max_occupancy_ratio` | 0.45 | Path blocking ratio threshold |
| `path_align_use_orientations` | false | Whether to consider path headings |

### 8.8 Path Angle Cost Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `path_angle_weight` | 5.0 | Path angle weight |
| `path_angle_offset` | 4 | Steps to offset forward from furthest path point |
| `path_angle_threshold` | 0.3 | Distance threshold (m) |
| `path_angle_max` | 0.15 | Maximum allowed angle deviation (rad) |
| `path_angle_mode` | 0 | Angle mode: 0=prefer forward, 1=no preference, 2=consider path heading |

### 8.9 Path Following Cost Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `path_follow_weight` | 8.0 | Path following weight |
| `path_follow_offset` | 4 | Steps to offset forward from furthest path point |
| `path_follow_threshold` | 0.4 | Distance threshold (m) |

### 8.10 Goal Cost Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `goal_weight` | 5.0 | Goal point weight |
| `goal_threshold` | 0.8 | Distance threshold to activate goal cost |
| `goal_angle_weight` | 3.0 | Goal angle weight |
| `goal_angle_threshold` | 0.6 | Distance threshold to activate goal angle cost |

### 8.11 Other Cost Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `prefer_forward_weight` | 3.0 | Prefer forward weight |
| `prefer_forward_threshold` | 0.5 | Distance threshold |
| `constraint_weight` | 3.0 | Constraint cost weight |
| `motion_model_type` | 0 | Motion model type: 0=diff, 1=omni, 2=ackermann |
| `velocity_deadband_weight` | 0.5 | Velocity deadband weight |
| `velocity_deadband_vx` | 0.05 | Linear velocity deadband threshold |
| `velocity_deadband_vy` | 0.05 | Lateral velocity deadband threshold |
| `velocity_deadband_wz` | 0.1 | Angular velocity deadband threshold |

### 8.12 SG Filter Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `use_sg_filter` | true | Enable SG filter |
| `shift_control_sequence` | false | Enable control sequence temporal smoothing |
| `retry_attempt_limit` | 2 | Fallback retry attempt limit |
| `use_mean_normalization` | false | Use mean normalization |

### 8.13 Motion Model Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `motion_model` | "DiffDrive" | Motion model: DiffDrive/Omni/Ackermann |
| `ackermann_min_turning_radius` | 0.3 | Ackermann minimum turning radius (m) |

### 8.14 Omni/Diff Auto-Switching Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `enable_omni_switching` | true | Enable omni/diff auto-switching |
| `omni_trigger_obstacle_dist` | 0.9 | Obstacle distance threshold to trigger omni mode |
| `omni_trigger_path_deviation` | 0.3 | Path deviation threshold to trigger omni mode |
| `diff_restore_path_threshold` | 0.20 | Path deviation threshold to restore diff mode |
| `omni_switch_delay_frames` | 3 | Mode switch delay frames |

### 8.15 Diff Mode Configuration Notes

When switching to diff mode, modify these key parameters:

```yaml
motion_model: "DiffDrive"          # Motion model set to diff
motion_model_type: 0               # Model type in constraint cost
vx_min: -0.0                       # Disallow reverse (or set negative to allow)
vy_std: 0.0                        # Lateral velocity noise set to 0
enable_omni_switching: false       # Disable omni switching (pure diff mode) hack part
consider_footprint: true           # Recommend enabling footprint detection
robot_radius: 0.35                 # Adjust according to actual robot size
```

***

## 9. Topics and Interfaces

### 9.1 Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/plan` | `nav_msgs/Path` | Global path to follow |
| `/odom` | `nav_msgs/Odometry` | Robot pose and velocity |
| `/scan` | `sensor_msgs/LaserScan` | Laser scan for obstacle detection |

### 9.2 Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | Output velocity command |
| `/debug_optimal_trajectory` | `nav_msgs/Path` | Optimal trajectory (for RViz visualization) |

### 9.3 Core C++ Interface

```cpp
// Initialize controller
mppi::MPPIController controller;
controller.initialize(settings, "DiffDrive", ackermann_radius);

// Set each Critic parameters
controller.getObstaclesCritic()->setParams(...);
controller.getPathAlignCritic()->setParams(...);
// ...

// Control loop
controller.setPath(path_poses);
controller.updateStaticObstacles(laser_points, robot_pose);
mppi::Twist2D cmd = controller.computeVelocityCommands(robot_pose, robot_speed);
```

***

## 10. Algorithm Details

### 10.1 Control Loop Flow

```
1. Receive laser scan → transform to global coordinates
2. Receive odometry → update robot pose and velocity
3. Receive global path → store path points
4. Timer trigger (50ms):
   a. Check goal arrival (< 0.20m)
   b. Update obstacles + path validity + motion model switching
   c. Compute velocity command:
      i.   Prune path (centered at robot, keep points within prune_distance)
      ii.  Iterative optimization:
           - Apply noise to generate K candidate trajectories
           - Multi-threaded forward simulation
           - All Critics scoring
           - Softmax weighted update control sequence
      iii. Collision detection and recovery
      iv.  Blocking detection
      v.   SG filtering
      vi.  Take first control
   d. Startup assist (optional)
   e. Publish velocity command
   f. Publish debug trajectory
```

### 10.2 Path Pruning

Nav2-style path pruning making index 0 correspond to robot's current position:

1. Find nearest path point to robot position
2. From that point, accumulate distance along path, keep points within `prune_distance`
3. Create pruned path

### 10.3 Furthest Reachable Path Point

Find the nearest point on the path for each sampled trajectory endpoint, take the maximum index as the furthest reachable path point. This index determines the reference point position for PathFollowCritic, PathAlignCritic, and PathAngleCritic.

### 10.4 Collision Recovery Mechanism

When all trajectories collide:

1. Set `fail_flag = true`
2. Attempt `fallback()`: reset control sequence, resample
3. Retry up to `retry_attempt_limit` times
4. After exceeding retries, throw exception — ROS1 node catches and outputs zero velocity

### 10.5 Omni/Diff Auto-Switching

Switch conditions:

| Direction | Condition | Action |
| --- | --- | --- |
| Diff→Omni | Nearest obstacle < `omni_trigger_obstacle_dist` OR path deviation > `omni_trigger_path_deviation` | Switch to OmniMotionModel |
| Omni→Diff | Nearest obstacle > `omni_trigger_obstacle_dist × 1.2` AND path deviation < `diff_restore_path_threshold` | Restore base motion model |

Switch execution:

1. `optimizer_->softReset()`: Reset control sequence
2. Create new motion model
3. `optimizer_->setMotionModel()`: Update motion model
4. Update ConstraintCritic parameters

Delay mechanism: Switch only after `omni_switch_delay_frames` consecutive frames meet conditions, preventing frequent switching.

### 10.6 Startup Assist

When robot speed is near zero and no obstacles present, apply initial velocity based on path direction:

$$v\_x = v\_{\text{boost}}, \quad \omega\_z = \text{clamp}(K\_p \cdot \Delta\theta, -\omega\_{\max}, \omega\_{\max})$$

***

## 11. Performance Optimization

### 11.1 Asynchronous Noise Pre-generation

Noise generation uses a separate thread running in parallel with the main control loop:

```
Timeline:
Main thread:   [use noise A] → [score with noise A] → [use noise B] → [score with noise B] → ...
Noise thread:  [generate noise A] → [generate noise B] → [generate noise C] → ...
```

While the main thread scores with current noise, the noise thread pre-generates the next batch, reducing main loop wait time.

### 11.2 Multi-threaded Trajectory Integration

Trajectory integration uses `std::thread` for parallel computation:

```cpp
// Distribute batch_size trajectories across thread_count threads
unsigned int batch_per_thread = (batch_size + thread_count - 1) / thread_count;
for (unsigned int t = 0; t < thread_count; ++t) {
    threads.emplace_back(worker, start, end);
}
```

Each thread independently computes forward simulation for its batch of trajectories, avoiding data races.

### 11.3 Grid Distance Field O(1) Query

After building the local BFS distance field centered at the robot, trajectory scoring requires only coordinate mapping for direct distance lookup — O(1) time complexity, eliminating brute force search through all obstacle points.

### 11.4 xt::noalias Optimization

Use `xt::noalias` in operations like noise addition to skip alias checking, improving xtensor assignment performance:

```cpp
xt::noalias(state.cvx) = xt::view(control_sequence.vx, xt::newaxis(), xt::all()) + noises_vx_;
```

### 11.5 Path Cumulative Distance Caching

PathAlignCritic uses path cumulative distance matching instead of brute force search, enabling early termination via monotonically increasing distance sequences:

```cpp
for (size_t i = start_idx + 1; i < path_distances.size(); ++i) {
    float diff = std::abs(path_distances[i] - target_distance);
    if (diff < min_diff) { min_diff = diff; closest_idx = i; }
    else if (diff > min_diff) { break; }  // Early termination
}
```

***

## 12. Differences from Nav2 Official MPPI Controller

### 12.1 Framework Dependencies

| Feature | This Project | Nav2 MPPI |
| --- | --- | --- |
| ROS2 Dependencies | None | Strong (rclcpp, nav2\_core, nav2\_costmap\_2d, etc.) |
| ROS1 Support | Native | Not yet supported |
| Parameter System | External setParams | ROS2 parameter server + ParametersHandler |
| Plugin Mechanism | Compile-time static registration | pluginlib dynamic loading |
| Lifecycle Management | None | LifecycleNode state machine |
| Code Organization | Modular multi-file | Inline implementation within plugin |

### 12.2 Obstacle Handling

| Feature | This Project | Nav2 MPPI |
| --- | --- | --- |
| Obstacle Source | Laser point cloud (direct input) | Costmap2D global costmap |
| Distance Query | Self-built local grid BFS distance field, can integrate RC-ESDF for zoned collision detection and risk-zone gradient-biased sampling | Costmap2D inflation layer + FootprintCollisionChecker |
| Dynamic Obstacles | Built-in linear prediction support | None (relies on costmap updates) |
| Footprint Detection | Self-implemented coordinate transform + grid query | nav2\_costmap\_2d::FootprintCollisionChecker |

### 12.3 Numerical Library

| Feature | This Project | Nav2 MPPI |
| --- | --- | --- |
| Matrix Operations | xtensor | Eigen |
| Random Generation | xt::random::randn | std::normal\_distribution |

### 12.4 Algorithm Enhancements (some hacks)

Additional features built on top of Nav2 MPPI:

| Enhancement | Description | Nav2 Status |
| --- | --- | --- |
| **Blocking Detection** | wz/vx ratio with consecutive frame detection, identifies spinning and outputs zero velocity | Not present |
| **Omni/Diff Auto-Switching** | Automatically switches motion models based on obstacle distance and path deviation | Manual configuration |
| **Adaptive Temperature** | Dynamically adjusts softmax temperature based on cost distribution | Fixed temperature |
| **Mean Normalization Option** | Mean normalization as alternative to min normalization | Min normalization only |
| **Path Point Validity Check** | PathFollowCritic can skip path points blocked by obstacles | Basic implementation |
| **Path Blocking Detection** | PathAlignCritic auto-disabled when path heavily blocked | Basic implementation |
| **Dynamic Obstacle Prediction** | ObstaclesCritic supports velocity-based dynamic obstacle position prediction | None (relies on costmap updates) |
| **Startup Assist** | Resolves zero-velocity startup issues / redundant, ineffective | Not present |
| **Arbitrary Shape Robot Collision Avoidance** | Builds RC-ESDF-based zoned collision detection; risk-zone analytical gradients bias MPPI sampling | Not present |

### 12.5 Critic Differences

| Critic | This Project | Nav2 MPPI |
| --- | --- | --- |
| CostCritic | None | Present (based on costmap values) |
| ObstaclesCritic | BFS grid distance field + RC-ESDF zoned collision detection + risk-zone gradient-biased sampling + dynamic prediction | Costmap inflation layer query |
| PathAlignCritic | Path blocking detection + heading consideration | Basic implementation |
| PathAngleCritic | Three angle modes | Three angle modes |
| VelocityDeadbandCritic | Present | Present |
| TwirlingCritic | Present (includes goal distance deactivation logic) | Present |

### 12.6 Interface Differences

| Feature | This Project | Nav2 MPPI |
| --- | --- | --- |
| Main Interface | `computeVelocityCommands(Pose2D, Twist2D)` returns `Twist2D` | `computeVelocityCommands(PoseStamped, Twist, Path)` returns `TwistStamped` |
| Path Handling | Integrated in `Optimizer::prunePath()` | Handled by separate PathHandler node |
| Parameter Setting | Via `setParams()` method | Via ROS2 parameter server |

### 12.7 Threading Model

| Feature | This Project | Nav2 MPPI |
| --- | --- | --- |
| Noise Generation | Separate thread + xt::random | Separate thread + std::normal\_distribution |
| Trajectory Integration | std::thread multi-threaded parallel | Eigen vectorization (single-threaded) |

### 12.8 Code Organization Differences

| Feature | This Project | Nav2 MPPI |
| --- | --- | --- |
| Responsibility Division | Single responsibility per file | Multiple classes inline in same file, plugin-based |
| Maintainability | High (modular independent, clear interfaces) | Medium (depends on understanding) |
| Compile Dependencies | Only xtensor | Eigen + Nav2 full stack |

***

## 13. License

Copyright © 2025 MPPI ROS1 Project

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.

***

## 14. References

### 14.1 MPPI Algorithm Papers

- Williams G, Drews P, Goldfain B, Rehg J M, Theodorou E A. **Information-Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving**[J]. IEEE Transactions on Robotics, 2018, 34(6): 1603-1622.
- Williams G, Aldrich A, Theodorou E A. **Model Predictive Path Integral Control: From Theory to Parallel Computation**[J]. Journal of Guidance, Control, and Dynamics, 2017, 40(2): 344-357.

### 14.2 Nav2 MPPI Controller

- Nav2 Official Documentation: <https://docs.nav2.org/>
- Nav2 MPPI Controller Source: <https://github.com/ros-planning/navigation2/tree/main/nav2_mppi_controller>

### 14.3 Third-party Libraries

- xtensor (header-only multi-dimensional array library): <https://github.com/xtensor-stack/xtensor>
- xsimd (C++ SIMD acceleration library): <https://github.com/xtensor-stack/xsimd>
- xtl (xtensor base template library): <https://github.com/xtensor-stack/xtl>

### 14.4 Related Blogs

- [MPPI Local Path Planning Controller — Modular ROS1 Implementation Decoupled from Nav2](https://blog.csdn.net/qq_56908984/article/details/158774722?sharetype=blogdetail\&sharerId=158774722\&sharerefer=PC\&sharesource=qq_56908984\&spm=1011.2480.3001.8118)
