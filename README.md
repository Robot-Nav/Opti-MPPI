# DC-PMPPI Disturbance-Compensated Model Predictive Path Integral Controller

![License](https://img.shields.io/badge/License-Apache--2.0-blue.svg)
![Language](https://img.shields.io/badge/Language-C%2B%2B17-orange.svg)
![ROS](https://img.shields.io/badge/ROS-Noetic-22314E.svg)
![Build](https://img.shields.io/badge/Build-catkin-green.svg)
![Parallel](https://img.shields.io/badge/Parallel-OpenMP-red.svg)
![Vectorization](https://img.shields.io/badge/Vectorization-xtensor%20%2B%20xsimd-yellow.svg)
![Platform](https://img.shields.io/badge/Platform-Linux-lightgrey.svg)
![Status](https://img.shields.io/badge/Status-Release-brightgreen.svg)

> [👉 README_CN ](https://github.com/Robot-Nav/SA-MPPI/blob/main/README_CN.md)
> 
> A ROS1 local path-tracking controller designed for low-adhesion and externally disturbed conditions such as slippery roads and oscillating decks.  
> On top of the standard MPPI sampling-based optimization framework, it introduces equivalent disturbance estimation and control-effectiveness identification so that the prediction model remains consistent with the actual controlled system, significantly improving trajectory-tracking accuracy under slip conditions while preserving real-time computation.

---

## Table of Contents

- [1. Project Overview](#1-project-overview)
- [2. Algorithm Principles](#2-algorithm-principles)
  - [2.1 Review of the MPPI Control Framework](#21-review-of-the-mppi-control-framework)
  - [2.2 Core Improvements of DC-PMPPI](#22-core-improvements-of-dc-pmppi)
  - [2.3 Overall System Data Flow](#23-overall-system-data-flow)
- [3. Algorithm Formulation](#3-algorithm-formulation)
  - [3.1 Path-Integral Control Law](#31-path-integral-control-law)
  - [3.2 Rollout Dynamics with Disturbances](#32-rollout-dynamics-with-disturbances)
  - [3.3 Online Disturbance Estimation](#33-online-disturbance-estimation)
  - [3.4 Control-Effectiveness Identification](#34-control-effectiveness-identification)
  - [3.5 MPCC-Style Tracking Cost](#35-mpcc-style-tracking-cost)
  - [3.6 Adaptive Temperature and Elite Truncation](#36-adaptive-temperature-and-elite-truncation)
  - [3.7 Kinematic Constraints](#37-kinematic-constraints)
- [4. Project Structure](#4-project-structure)
  - [4.1 Directory Structure](#41-directory-structure)
  - [4.2 Core Module Description](#42-core-module-description)
  - [4.3 Key Parameter Description](#43-key-parameter-description)
- [5. Running Instructions](#5-running-instructions)
- [6. Project Dependencies](#6-project-dependencies)
- [7. Extensions and Ablation Experiments](#7-extensions-and-ablation-experiments)

---

## 1. Project Overview

`dc_pmppi` stands for **Disturbance-Compensated Model Predictive Path Integral Control**. It is a local path-tracking controller for wheeled mobile robots, including differential-drive, omnidirectional, and Ackermann platforms.

Standard MPPI obtains an optimal control command by Gaussian sampling of the control sequence, rollout simulation, and weighted averaging. It does not require a differentiable cost function and naturally handles nonlinear models and non-convex constraints. However, it implicitly assumes that the **prediction model is consistent with the real controlled system**. Once this assumption is violated, for example:

- Tire cornering stiffness decreases on wet, snowy, or slippery roads, so commanded velocity is no longer equal to actual velocity;
- Shipborne or deck-mounted robots are subjected to externally forced body motion caused by deck oscillation;
- Systematic odometry or chassis bias acts as a persistent equivalent external disturbance;

the trajectories predicted by MPPI develop a systematic deviation from the real robot trajectory. The controller is then unable to actively correct this mismatch within the prediction horizon, which can lead to path offset, overshoot, or even instability.

The engineering objectives of DC-PMPPI are:

1. Online identification of the equivalent body-frame lateral slip velocity `v_y^dist`, yaw disturbance rate `ω^dist`, and longitudinal/yaw control-effectiveness coefficients `η_v, η_ω`;
2. Injection of these estimates into the MPPI rollout model, reference-control generation, and cost function so that predicted trajectories more closely match the real controlled system;
3. Use of confidence weighting and a disturbance-decay time constant to prevent transient estimation errors, outliers, or dropped frames from contaminating the control law.

When `vy_dist = 0, wz_dist = 0`, DC-PMPPI automatically reduces to standard MPPI, making A/B comparison experiments straightforward.

---

## 2. Algorithm Principles

### 2.1 Review of the MPPI Control Framework

MPPI transforms the optimal control problem into a path-integral form. Consider the discrete dynamics:

```text
x_{t+1} = F(x_t, u_t)
```

with the cost function:

```text
S(x_{0:T}, u_{0:T}) = Σ_{t=0}^{T-1} q(x_t, u_t) + φ(x_T)
```

The control law is obtained by exponentially weighted averaging of sampled trajectories (Williams et al., 2017):

```text
u* = Σ_i w_i · u_i,   w_i = exp(-1/λ · (S_i - min S))
```

Each control cycle performs five steps: **sampling → rollout → scoring → weighted update → sequence shift**. Warm start is used to maintain temporal smoothness.

### 2.2 Core Improvements of DC-PMPPI

This project introduces five key improvements over standard MPPI.

**(1) Equivalent Disturbance Estimation**

By comparing pose-difference velocities from odometry with previously executed commands, the controller estimates body-frame lateral residual velocity `v_y^dist` and yaw-rate residual `ω^dist`. These residuals provide a unified representation of model mismatch caused by lateral slip, externally forced motion, localization bias, and related effects, avoiding the need to model every disturbance source separately.

**(2) Control Effectiveness `η_v / η_ω`**

On low-adhesion surfaces, commanded motion is attenuated before appearing in the measured response. The following relationship is introduced:

```text
v_actual = η_v · v_cmd,   ω_actual = η_ω · ω_cmd
```

The control-effectiveness coefficient `η` is identified online using exponentially weighted Welford regression. This explicitly embeds the “discounted” actuator response into the rollout model instead of repeatedly penalizing the effect in the cost function.

**(3) Disturbance-Decay Injection**

The disturbance is not assumed to remain constant. Its future influence decays according to a time constant `τ`:

```text
d_t = d_0 · exp(-t / τ)
```

Thus, the rollout gradually returns to the nominal model farther into the prediction horizon, avoiding excessive accumulation and amplification of long-horizon disturbance-estimation error.

**(4) Adaptive Temperature + ESS Target**

The MPPI temperature parameter `λ` controls the exploration-exploitation tradeoff. A fixed `λ` may behave very differently under different cost scales. This project uses a **target Effective Sample Size (ESS)** and adjusts `λ` through binary search, improving robustness to changes in cost-function scale.

**(5) Elite Truncation (CEM-Style) + Antithetic Sampling**

The controller retains MPPI-style relative cost weighting within an elite subset while discarding long-tail low-quality samples. Antithetic sampling is also used to reduce Monte Carlo variance.

### 2.3 Overall System Data Flow

```text
┌────────────────┐   /plan    ┌──────────────────┐
│ Global Planner │ ─────────> │  pathCallback     │
└────────────────┘            │  · Resampling     │
                              │  · Deduplication   │
                              │  · Tangent heading │
┌────────────────┐   /odom1   └────────┬─────────┘
│ Odometry/SLAM  │ ─────────┐          │
└────────────────┘          ▼          │
                  ┌──────────────────┐  │
                  │  odomCallback     │ │
                  │  · Pose/velocity  │ │
                  │    buffering      │ │
                  │  · Disturbance    │ │
                  │    estimator      │ │
                  └────────┬─────────┘ │
                           │           │
                           ▼           ▼
              ┌──────────────────────────────┐
              │      controlLoop (50 ms)      │
              │  1. Goal/reacquisition check │
              │  2. Confidence-weighted      │
              │     disturbance injection     │
              │  3. MPPI optimizer            │
              │     evalControl               │
              │     · Path pruning +          │
              │       reference-speed preview │
              │     · Gaussian + guided       │
              │       sampling                │
              │     · Rollout with            │
              │       disturbance decay       │
              │     · Critic scoring          │
              │     · Softmax weighted update │
              │     · Savitzky-Golay smoothing│
              │  4. GoalDocking terminal      │
              │     stabilizer                │
              │  5. enforceMotionConstraints │
              └────────────┬─────────────────┘
                           │
                           ▼
                    /cmd_vel, /local_path,
                    /controller_solve_time_ms
```

---

## 3. Algorithm Formulation

### 3.1 Path-Integral Control Law

Let the current control sequence be `U = (u_0, u_1, ..., u_{T-1})`. Gaussian noise `ε_i,t ~ N(0, Σ)` is added to obtain `N` sampled sequences `U_i = U + ε_i`. The cumulative cost of each sequence is:

```text
S_i = φ(x_T^i) + Σ_{t=0}^{T-1} [ q(x_t^i, u_t^i)
        + γ/2 · u_t^iᵀ · Σ⁻¹ · ε_{i,t} ]
```

where `γ` is the control-cost discount factor. The second term is the MPPI control-energy regularization term, which suppresses unnecessary high-frequency oscillation.

The control sequence is updated according to the information-theoretic optimal update law:

```text
U ← U + Σ_i w_i · ε_i,   w_i = (1/Z) · exp(-1/λ · (S_i - S_min))
```

where `Z = Σ_i exp(-1/λ · (S_i - S_min))` is the normalization constant. The project additionally supports:

- `use_mean_normalization`: use the mean cost instead of the minimum cost as the reference point;
- `elite_sample_count > 0`: retain only Top-K samples for weighting, i.e. CEM-style truncation;
- `elite_update_rate`: elite-mean update step size controlling convergence speed.

### 3.2 Rollout Dynamics with Disturbances

For nonholonomic models such as differential-drive and Ackermann vehicles, the nominal rollout model is:

```text
x_{t+1} = x_t + (v·cosθ)·dt
y_{t+1} = y_t + (v·sinθ)·dt
θ_{t+1} = θ_t + ω·dt
```

DC-PMPPI injects lateral slip and yaw disturbance into the model:

```text
θ_{t+1} = θ_t + (ω_cmd·η_ω + ω^dist·e^{-t/τ})·dt
x_{t+1} = x_t + (v·cosθ - v_y^dist·e^{-t/τ}·sinθ)·dt
y_{t+1} = y_t + (v·sinθ + v_y^dist·e^{-t/τ}·cosθ)·dt
```

Control effectiveness is introduced at the prediction-model level:

```text
v_realized = η_v · v_cmd,   ω_realized = η_ω · ω_cmd
```

Acceleration constraints are then applied:

```text
|v_realized - v_prev| ≤ a_max·dt
```

to maintain physical feasibility.

### 3.3 Online Disturbance Estimation

Let the body-frame longitudinal, lateral, and yaw velocities obtained from pose differencing be `v_x^meas, v_y^meas, ω^meas`. Let the executed commands after first-order lag compensation be `v_x^cmd, ω^cmd`. The expected yaw rate is:

```text
ω^exp = η_ω · ω^cmd
```

The disturbance residuals are:

```text
ω^dist_obs = ω^meas - ω^exp
v_y^dist_obs = v_y^meas
```

An exponential low-pass filter with time constant `τ_f` is used:

```text
v_y^dist ← β·v_y^dist + (1-β)·clip(v_y^dist_obs - v_y^dist, [-Δ_max, Δ_max])
```

where `β = exp(-dt/τ_f)` and `Δ_max` limits the single-step innovation so that localization jumps do not contaminate the estimate.

The variance is recursively updated at the same time:

```text
σ²_vy ← β·σ²_vy + (1-β)·(Δ_vy)²
```

and can be used for optional disturbance-uncertainty sigma-point sampling.

### 3.4 Control-Effectiveness Identification

The project uses either exponentially weighted centered Welford regression or a simple ratio method.

**Ratio method** (default):

```text
η_v_obs = clip(v_x^meas / v_x^cmd, 0.3, 1.0)   if |v_x^cmd| > deadzone
η_ω_obs = clip(ω^meas / ω^cmd, 0.3, 1.0)       if |ω^cmd| > deadzone
```

**Centered regression method**:

```text
cov(v_cmd, v_meas) / var(v_cmd)
```

Centering removes the influence of slowly varying additive disturbances on slope estimation.

The coefficient `η` is fused using a slow low-pass filter with time constant `τ_η` of approximately 1.5 s, preventing short-term drift from being misinterpreted as a loss of adhesion.

The **traction capability indicator** `traction_scale` is defined as:

```text
severity = k_1·|v_y^dist| + k_2·|ω^dist| + 0.2·(σ_vy + σ_ω)
traction_scale = clip(1 - severity, τ_min, 1.0)
```

`traction_scale` is not a physical friction coefficient. It is a conservative capability indicator in the range `[0,1]`, used to dynamically tighten speed and acceleration limits:

```text
v_max ← v_max · traction_scale^α_v
a_max ← a_max · traction_scale^α_a
```

### 3.5 MPCC-Style Tracking Cost

Following the Model Predictive Contouring Control idea, path arc length `s` is treated as a virtual progress state. For each rollout trajectory:

```text
e_c = signed_lateral_error                  # contouring error (lateral distance)
e_l = s_exp - s_proj                        # lag error (progress difference)
e_h = wrap(θ_traj - (θ_path - β_slip))      # heading error with slip-angle compensation
e_v = v_traj - v_ref(s)                     # speed-tracking error
e_ω = (ω_traj + ω^dist·decay) - v_ref·κ     # yaw-rate error
```

where

```text
β_slip = atan2(v_y^dist·decay, max(v_ref, 0.1))
```

is the body slip-angle compensation. This makes the desired heading account for the actual direction of motion under slip.

The cost is:

```text
J_i = Σ_t w(t) · (q_c·e_c² + q_l·e_l² + q_h·e_h² + q_v·e_v² + q_ω·e_ω²)
     + w_term · (q_pos·|p_T - p_goal|² + q_yaw·Δθ_goal² + q_vel·v_T²)
```

with

```text
w(t) = 1 + t/(T-1)
```

used as a time-dependent weight that emphasizes the terminal part of the horizon. The terminal term is activated according to `goal_blend` only when the prediction horizon covers the global goal.

**Reference-speed preview** considers three constraints: curvature-based speed limiting, preview deceleration before a corner, and braking before the goal:

```text
v_ref(s) = smooth_min(v_cruise,
                      sqrt(a_lat_max / |κ|),
                      sqrt(2·a_goal·d_goal),
                      sqrt(v_corner² + 2·a_corner·d_corner))
```

`smooth_min` is a smooth minimum operator with a transition region, avoiding control oscillation caused by hard switching.

### 3.6 Adaptive Temperature and Elite Truncation

**ESS-targeted temperature search**:

```text
ESS(T) = (Σ w_i)² / Σ w_i²,   w_i = exp(-(S_i - S_min)/T)
```

Binary search is used to find the temperature `T` satisfying:

```text
ESS(T) = target_ess_ratio · N
```

with `T` constrained to `[T_min, T_max]`. The temperature therefore adapts automatically to cost-scale changes without manual retuning.

**CEM-style elite truncation** retains only Top-K samples and discards the long tail. MPPI relative-cost weighting is still used inside the elite subset rather than equal-weight averaging as in standard CEM:

```text
w_i = 0                                   if i not in Top-K
w_i = exp(-(S_i - S_min)/T) / Z           otherwise
```

### 3.7 Kinematic Constraints

**Differential-drive model:** directly limits `|v_x| ≤ v_max` and `|ω| ≤ ω_max`, with independent acceleration constraints `|Δv_x| ≤ a_x·dt` and `|Δω| ≤ a_z·dt`.

**Omnidirectional model:** the combined speed `v = hypot(v_x, v_y)` is constrained, with `v_y` treated as an independent active control channel.

**Ackermann model:** after acceleration propagation, each state is projected back into the feasible set:

```text
|ω| ≤ min(|v| / R_min, a_lat_max / |v|)
```

This ensures that predicted states satisfy both the minimum turning-radius constraint and the lateral-acceleration constraint, preventing instantaneous infinite curvature during startup or braking.

---

## 4. Project Structure

### 4.1 Directory Structure

```text
dc_pmppi/
├── CMakeLists.txt                      # catkin build configuration
├── package.xml                         # ROS package description and dependencies
├── config/
│   ├── dc_pmppi_params.yaml            # all tunable controller parameters
│   └── dc_pmppi_test.rviz              # RViz visualization configuration
├── include/
│   ├── controller.hpp                  # external controller interface
│   ├── optimizer.hpp                   # MPPI optimizer core
│   ├── motion_models.hpp               # three kinematic models
│   ├── critics/                        # cost-function collection
│   │   ├── critic_function.hpp         # base critic class
│   │   ├── critic_manager.hpp          # critic manager with statistics
│   │   ├── critic_data.hpp             # scoring-data transfer structure
│   │   ├── mpcc_tracking_critic.hpp    # MPCC-style tracking cost (core)
│   │   ├── control_rate_critic.hpp     # control-rate smoothing cost
│   │   ├── constraint_critic.hpp       # kinematic-constraint violation cost
│   │   ├── stability_critic.hpp        # low-adhesion stability cost
│   │   ├── path_align_critic.hpp       # path-alignment cost
│   │   ├── path_angle_critic.hpp       # path-angle cost
│   │   ├── path_follow_critic.hpp      # path-following cost
│   │   ├── lateral_error_critic.hpp    # LQR optimal lateral-error cost
│   │   ├── prefer_forward_critic.hpp   # forward-motion preference cost
│   │   ├── twirling_critic.hpp         # in-place rotation suppression
│   │   └── velocity_deadband_critic.hpp# velocity-deadband cost
│   ├── models/                         # data structures
│   │   ├── types.hpp                   # Pose2D / Twist2D / Control / DisturbanceEstimate
│   │   ├── constraints.hpp             # control constraints and sampling-noise parameters
│   │   ├── state.hpp                   # sampled-state container
│   │   ├── trajectories.hpp            # trajectory container
│   │   ├── control_sequence.hpp        # control sequence
│   │   ├── path.hpp                    # global path
│   │   └── optimizer_settings.hpp      # complete optimizer settings
│   └── tools/                          # utility modules
│       ├── math_utils.hpp              # math constants and angle/clamp utilities
│       ├── noise_generator.hpp         # Gaussian noise generator (AR/KMPPI/antithetic)
│       ├── disturbance_estimator.hpp   # online disturbance/effectiveness estimator
│       ├── path_projection.hpp         # point-to-path projection and arc-length calculation
│       ├── path_reference.hpp          # path reference state and speed preview
│       └── goal_docking.hpp            # terminal pose stabilizer
├── launch/
│   ├── j15_dc_pmppi.launch             # top-level launch (with RViz/DOT options)
│   ├── j15_dc_pmppi_node.launch        # node launch
│   ├── j15_path.launch                 # path publisher
│   └── j15_sim.launch                  # simulation launch
├── scripts/
│   ├── JZJ_path.py                     # path-generation script
│   └── twist_to_ackermann.py           # Twist → Ackermann conversion
├── src/
│   └── dc_pmppi_node.cpp               # ROS node entry point
├── test/
│   └── disturbance_estimator_test.cpp  # disturbance-estimator unit tests
├── xsimd/                              # embedded SIMD acceleration library
└── xtensor/                            # embedded tensor-computation library
```

### 4.2 Core Module Description

#### `controller.hpp` — External Controller Interface

This module encapsulates the optimizer, motion model, and critic manager, and exposes:

- `initialize(settings, motion_model_type, ackermann_radius)`: selects the motion model from a string and registers default critics;
- `setPath(path)`: updates the global path, automatically detects path changes, and resets the optimizer when needed;
- `setDisturbanceEstimate(...)`: injects an external disturbance estimate, which is a core DC-PMPPI interface;
- `computeVelocityCommands(pose, speed)`: called by the main loop and returns the optimal `Twist2D`;
- `enforceMotionConstraints(cmd)`: reapplies motion constraints to the terminal stabilizer output;
- getters such as `getOptimizedTrajectory`, `getEffectiveSampleSize`, `getLastMinCost/MaxCost`, and `getCriticStatistics`.

#### `optimizer.hpp` — MPPI Optimizer Core

This module implements the complete optimization loop:

1. `prepare`: path pruning, reference-control sequence generation, and constraint scaling;
2. `optimize`: noise-trajectory generation → critic scoring → weighted update → constraint enforcement;
3. `evalControl`: multiple optimization iterations + early stopping when cost change falls below a threshold + Savitzky-Golay filtering;
4. `getControlFromSequence`: extracts the first action from the control sequence and applies heading-alignment gating, corner alignment, goal braking, curvature-consistency preservation, and other post-processing.

Path pruning uses a three-part strategy consisting of a **local search window + physical distance + heading matching**, preventing false nearest-point matches at hairpin turns or other large heading changes. `furthest_reached_quantile` uses a quantile rather than the mean to determine the path progress reached by the trajectory endpoint, making it robust to a small number of abnormal rollouts.

#### `motion_models.hpp` — Three Kinematic Models

- `DiffDriveMotionModel`: differential-drive nonholonomic model;
- `OmniMotionModel`: omnidirectional motion with active `v_y` control;
- `AckermannMotionModel`: Ackermann steering with minimum turning-radius and lateral-acceleration constraints, projecting every state back into the feasible set during `predict`.

All models share the same `predict` interface. The first step uses the current measured velocity, and subsequent steps propagate the state according to **control effectiveness × noisy control input**, followed by acceleration constraints.

#### `critics/` — Cost-Function Collection

Each critic inherits from `CriticFunction` and implements `score(CriticData&)`, adding its contribution to `data.costs`. Two core critics are enabled by default:

- **`MPCCTrackingCritic`**: integrated MPCC-style tracking cost including contouring, lag, heading, speed, yaw-rate, and terminal terms. `thread_count` controls OpenMP parallel scoring.
- **`ControlRateCritic`**: control-rate cost penalizing `(Δv_x/Δt)²` and `(Δω/Δt)²` to smooth the command sequence.

Other critics are optional and can be enabled according to the application:

- `ConstraintCritic`: hard-constraint violation cost, including Ackermann turning-radius violations;
- `StabilityCritic`: low-adhesion stability cost that constrains combined acceleration and yaw acceleration;
- `PathAlignCritic` / `PathFollowCritic` / `PathAngleCritic`: conventional Nav2-style path alignment, path following, and path-angle costs;
- `LateralErrorCritic`: optimal lateral-error weighting based on an LQR Riccati solution;
- `PreferForwardCritic` / `TwirlingCritic` / `VelocityDeadbandCritic`: forward-motion preference, in-place rotation suppression, and velocity-deadband costs.

`CriticManager` supports per-critic cost-contribution and runtime statistics through the `collect_critic_statistics` switch, helping identify tuning and performance bottlenecks.

#### `tools/disturbance_estimator.hpp` — Online Disturbance and Effectiveness Estimator

This is a core DC-PMPPI module. It implements:

- pose-difference velocity calculation, with optional direct use of odometry twist;
- first-order actuator-lag compensation, preventing motor response delay from being mistaken for reduced adhesion;
- centered Welford regression or robust ratio-based identification of `η_v, η_ω`;
- exponential low-pass filtering with innovation clipping for disturbance estimation;
- synchronized variance recursion for optional sigma-point disturbance sampling;
- confidence estimation based on innovation magnitude and the number of accumulated samples.

#### `tools/noise_generator.hpp` — Noise Generator

The module supports multiple mutually protected sampling strategies:

- **White noise + AR(1) temporal correlation**: `noise_correlation` β controls temporal correlation; β=0 gives white noise and β→1 gives a smoother sequence;
- **KMPPI-style RBF kernel interpolation**: noise is optimized at a small number of support points and expanded over the complete horizon using an RBF kernel, reducing optimization dimensionality;
- **Control-derivative sampling (Smooth-MPPI)**: noise is applied to control increments and integrated to obtain smooth absolute control;
- **Antithetic sampling**: positive/negative perturbation pairs reduce variance;
- **Nominal sample preservation**: `nominal_sample_count` zero-noise samples ensure that the current control sequence is not completely excluded from the sample set;
- **Cross-cycle sequence shifting**: `reuse_noise_sequence` reuses low-frequency noise from the previous control cycle and injects only orthogonal innovations, reducing random jumps between consecutive cycles.

#### `tools/path_projection.hpp` / `path_reference.hpp` — Path Geometry Utilities

- `computePathArcLengths`: computes cumulative path arc length;
- `projectPointToPath`: projects a point onto the piecewise-linear path and returns the nearest point, signed lateral error, arc length, and tangent heading;
- `samplePathReference`: samples reference states along arc length, including position, heading, and curvature;
- `previewReferenceSpeed`: computes reference speed under curvature limits, preview corner deceleration, and goal braking;
- `smoothMinimum`: smooth minimum operator with a transition region.

#### `tools/goal_docking.hpp` — Terminal Pose Stabilizer

Inside the goal `activation_distance`, this module gradually takes over control by blending the nominal MPPI command with pose-stabilization feedback:

- distance-speed model: `v = min(v_max, k_d·d, sqrt(2·a_goal·d))`;
- bearing feedback coupled with terminal heading;
- a “reacquisition” mode triggered when velocity is low while the goal distance is still not satisfied, allowing a larger speed to avoid stagnation near the goal;
- slip-angle compensation and `η` scaling to maintain accurate docking under slippery conditions.

### 4.3 Key Parameter Description

The complete parameter set is given in [`config/dc_pmppi_params.yaml`](config/dc_pmppi_params.yaml). The following table lists the parameters with the greatest influence on performance:

| Parameter | Default | Description |
|---|---:|---|
| `batch_size` | 400 | Number of sampled trajectories per cycle; a robust Gazebo baseline that can be reduced proportionally if timing overruns occur |
| `time_steps` | 40 | Number of prediction steps; 40×0.05 s = 2.0 s horizon |
| `model_dt` | 0.05 s | Model discretization step |
| `temperature` | 0.60 | Softmax temperature `λ`; used as the initial value when `adaptive_temperature` is enabled |
| `noise_correlation` | 0.85 | AR(1) temporal noise correlation; 0 = white noise |
| `antithetic_sampling` | true | Antithetic sampling to reduce Monte Carlo variance |
| `guided_sampling_ratio` | 0.25 | 25% of rollouts use a curvature-feedforward guided distribution |
| `reuse_noise_sequence` | true | Reuses low-frequency noise across cycles to suppress cycle-to-cycle jumps |
| `prune_distance` | 3.5 m | Path-pruning distance; only path points within this range ahead of the vehicle are retained |
| `motion_model` | `"Ackermann"` | Motion-model selection: DiffDrive / Omni / Ackermann |
| `ackermann_min_turning_radius` | 0.50 m | Minimum Ackermann turning radius |
| `path_reference_speed` | 0.60 m/s | Nominal cruise speed |
| `path_max_lateral_acceleration` | 0.60 m/s² | Lateral-acceleration limit used for curvature-based speed limiting |
| `path_corner_preview_distance` | 1.20 m | Corner preview distance |
| `mpcc_contour_weight` | 2.2 | Contouring-error weight for stronger geometric tracking |
| `dc_pmppi_enabled` | false | Master DC-PMPPI switch; when false, the controller reduces to standard MPPI |
| `dc_pmppi_filter_time_constant` | 0.20 s | Disturbance-estimation low-pass time constant |
| `dc_pmppi_effectiveness_time_constant` | 1.50 s | Time constant for `η` control-effectiveness identification |
| `dc_pmppi_confidence_time_constant` | 0.50 s | Confidence ramp-up time constant |
| `dc_pmppi_compensation_gain` | 0.85 | Overall compensation gain used to suppress over-compensation caused by multiple injection points |
| `disturbance_decay_time_constant` | 0.8 s | Disturbance-decay time constant `τ` in rollouts |
| `min_traction_scale` | 0.35 | Lower bound of the traction-capability indicator |

---

## 5. Running Instructions

### 5.1 Build

Run the following commands from the root of the catkin workspace:

```bash
cd ~/catkin_ws
catkin_make --pkg dc_pmppi
source devel/setup.bash
```

To enable OpenMP acceleration, make sure `libomp-dev` is installed. CMake detects and links OpenMP automatically.

### 5.2 Launch the Controller

```bash
# 1) Launch the simulator and odometry source
#    (j15_sim.launch is included in the project)
roslaunch dc_pmppi j15_sim.launch

# 2) Launch the DC-PMPPI controller
#    dc_pmppi_enabled=false by default, which gives the standard MPPI baseline
roslaunch dc_pmppi j15_dc_pmppi.launch

# 3) Enable DC-PMPPI disturbance compensation for A/B experiments
roslaunch dc_pmppi j15_dc_pmppi.launch dc_pmppi_enabled:=true

# 4) Publish the global path
roslaunch dc_pmppi j15_path.launch
```

### 5.3 Topic Interfaces

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribe | `/plan` | `nav_msgs/Path` | Global path; its frame must match `planning_frame` |
| Subscribe | `/odom1` | `nav_msgs/Odometry` | Robot pose and velocity |
| Subscribe | `/constrained_cmd_vel_stamped` | `geometry_msgs/TwistStamped` | Actually executed command, optionally used for disturbance estimation |
| Publish | `/cmd_vel` | `geometry_msgs/Twist` | Control command |
| Publish | `/local_path` | `nav_msgs/Path` | Optimizer-predicted trajectory for visualization |
| Publish | `/controller_solve_time_ms` | `std_msgs/Float64` | Per-cycle solve time |

### 5.4 Runtime Tuning

Parameters can be adjusted at runtime with `rqt_reconfigure` or directly through `rosparam set`:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

Pay particular attention to:

- `/controller_solve_time_ms` should remain below `control_period_ms`, whose default is 50 ms;
- ESS, shown as `ESS=` in the log, is recommended to remain around 5%–30% of `batch_size`;
- when `collect_critic_statistics` is enabled, the log reports the mean contribution and runtime of each critic.

### 5.5 Unit Tests

```bash
catkin_make run_tests_dc_pmppi
```

The tests mainly verify `DisturbanceEstimator` convergence under step and sinusoidal disturbances, correctness of the variance recursion, and the confidence-transition behavior.

---

## 6. Project Dependencies

### 6.1 System Dependencies

| Dependency | Version Requirement | Purpose | Installation |
|---|---|---|---|
| ROS | Noetic | Node framework and message interfaces | apt |
| catkin | - | Build system | Installed with ROS |
| C++ | C++17 | Language standard | Provided by compiler |
| CMake | ≥ 3.1 | Build configuration | apt |
| OpenMP | ≥ 4.5 | Parallel sampling and scoring | apt `libomp-dev` |
| Eigen3 | ≥ 3.3 | Geometric computation | apt `libeigen3-dev` |

### 6.2 ROS Package Dependencies

Declared in `package.xml`:

- `roscpp` / `rospy` — ROS communication;
- `nav_msgs` — Path / Odometry messages;
- `geometry_msgs` — Twist / TwistStamped / Pose;
- `std_msgs` — basic message types such as Float64;
- `sensor_msgs` — sensor messages, indirectly required;
- `tf` — coordinate transformations;
- `ackermann_msgs` — Ackermann command messages;
- `onboard_detector` — optional static/dynamic obstacle detector (`exec_depend`).

### 6.3 Embedded Third-Party Libraries

To reduce the burden of external dependency management, the following libraries are embedded in the package source tree:

- **`xtensor/`** — C++ tensor-computation library providing NumPy-style `xtensor<float, N>` multidimensional arrays and vectorized mathematical operations. All batched sampling, rollout, and cost computation are implemented with xtensor, with optional compile-time SIMD optimization.
- **`xsimd/`** — C++ SIMD abstraction library used by xtensor for low-level AVX/SSE/NEON vectorization.

The embedded include paths are explicitly specified in `CMakeLists.txt` through `XTENSOR_INCLUDE_DIRS`:

```cmake
set(XTENSOR_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/xtensor/include
    ${CMAKE_CURRENT_SOURCE_DIR}/xsimd/include
    ${CMAKE_CURRENT_SOURCE_DIR}/xtl/include
)
```

No separate installation of xtensor or xsimd is required.

### 6.4 Python Dependencies

`scripts/JZJ_path.py` and `scripts/twist_to_ackermann.py` depend only on `rospy` and `numpy`, both of which are normally available with ROS Noetic.

---

## 7. Extensions and Ablation Experiments

The project retains multiple optional modules for ablation studies and research reproduction:

| Configuration | Behavior When Disabled | Typical Use |
|---|---|---|
| `dc_pmppi_enabled` | Reduces to standard MPPI | A/B comparison of disturbance compensation |
| `adaptive_temperature` | Uses fixed temperature `λ` | Evaluate robustness of ESS-targeted temperature adaptation |
| `elite_sample_count` | Uses all-sample MPPI weighting | Ablation of CEM-style truncation |
| `antithetic_sampling` | Uses independent identically distributed Gaussian sampling | Evaluate variance reduction |
| `noise_correlation` | Uses white-noise sampling | Temporal-correlation ablation |
| `sampling_support_points` | Uses independent noise over the full horizon | KMPPI kernel-interpolation ablation |
| `reuse_noise_sequence` | Resamples from scratch every cycle | Cross-cycle correlation ablation |
| `use_sg_filter` | Disables control-sequence post-processing smoothing | Savitzky-Golay filtering ablation |
| `guided_sampling_ratio` | Uses pure Gaussian sampling | Curvature-feedforward guidance ablation |
| `sample_disturbance_uncertainty` | Injects only the mean disturbance | Sigma-point disturbance-sampling ablation |
| `dc_pmppi_use_centered_effectiveness_regression` | Uses the robust ratio method | Compare `η` identification methods |

By toggling these parameters individually, the project can reproduce the behavior of standard MPPI, CEM-MPPI, Smooth-MPPI, KMPPI, RMPPI, and related variants, which is convenient for comparative algorithm experiments.

---

## License

This project is released under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).

The embedded `xtensor` and `xsimd` libraries retain their original BSD-3-Clause licenses. See the `LICENSE` files in the corresponding directories for details.
