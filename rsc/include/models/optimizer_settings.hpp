#ifndef MPPI_MODELS_OPTIMIZER_SETTINGS_HPP_
#define MPPI_MODELS_OPTIMIZER_SETTINGS_HPP_

#include "models/constraints.hpp"

namespace mppi
{

struct OptimizerSettings
{
    ControlConstraints base_constraints;
    ControlConstraints constraints;
    SamplingStd sampling_std;

    float model_dt = 0.05f;
    float temperature = 0.6f;
    float gamma = 0.015f;
    unsigned int batch_size = 400;
    unsigned int time_steps = 50;
    unsigned int iteration_count = 1;
    bool shift_control_sequence = true;
    unsigned int thread_count = 4;
    unsigned int random_seed = 42;

    bool adaptive_temperature = false;
    float adaptive_temperature_min = 0.2f;
    float adaptive_temperature_max = 2.0f;
    float target_ess_ratio = 0.15f;
    bool use_mean_normalization = false;  // 保留兼容；softmax 对常数平移不敏感
    unsigned int elite_sample_count = 0;  // 0=全部样本；>0=仅Top-K参与加权更新
    float elite_update_rate = 1.0f;       // CEM均值更新率，1=完全采用本轮精英均值

    bool use_sg_filter = false;

    float prune_distance = 3.5f;
    float furthest_reached_quantile = 0.85f;

    // 时间相关采样噪声。0=白噪声，接近1时更平滑。
    float noise_correlation = 0.85f;
    // Smooth-MPPI/input-lifting style sampling.  When enabled, Gaussian
    // perturbations act on control increments and are integrated over the
    // horizon instead of being added independently to absolute controls.
    bool sample_control_derivatives = false;
    float control_derivative_noise_scale = 0.10f;
    unsigned int sampling_support_points = 0;
    float sampling_kernel_sigma = 1.0f;
    bool reuse_noise_sequence = false;
    float noise_cycle_correlation = 0.98f;
    unsigned int nominal_sample_count = 1;
    bool antithetic_sampling = true;

    // Path-informed proposal distribution. A subset of rollouts is centered
    // around the curvature feed-forward control instead of the warm start.
    float guided_sampling_ratio = 0.25f;
    float guided_sampling_blend = 0.80f;
    float path_reference_speed = 0.60f;
    float path_max_lateral_acceleration = 0.60f;
    float path_corner_preview_distance = 1.20f;
    float path_corner_deceleration = 0.90f;
    float heading_alignment_start = 0.25f;
    float heading_alignment_stop = 1.20f;
    float disturbance_heading_gate_scale = 1.50f;
    float corner_alignment_distance = 0.05f;
    float corner_approach_distance = 0.60f;
    float corner_alignment_min_angle = 0.60f;
    float corner_alignment_yaw_tolerance = 0.08f;
    float corner_alignment_gain = 2.80f;
    float corner_alignment_max_yaw_rate = 1.60f;
    float goal_deceleration = 0.50f;
    float guide_lateral_gain = 1.8f;
    float guide_heading_gain = 2.2f;
    float curvature_consistent_speed = 0.30f;
    float ancillary_lateral_gain = 0.0f;
    float ancillary_heading_gain = 0.0f;
    float ancillary_max_yaw_rate = 0.40f;
    float ancillary_nominal_scale = 1.0f;
    float ancillary_disturbance_scale = 0.0f;
    float ancillary_curvature_activation = 0.0f;

    bool sample_disturbance_uncertainty = false;
    float disturbance_sigma_clip = 2.0f;
    float disturbance_decay_time_constant = 0.8f;
    float min_traction_scale = 0.35f;
    float traction_speed_exponent = 0.5f;
    float traction_acceleration_exponent = 1.0f;
};

}  // namespace mppi

#endif  // MPPI_MODELS_OPTIMIZER_SETTINGS_HPP_
