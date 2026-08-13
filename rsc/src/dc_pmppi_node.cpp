#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "controller.hpp"
#include "tools/disturbance_estimator.hpp"
#include "tools/goal_docking.hpp"
#include "tools/math_utils.hpp"
#include "tools/path_projection.hpp"

class MPPIRos1Node
{
public:
    MPPIRos1Node(ros::NodeHandle & nh, ros::NodeHandle & pnh)
        : nh_(nh), pnh_(pnh)
    {
        initController();

        path_sub_ = nh_.subscribe("/plan", 1, &MPPIRos1Node::pathCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 20, &MPPIRos1Node::odomCallback, this);
        if (use_executed_command_ && !executed_command_topic_.empty()) {
            executed_cmd_sub_ = nh_.subscribe(
                executed_command_topic_, 50,
                &MPPIRos1Node::executedCommandCallback, this);
        }
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        solve_time_pub_ = nh_.advertise<std_msgs::Float64>("/controller_solve_time_ms", 100);
        local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 10);

        control_timer_ = nh_.createTimer(
            ros::Duration(static_cast<double>(control_period_ms_) / 1000.0),
            &MPPIRos1Node::controlLoop, this);

#ifdef _OPENMP
        ROS_INFO(
            "DC-PMPPI ROS1 pure-tracking controller initialized; odom=%s, DC=%s, "
            "OpenMP=ON (threads=%d, runtime_max=%d).",
            odom_topic_.c_str(), dc_pmppi_enabled_ ? "ON" : "OFF",
            thread_count_, omp_get_max_threads());
#else
        ROS_WARN(
            "DC-PMPPI ROS1 pure-tracking controller initialized; odom=%s, DC=%s, "
            "OpenMP=OFF (serial fallback).",
            odom_topic_.c_str(), dc_pmppi_enabled_ ? "ON" : "OFF");
#endif
    }

private:
    struct TimedControl
    {
        ros::Time stamp;
        mppi::Twist2D command;
    };

    static float quaternionYaw(const geometry_msgs::Quaternion & q)
    {
        return static_cast<float>(std::atan2(
            2.0 * (q.z * q.w + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
    }

    static bool validQuaternion(const geometry_msgs::Quaternion & q)
    {
        const double norm_squared =
            q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        return std::isfinite(norm_squared) && norm_squared > 0.25;
    }

    std::vector<mppi::Pose2D> preprocessPath(const nav_msgs::Path & message) const
    {
        std::vector<mppi::Point2D> points;
        points.reserve(message.poses.size());
        for (const auto & pose_stamped : message.poses) {
            const float x = static_cast<float>(pose_stamped.pose.position.x);
            const float y = static_cast<float>(pose_stamped.pose.position.y);
            if (!std::isfinite(x) || !std::isfinite(y)) continue;
            if (!points.empty() &&
                std::hypot(x - points.back().x, y - points.back().y) <
                    path_duplicate_epsilon_) {
                continue;
            }
            points.emplace_back(x, y);
        }

        if (points.empty()) return {};
        if (points.size() == 1) {
            float yaw = 0.0f;
            if (!message.poses.empty() &&
                validQuaternion(message.poses.back().pose.orientation)) {
                yaw = quaternionYaw(message.poses.back().pose.orientation);
            }
            return {mppi::Pose2D(points.front().x, points.front().y, yaw)};
        }

        std::vector<float> cumulative(points.size(), 0.0f);
        for (size_t i = 1; i < points.size(); ++i) {
            cumulative[i] = cumulative[i - 1] +
                std::hypot(points[i].x - points[i - 1].x,
                           points[i].y - points[i - 1].y);
        }
        const float total_length = cumulative.back();
        if (total_length <= path_duplicate_epsilon_) return {};

        std::vector<mppi::Point2D> resampled_points;
        const float spacing = std::max(path_resample_spacing_, 0.01f);
        size_t segment = 0;
        for (float distance = 0.0f; distance < total_length; distance += spacing) {
            while (segment + 1 < cumulative.size() &&
                   cumulative[segment + 1] < distance) {
                ++segment;
            }
            const size_t next = std::min(segment + 1, points.size() - 1);
            const float segment_length = cumulative[next] - cumulative[segment];
            const float ratio = segment_length > 1e-6f ?
                (distance - cumulative[segment]) / segment_length : 0.0f;
            resampled_points.emplace_back(
                points[segment].x + ratio * (points[next].x - points[segment].x),
                points[segment].y + ratio * (points[next].y - points[segment].y));
        }
        if (resampled_points.empty() ||
            std::hypot(resampled_points.back().x - points.back().x,
                       resampled_points.back().y - points.back().y) >
                path_duplicate_epsilon_) {
            resampled_points.push_back(points.back());
        }

        std::vector<mppi::Pose2D> result;
        result.reserve(resampled_points.size());
        for (size_t i = 0; i < resampled_points.size(); ++i) {
            const size_t previous = i > 0 ? i - 1 : i;
            const size_t next = i + 1 < resampled_points.size() ? i + 1 : i;
            const float yaw = std::atan2(
                resampled_points[next].y - resampled_points[previous].y,
                resampled_points[next].x - resampled_points[previous].x);
            result.emplace_back(resampled_points[i].x, resampled_points[i].y, yaw);
        }

        if (use_path_goal_orientation_ && !message.poses.empty() &&
            validQuaternion(message.poses.back().pose.orientation)) {
            result.back().theta = quaternionYaw(message.poses.back().pose.orientation);
        }
        return result;
    }

    void initController()
    {
        mppi::OptimizerSettings settings;
        int batch_size = 400;
        int time_steps = 50;
        int iteration_count = 1;
        int thread_count = 4;
        int random_seed = 42;
        int nominal_sample_count = 1;
        int elite_sample_count = 0;
        pnh_.param("batch_size", batch_size, batch_size);
        pnh_.param("time_steps", time_steps, time_steps);
        pnh_.param("iteration_count", iteration_count, iteration_count);
        pnh_.param("thread_count", thread_count, thread_count);
        pnh_.param("random_seed", random_seed, random_seed);
        thread_count_ = std::max(1, thread_count);
        pnh_.param("nominal_sample_count", nominal_sample_count, nominal_sample_count);
        pnh_.param("elite_sample_count", elite_sample_count, elite_sample_count);
        settings.batch_size = static_cast<unsigned int>(std::max(1, batch_size));
        settings.time_steps = static_cast<unsigned int>(std::max(2, time_steps));
        settings.iteration_count = static_cast<unsigned int>(std::max(1, iteration_count));
        settings.thread_count = static_cast<unsigned int>(thread_count_);
        settings.random_seed = static_cast<unsigned int>(std::max(0, random_seed));
        settings.nominal_sample_count =
            static_cast<unsigned int>(std::max(0, nominal_sample_count));
        settings.elite_sample_count =
            static_cast<unsigned int>(std::max(0, elite_sample_count));

        pnh_.param("model_dt", settings.model_dt, 0.05f);
        pnh_.param("temperature", settings.temperature, 0.6f);
        pnh_.param("gamma", settings.gamma, 0.015f);
        pnh_.param("prune_distance", settings.prune_distance, 3.5f);
        pnh_.param("furthest_reached_quantile", settings.furthest_reached_quantile, 0.85f);
        pnh_.param("noise_correlation", settings.noise_correlation, 0.85f);
        pnh_.param("sample_control_derivatives",
                   settings.sample_control_derivatives, false);
        pnh_.param("control_derivative_noise_scale",
                   settings.control_derivative_noise_scale, 0.10f);
        int sampling_support_points = 0;
        pnh_.param("sampling_support_points", sampling_support_points, 0);
        settings.sampling_support_points = static_cast<unsigned int>(
            std::max(0, sampling_support_points));
        pnh_.param("sampling_kernel_sigma", settings.sampling_kernel_sigma, 1.0f);
        pnh_.param("reuse_noise_sequence", settings.reuse_noise_sequence, false);
        pnh_.param("noise_cycle_correlation",
                   settings.noise_cycle_correlation, 0.98f);

        pnh_.param("vx_max", settings.base_constraints.vx_max, 0.7f);
        pnh_.param("vx_min", settings.base_constraints.vx_min, 0.0f);
        pnh_.param("vy_max", settings.base_constraints.vy_max, 0.0f);
        pnh_.param("wz_max", settings.base_constraints.wz_max, 1.0f);
        pnh_.param("ax_max", settings.base_constraints.ax_max, 1.2f);
        pnh_.param("ay_max", settings.base_constraints.ay_max, 0.0f);
        pnh_.param("az_max", settings.base_constraints.az_max, 2.5f);
        settings.constraints = settings.base_constraints;

        pnh_.param("vx_std", settings.sampling_std.vx, 0.20f);
        pnh_.param("vy_std", settings.sampling_std.vy, 0.0f);
        pnh_.param("wz_std", settings.sampling_std.wz, 0.35f);

        pnh_.param("use_sg_filter", settings.use_sg_filter, false);
        pnh_.param("shift_control_sequence", settings.shift_control_sequence, true);
        pnh_.param("use_mean_normalization", settings.use_mean_normalization, false);
        pnh_.param("adaptive_temperature", settings.adaptive_temperature, false);
        pnh_.param("adaptive_temperature_min", settings.adaptive_temperature_min, 0.2f);
        pnh_.param("adaptive_temperature_max", settings.adaptive_temperature_max, 2.0f);
        pnh_.param("target_ess_ratio", settings.target_ess_ratio, 0.15f);
        pnh_.param("antithetic_sampling", settings.antithetic_sampling, true);
        pnh_.param("elite_update_rate", settings.elite_update_rate, 1.0f);
        pnh_.param("guided_sampling_ratio", settings.guided_sampling_ratio, 0.25f);
        pnh_.param("guided_sampling_blend", settings.guided_sampling_blend, 0.80f);
        pnh_.param("path_reference_speed", settings.path_reference_speed, 0.60f);
        pnh_.param("path_max_lateral_acceleration",
                   settings.path_max_lateral_acceleration, 0.60f);
        pnh_.param("path_corner_preview_distance",
                   settings.path_corner_preview_distance, 1.20f);
        pnh_.param("path_corner_deceleration",
                   settings.path_corner_deceleration, 0.90f);
        pnh_.param("heading_alignment_start",
                   settings.heading_alignment_start, 0.25f);
        pnh_.param("heading_alignment_stop",
                   settings.heading_alignment_stop, 1.20f);
        pnh_.param("disturbance_heading_gate_scale",
                   settings.disturbance_heading_gate_scale, 1.50f);
        pnh_.param("corner_alignment_distance",
                   settings.corner_alignment_distance, 0.05f);
        pnh_.param("corner_approach_distance",
                   settings.corner_approach_distance, 0.60f);
        pnh_.param("corner_alignment_min_angle",
                   settings.corner_alignment_min_angle, 0.60f);
        pnh_.param("corner_alignment_yaw_tolerance",
                   settings.corner_alignment_yaw_tolerance, 0.08f);
        pnh_.param("corner_alignment_gain",
                   settings.corner_alignment_gain, 2.80f);
        pnh_.param("corner_alignment_max_yaw_rate",
                   settings.corner_alignment_max_yaw_rate, 1.60f);
        pnh_.param("goal_deceleration", settings.goal_deceleration, 0.50f);
        pnh_.param("guide_lateral_gain", settings.guide_lateral_gain, 1.80f);
        pnh_.param("guide_heading_gain", settings.guide_heading_gain, 2.20f);
        pnh_.param("curvature_consistent_speed",
                   settings.curvature_consistent_speed, 0.30f);
        pnh_.param("ancillary_lateral_gain", settings.ancillary_lateral_gain, 0.0f);
        pnh_.param("ancillary_heading_gain", settings.ancillary_heading_gain, 0.0f);
        pnh_.param("ancillary_max_yaw_rate", settings.ancillary_max_yaw_rate, 0.40f);
        pnh_.param("ancillary_nominal_scale",
                   settings.ancillary_nominal_scale, 1.0f);
        pnh_.param("ancillary_disturbance_scale",
                   settings.ancillary_disturbance_scale, 0.0f);
        pnh_.param("ancillary_curvature_activation",
                   settings.ancillary_curvature_activation, 0.0f);
        pnh_.param("sample_disturbance_uncertainty",
                   settings.sample_disturbance_uncertainty, false);
        pnh_.param("disturbance_sigma_clip", settings.disturbance_sigma_clip, 2.0f);
        pnh_.param("disturbance_decay_time_constant",
                   settings.disturbance_decay_time_constant, 0.8f);
        pnh_.param("min_traction_scale", settings.min_traction_scale, 0.35f);
        pnh_.param("traction_speed_exponent", settings.traction_speed_exponent, 0.5f);
        pnh_.param("traction_acceleration_exponent",
                   settings.traction_acceleration_exponent, 1.0f);

        pnh_.param("goal_tolerance", goal_tolerance_, 0.10f);
        pnh_.param("goal_yaw_tolerance", goal_yaw_tolerance_, 0.15f);
        pnh_.param("goal_speed_tolerance", goal_speed_tolerance_, 0.08f);
        pnh_.param("goal_yaw_rate_tolerance", goal_yaw_rate_tolerance_, 0.08f);
        pnh_.param("goal_settle_cycles", goal_settle_cycles_, 4);
        pnh_.param("goal_reacquire_distance", goal_reacquire_distance_, 0.08f);
        pnh_.param("goal_reacquire_yaw", goal_reacquire_yaw_, 0.20f);
        pnh_.param("goal_docking_distance",
                   goal_docking_config_.activation_distance, 0.20f);
        pnh_.param("goal_docking_full_control_distance",
                   goal_docking_config_.full_control_distance, 0.12f);
        pnh_.param("goal_docking_max_speed",
                   goal_docking_config_.maximum_linear_speed, 0.18f);
        pnh_.param("goal_recovery_distance",
                   goal_docking_config_.recovery_distance, 1.20f);
        pnh_.param("goal_recovery_speed_threshold",
                   goal_docking_config_.recovery_speed_threshold, 0.08f);
        pnh_.param("goal_recovery_max_speed",
                   goal_docking_config_.recovery_maximum_linear_speed, 0.35f);
        pnh_.param("goal_docking_distance_gain",
                   goal_docking_config_.distance_gain, 1.20f);
        pnh_.param("goal_docking_bearing_gain",
                   goal_docking_config_.bearing_gain, 2.50f);
        pnh_.param("goal_docking_pose_alignment_gain",
                   goal_docking_config_.pose_alignment_gain, 0.80f);
        pnh_.param("goal_docking_final_yaw_gain",
                   goal_docking_config_.final_yaw_gain, 2.00f);
        goal_docking_config_.position_tolerance = goal_tolerance_;
        goal_docking_config_.goal_deceleration = settings.goal_deceleration;
        goal_docking_config_.linear_acceleration = settings.base_constraints.ax_max;
        goal_docking_config_.angular_acceleration = settings.base_constraints.az_max;
        goal_docking_config_.maximum_angular_speed = settings.base_constraints.wz_max;
        pnh_.param("path_resample_spacing", path_resample_spacing_, 0.10f);
        pnh_.param("path_duplicate_epsilon", path_duplicate_epsilon_, 0.005f);
        pnh_.param("use_path_goal_orientation", use_path_goal_orientation_, false);

        std::string motion_model;
        pnh_.param("motion_model", motion_model, std::string("DiffDrive"));
        double ackermann_radius = 0.45;
        pnh_.param("ackermann_min_turning_radius", ackermann_radius, ackermann_radius);

        controller_ = std::make_unique<mppi::MPPIController>();
        controller_->initialize(settings, motion_model,
                                static_cast<float>(ackermann_radius));

        if (auto * critic = controller_->getMPCCTrackingCritic()) {
            double weight, contour_weight, lag_weight, heading_weight;
            double velocity_weight, yaw_rate_weight, terminal_weight;
            double contour_scale, lag_scale, heading_scale;
            double velocity_scale, yaw_rate_scale;
            int step;
            pnh_.param("mpcc_weight", weight, 8.0);
            pnh_.param("mpcc_contour_weight", contour_weight, 1.0);
            pnh_.param("mpcc_lag_weight", lag_weight, 0.35);
            pnh_.param("mpcc_heading_weight", heading_weight, 0.70);
            pnh_.param("mpcc_velocity_weight", velocity_weight, 0.25);
            pnh_.param("mpcc_yaw_rate_weight", yaw_rate_weight, 0.20);
            pnh_.param("mpcc_terminal_weight", terminal_weight, 2.0);
            pnh_.param("mpcc_contour_scale", contour_scale, 0.20);
            pnh_.param("mpcc_lag_scale", lag_scale, 0.35);
            pnh_.param("mpcc_heading_scale", heading_scale, 0.40);
            pnh_.param("mpcc_velocity_scale", velocity_scale, 0.40);
            pnh_.param("mpcc_yaw_rate_scale", yaw_rate_scale, 0.80);
            pnh_.param("mpcc_traj_step", step, 2);
            critic->setParams(
                weight, contour_weight, lag_weight, heading_weight,
                velocity_weight, yaw_rate_weight, terminal_weight,
                contour_scale, lag_scale, heading_scale, velocity_scale,
                yaw_rate_scale, settings.path_reference_speed,
                settings.path_max_lateral_acceleration,
                settings.goal_deceleration,
                settings.path_corner_preview_distance,
                settings.path_corner_deceleration, step);
        }

        if (auto * critic = controller_->getControlRateCritic()) {
            double weight, vx_weight, wz_weight, vx_scale, wz_scale;
            int step;
            pnh_.param("control_rate_weight", weight, 2.0);
            pnh_.param("control_rate_vx_weight", vx_weight, 0.5);
            pnh_.param("control_rate_wz_weight", wz_weight, 1.0);
            pnh_.param("control_rate_vx_scale", vx_scale, 1.2);
            pnh_.param("control_rate_wz_scale", wz_scale, 2.5);
            pnh_.param("control_rate_traj_step", step, 1);
            critic->setParams(weight, vx_weight, wz_weight,
                              vx_scale, wz_scale, step);
        }

        if (auto * critic = controller_->getMPCCTrackingCritic()) {
            critic->setThreadCount(static_cast<unsigned int>(thread_count));
        }
        pnh_.param("collect_critic_statistics", collect_critic_statistics_, false);
        controller_->setCollectCriticStatistics(collect_critic_statistics_);

        pnh_.param("control_period_ms", control_period_ms_, 50);

        // 先读旧参数，再由新参数覆盖。
        pnh_.param("dc_mppi_enabled", dc_pmppi_enabled_, false);
        pnh_.param("dc_pmppi_enabled", dc_pmppi_enabled_, dc_pmppi_enabled_);
        pnh_.param("dc_corner_suppression_enabled",
                   dc_corner_suppression_enabled_, true);
        pnh_.param("dc_corner_suppression_distance",
                   dc_corner_suppression_distance_, 1.0f);
        pnh_.param("dc_corner_suppression_hold_time",
                   dc_corner_suppression_hold_time_, 3.0);
        pnh_.param("dc_corner_suppression_min_angle",
                   dc_corner_suppression_min_angle_, 0.60f);
        mppi::DisturbanceEstimatorConfig estimator_config;
        pnh_.param("dc_mppi_filter_time_constant",
                   estimator_config.filter_time_constant, 0.20f);
        pnh_.param("dc_pmppi_filter_time_constant",
                   estimator_config.filter_time_constant,
                   estimator_config.filter_time_constant);
        pnh_.param("dc_pmppi_effectiveness_time_constant",
                   estimator_config.effectiveness_time_constant, 1.50f);
        pnh_.param("dc_pmppi_use_centered_effectiveness_regression",
                   estimator_config.use_centered_effectiveness_regression,
                   false);
        pnh_.param("dc_pmppi_effectiveness_excitation_variance",
                   estimator_config.effectiveness_excitation_variance,
                   0.0025f);
        pnh_.param("dc_pmppi_longitudinal_actuator_time_constant",
                   estimator_config.longitudinal_actuator_time_constant, 0.10f);
        pnh_.param("dc_pmppi_yaw_actuator_time_constant",
                   estimator_config.yaw_actuator_time_constant, 0.0f);
        pnh_.param("dc_pmppi_confidence_time_constant",
                   estimator_config.confidence_time_constant, 0.50f);
        pnh_.param("dc_pmppi_minimum_confidence",
                   estimator_config.minimum_confidence, 0.25f);
        pnh_.param("dc_pmppi_pose_velocity_window",
                   estimator_config.pose_velocity_window, 0.10f);
        pnh_.param("dc_mppi_max_sample_interval",
                   estimator_config.max_sample_interval, 0.50f);
        pnh_.param("dc_pmppi_max_sample_interval",
                   estimator_config.max_sample_interval,
                   estimator_config.max_sample_interval);
        pnh_.param("dc_mppi_vy_deadzone",
                   estimator_config.lateral_deadzone, 0.02f);
        pnh_.param("dc_pmppi_vy_deadzone",
                   estimator_config.lateral_deadzone,
                   estimator_config.lateral_deadzone);
        pnh_.param("dc_mppi_wz_deadzone",
                   estimator_config.yaw_rate_deadzone, 0.02f);
        pnh_.param("dc_pmppi_wz_deadzone",
                   estimator_config.yaw_rate_deadzone,
                   estimator_config.yaw_rate_deadzone);
        pnh_.param("dc_mppi_max_vy",
                   estimator_config.max_lateral_velocity, 0.50f);
        pnh_.param("dc_pmppi_max_vy",
                   estimator_config.max_lateral_velocity,
                   estimator_config.max_lateral_velocity);
        pnh_.param("dc_mppi_max_wz_dist",
                   estimator_config.max_yaw_rate, 1.0f);
        pnh_.param("dc_pmppi_max_wz_dist",
                   estimator_config.max_yaw_rate,
                   estimator_config.max_yaw_rate);
        pnh_.param("dc_mppi_vy_std_floor",
                   estimator_config.lateral_std_floor, 0.01f);
        pnh_.param("dc_pmppi_vy_std_floor",
                   estimator_config.lateral_std_floor,
                   estimator_config.lateral_std_floor);
        pnh_.param("dc_mppi_wz_std_floor",
                   estimator_config.yaw_rate_std_floor, 0.01f);
        pnh_.param("dc_pmppi_wz_std_floor",
                   estimator_config.yaw_rate_std_floor,
                   estimator_config.yaw_rate_std_floor);
        pnh_.param("min_traction_scale", estimator_config.min_traction_scale, 0.35f);
        pnh_.param("traction_lateral_gain", estimator_config.traction_lateral_gain, 1.5f);
        pnh_.param("traction_yaw_gain", estimator_config.traction_yaw_gain, 0.8f);
        pnh_.param("longitudinal_effectiveness",
                   estimator_config.longitudinal_effectiveness, 1.0f);
        pnh_.param("yaw_effectiveness", estimator_config.yaw_effectiveness, 1.0f);
        pnh_.param("dc_pmppi_command_speed_deadzone",
                   estimator_config.command_speed_deadzone, 0.40f);
        pnh_.param("dc_pmppi_command_yaw_rate_deadzone",
                   estimator_config.command_yaw_rate_deadzone, 0.30f);
        pnh_.param("dc_pmppi_max_vy_innovation",
                   estimator_config.max_lateral_innovation, 0.40f);
        pnh_.param("dc_pmppi_max_wz_innovation",
                   estimator_config.max_yaw_rate_innovation, 0.80f);
        pnh_.param("dc_pmppi_use_odom_twist_lateral_velocity",
                   estimator_config.use_odom_twist_lateral_velocity, false);
        pnh_.param("dc_pmppi_use_odom_twist_yaw_rate",
                   estimator_config.use_odom_twist_yaw_rate, false);
        pnh_.param("dc_pmppi_use_odom_twist_longitudinal_velocity",
                   estimator_config.use_odom_twist_longitudinal_velocity, false);
        disturbance_estimator_.setConfig(estimator_config);

        pnh_.param("dc_mppi_actuator_delay", actuator_delay_, 0.05);
        pnh_.param("dc_pmppi_actuator_delay", actuator_delay_, actuator_delay_);
        pnh_.param("dc_pmppi_compensation_gain",
                   compensation_gain_, 0.85f);
        compensation_gain_ = mppi::clamp(compensation_gain_, 0.0f, 1.0f);
        pnh_.param("dc_pmppi_use_executed_command",
                   use_executed_command_, true);
        pnh_.param("dc_pmppi_executed_command_topic",
                   executed_command_topic_,
                   std::string("/constrained_cmd_vel_stamped"));
        pnh_.param("dc_pmppi_executed_command_timeout",
                   executed_command_timeout_, 0.20);
        executed_command_timeout_ =
            std::max(executed_command_timeout_, 0.02);
        pnh_.param("odom_timeout", odom_timeout_, 0.30);
        pnh_.param("planning_frame", planning_frame_, std::string("map"));
        pnh_.param("odom_topic", odom_topic_, std::string("/odom1"));
        pnh_.param("strict_frame_check", strict_frame_check_, true);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr & message)
    {
        if (message->poses.empty()) {
            {
                std::lock_guard<std::mutex> controller_lock(controller_mutex_);
                global_path_.clear();
                controller_->setPath(global_path_);
                controller_->reset();
            }
            path_received_ = false;
            goal_reached_ = false;
            goal_settle_count_ = 0;
            disturbance_estimator_.reset();
            publishStop();
            ROS_WARN_THROTTLE(1.0, "Received empty path; controller stopped.");
            return;
        }
        if (strict_frame_check_ && !message->header.frame_id.empty() &&
            message->header.frame_id != planning_frame_) {
            ROS_ERROR_THROTTLE(1.0,
                "Rejected path in frame '%s'; expected '%s'.",
                message->header.frame_id.c_str(), planning_frame_.c_str());
            return;
        }

        std::vector<mppi::Pose2D> processed = preprocessPath(*message);
        if (processed.size() < 2) {
            ROS_WARN_THROTTLE(1.0, "Rejected path after preprocessing: fewer than 2 points.");
            path_received_ = false;
            publishStop();
            return;
        }

        std::lock_guard<std::mutex> controller_lock(controller_mutex_);
        bool equivalent = global_path_.size() == processed.size();
        for (size_t i = 0; equivalent && i < processed.size(); ++i) {
            equivalent = std::hypot(
                global_path_[i].x - processed[i].x,
                global_path_[i].y - processed[i].y) <= 1e-4f &&
                std::abs(mppi::shortestAngularDistance(
                    global_path_[i].theta, processed[i].theta)) <= 1e-4f;
        }
        if (equivalent) {
            path_received_ = true;
            return;
        }
        global_path_ = processed;
        controller_->setPath(global_path_);
        // A geometrically different reference must not inherit the previous
        // path's future control sequence.  The external safety guard preserves
        // acceleration continuity, while a fresh MPPI warm start reacts to the
        // new detour on the very next control cycle.
        controller_->reset();
        path_received_ = true;
        goal_reached_ = false;
        goal_settle_count_ = 0;
        ROS_INFO("Path accepted: raw=%zu, resampled=%zu, spacing=%.3f m",
                 message->poses.size(), processed.size(), path_resample_spacing_);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr & message)
    {
        if (strict_frame_check_ && !message->header.frame_id.empty() &&
            message->header.frame_id != planning_frame_) {
            ROS_ERROR_THROTTLE(1.0,
                "Rejected odometry in frame '%s'; expected '%s'.",
                message->header.frame_id.c_str(), planning_frame_.c_str());
            return;
        }
        if (!std::isfinite(message->pose.pose.position.x) ||
            !std::isfinite(message->pose.pose.position.y) ||
            !validQuaternion(message->pose.pose.orientation) ||
            !std::isfinite(message->twist.twist.linear.x) ||
            !std::isfinite(message->twist.twist.linear.y) ||
            !std::isfinite(message->twist.twist.angular.z)) {
            ROS_ERROR_THROTTLE(1.0, "Rejected invalid odometry sample.");
            return;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        robot_pose_.x = static_cast<float>(message->pose.pose.position.x);
        robot_pose_.y = static_cast<float>(message->pose.pose.position.y);
        robot_pose_.theta = quaternionYaw(message->pose.pose.orientation);
        robot_speed_.vx = static_cast<float>(message->twist.twist.linear.x);
        robot_speed_.vy = static_cast<float>(message->twist.twist.linear.y);
        robot_speed_.wz = static_cast<float>(message->twist.twist.angular.z);

        const ros::Time now = ros::Time::now();
        const ros::Time stamp = message->header.stamp.isZero() ?
            now : message->header.stamp;
        if (dc_pmppi_enabled_) {
            const mppi::Twist2D matched_command = findMatchedCommand(stamp);
            disturbance_estimate_ = disturbance_estimator_.update(
                robot_pose_, robot_speed_, stamp.toSec(), matched_command);
        }
        last_odom_receive_time_ = now;
        pose_received_ = true;
    }

    void executedCommandCallback(
        const geometry_msgs::TwistStamped::ConstPtr & message)
    {
        if (!std::isfinite(message->twist.linear.x) ||
            !std::isfinite(message->twist.angular.z)) {
            ROS_WARN_THROTTLE(
                1.0, "Rejected invalid executed-command sample.");
            return;
        }
        const ros::Time stamp = message->header.stamp.isZero() ?
            ros::Time::now() : message->header.stamp;
        std::lock_guard<std::mutex> lock(data_mutex_);
        mppi::Twist2D command;
        command.vx = static_cast<float>(message->twist.linear.x);
        command.vy = 0.0f;
        command.wz = static_cast<float>(message->twist.angular.z);
        executed_command_history_.push_back({stamp, command});
        while (executed_command_history_.size() > 200) {
            executed_command_history_.pop_front();
        }
    }

    static mppi::Twist2D findHistoryCommand(
        const std::deque<TimedControl> & history,
        const ros::Time & target_time,
        const mppi::Twist2D & fallback)
    {
        mppi::Twist2D matched = fallback;
        for (auto it = history.rbegin(); it != history.rend(); ++it) {
            if (it->stamp <= target_time) {
                matched = it->command;
                break;
            }
        }
        return matched;
    }

    mppi::Twist2D findMatchedCommand(const ros::Time & odom_stamp)
    {
        last_match_used_executed_command_ = false;
        if (use_executed_command_ && !executed_command_history_.empty()) {
            const double executed_age =
                (odom_stamp - executed_command_history_.back().stamp).toSec();
            if (executed_command_history_.front().stamp <= odom_stamp &&
                executed_age >= -0.02 &&
                executed_age <= executed_command_timeout_) {
                last_match_used_executed_command_ = true;
                return findHistoryCommand(
                    executed_command_history_, odom_stamp,
                    executed_command_history_.front().command);
            }
        }
        const ros::Time target_time = odom_stamp - ros::Duration(actuator_delay_);
        return findHistoryCommand(
            command_history_, target_time, last_cmd_sent_);
    }

    void recordCommand(const mppi::Twist2D & command)
    {
        last_cmd_sent_ = command;
        command_history_.push_back({ros::Time::now(), command});
        while (command_history_.size() > 100) command_history_.pop_front();
    }

    void publishStop()
    {
        geometry_msgs::Twist stop_message;
        cmd_vel_pub_.publish(stop_message);
        std::lock_guard<std::mutex> lock(data_mutex_);
        recordCommand(mppi::Twist2D{});
    }

    mppi::DisturbanceEstimate confidenceWeightedDisturbance(
        const mppi::DisturbanceEstimate & estimate) const
    {
        if (!estimate.valid) return mppi::DisturbanceEstimate{};
        mppi::DisturbanceEstimate weighted = estimate;
        const float weight = mppi::clamp(
            estimate.confidence * compensation_gain_, 0.0f, 1.0f);
        weighted.lateral_velocity *= weight;
        weighted.yaw_rate *= weight;
        weighted.lateral_velocity_std *= weight;
        weighted.yaw_rate_std *= weight;
        weighted.longitudinal_eta =
            1.0f - weight * (1.0f - estimate.longitudinal_eta);
        weighted.yaw_eta =
            1.0f - weight * (1.0f - estimate.yaw_eta);
        weighted.traction_scale =
            1.0f - weight * (1.0f - estimate.traction_scale);
        weighted.confidence = weight;
        return weighted;
    }

    void controlLoop(const ros::TimerEvent &)
    {
        if (!pose_received_ || !path_received_) return;

        mppi::Pose2D pose;
        mppi::Twist2D speed;
        mppi::DisturbanceEstimate disturbance;
        ros::Time odom_receive_time;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            pose = robot_pose_;
            speed = robot_speed_;
            disturbance = disturbance_estimate_;
            odom_receive_time = last_odom_receive_time_;
        }

        const double odom_age = odom_receive_time.isZero() ?
            std::numeric_limits<double>::infinity() :
            (ros::Time::now() - odom_receive_time).toSec();
        if (odom_age > odom_timeout_) {
            ROS_ERROR_THROTTLE(1.0,
                "Odometry timeout (age=%.3f s); publishing zero command.",
                odom_age);
            {
                std::lock_guard<std::mutex> controller_lock(controller_mutex_);
                controller_->reset();
            }
            disturbance_estimator_.reset();
            publishStop();
            return;
        }

        std::lock_guard<std::mutex> controller_lock(controller_mutex_);
        const mppi::Pose2D goal = controller_->getPath().getGoal();
        const float distance_to_goal = std::hypot(pose.x - goal.x, pose.y - goal.y);
        const float yaw_error = std::abs(mppi::shortestAngularDistance(
            goal.theta, pose.theta));
        if (goal_reached_) {
            if (distance_to_goal <= goal_reacquire_distance_ &&
                yaw_error <= goal_reacquire_yaw_) {
                publishStop();
                return;
            }
            goal_reached_ = false;
            goal_settle_count_ = 0;
            controller_->reset();
            ROS_WARN("Goal hold displaced: position_error=%.3fm, yaw_error=%.3frad; reacquiring.",
                     distance_to_goal, yaw_error);
        }
        mppi::DisturbanceEstimate applied_disturbance;
        if (dc_pmppi_enabled_) {
            applied_disturbance =
                confidenceWeightedDisturbance(disturbance);
            if (shouldSuppressCornerCompensation(pose, ros::Time::now())) {
                applied_disturbance = mppi::DisturbanceEstimate{};
                ROS_INFO_THROTTLE(1.0,
                    "DC rollout compensation suspended near discontinuous corner.");
            }
            controller_->setDisturbanceEstimate(applied_disturbance);
        } else {
            controller_->setDisturbanceEstimate(mppi::DisturbanceEstimate{});
        }

        const float speed_norm = std::hypot(speed.vx, speed.vy);
        const bool pose_at_goal = distance_to_goal <= goal_tolerance_ &&
            yaw_error <= goal_yaw_tolerance_;
        if (pose_at_goal) {
            if (speed_norm <= goal_speed_tolerance_ &&
                std::abs(speed.wz) <= goal_yaw_rate_tolerance_) {
                ++goal_settle_count_;
            } else {
                goal_settle_count_ = 0;
            }
            if (goal_settle_count_ >= std::max(1, goal_settle_cycles_)) {
                goal_reached_ = true;
                controller_->reset();
                publishStop();
                ROS_INFO("Goal reached and settled: position_error=%.3fm, yaw_error=%.3frad.",
                         distance_to_goal, yaw_error);
                return;
            }
        } else {
            goal_settle_count_ = 0;
        }

        const auto start = std::chrono::steady_clock::now();
        mppi::Twist2D command;
        try {
            command = controller_->computeVelocityCommands(pose, speed);
        } catch (const std::exception & exception) {
            ROS_WARN_THROTTLE(1.0, "MPPI compute failed: %s", exception.what());
            publishStop();
            controller_->reset();
            return;
        }
        const auto finish = std::chrono::steady_clock::now();
        const double solve_ms =
            std::chrono::duration<double, std::milli>(finish - start).count();
        std_msgs::Float64 solve_message;
        solve_message.data = solve_ms;
        solve_time_pub_.publish(solve_message);

        command = mppi::applyGoalDocking(
            command, pose, speed, goal, goal_docking_config_,
            static_cast<float>(control_period_ms_) / 1000.0f,
            dc_pmppi_enabled_ ?
                applied_disturbance : mppi::DisturbanceEstimate{});
        // 终端稳定器位于优化器之后，仍必须遵守同一Ackermann可行域，禁止
        // 在目标附近借助差速接口原地旋转或产生过大横向加速度。
        command = controller_->enforceMotionConstraints(command);

        controller_->setAppliedControl(command);

        geometry_msgs::Twist twist_message;
        twist_message.linear.x = command.vx;
        twist_message.linear.y = command.vy;
        twist_message.angular.z = command.wz;
        cmd_vel_pub_.publish(twist_message);
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            recordCommand(command);
        }

        ROS_INFO_THROTTLE(1.0,
            "MPPI solve=%.2fms, ESS=%.1f, cost=[%.3f, %.3f], odom_age=%.3fs, cmd=(%.3f, %.3f)",
            solve_ms,
            controller_->getEffectiveSampleSize(),
            controller_->getLastMinCost(),
            controller_->getLastMaxCost(),
            odom_age, command.vx, command.wz);
        if (dc_pmppi_enabled_) {
            ROS_INFO_THROTTLE(1.0,
                "DC estimate: valid=%s, vy=%+.3fm/s, wz=%+.3frad/s, "
                "eta_v=%.3f, eta_w=%.3f, traction=%.3f, confidence=%.2f, "
                "sigma=(%.3f, %.3f), input=%s",
                disturbance.valid ? "yes" : "no",
                disturbance.lateral_velocity, disturbance.yaw_rate,
                disturbance.longitudinal_eta, disturbance.yaw_eta,
                disturbance.traction_scale,
                disturbance.confidence,
                disturbance.lateral_velocity_std, disturbance.yaw_rate_std,
                last_match_used_executed_command_ ? "executed" : "raw-fallback");
        }
        if (solve_ms > static_cast<double>(control_period_ms_)) {
            ROS_WARN_THROTTLE(1.0,
                "MPPI solve time %.2f ms exceeds control period %d ms.",
                solve_ms, control_period_ms_);
        }

        const auto & statistics = controller_->getCriticStatistics();
        if (!statistics.empty()) {
            std::string text;
            for (const auto & statistic : statistics) {
                text += statistic.name + "=" +
                    std::to_string(statistic.mean_cost).substr(0, 6) +
                    "(" + std::to_string(statistic.elapsed_ms).substr(0, 5) +
                    "ms) ";
            }
            ROS_INFO_THROTTLE(2.0, "CriticStats: %s", text.c_str());
        }
        publishLocalPath();
    }

    void publishLocalPath()
    {
        const auto trajectory = controller_->getOptimizedTrajectory();
        const size_t trajectory_length = trajectory.shape(0);
        nav_msgs::Path message;
        message.header.stamp = ros::Time::now();
        message.header.frame_id = planning_frame_;
        message.poses.resize(trajectory_length);
        for (size_t i = 0; i < trajectory_length; ++i) {
            message.poses[i].header = message.header;
            message.poses[i].pose.position.x = trajectory(i, 0);
            message.poses[i].pose.position.y = trajectory(i, 1);
            const float yaw = trajectory(i, 2);
            message.poses[i].pose.orientation.z = std::sin(yaw * 0.5f);
            message.poses[i].pose.orientation.w = std::cos(yaw * 0.5f);
        }
        local_path_pub_.publish(message);
    }

    bool shouldSuppressCornerCompensation(
        const mppi::Pose2D & pose, const ros::Time & now)
    {
        if (!dc_corner_suppression_enabled_) return false;
        const mppi::Path & path = controller_->getPath();
        if (path.size() < 3) return false;
        const auto arc_lengths = mppi::computePathArcLengths(path);
        const mppi::PathProjection projection = mppi::projectPointToPath(
            path, arc_lengths, pose.x, pose.y);
        if (!projection.valid) return false;

        for (size_t segment = projection.segment_index + 1;
             segment + 1 < path.size(); ++segment) {
            const float next_yaw = std::atan2(
                path.y(segment + 1) - path.y(segment),
                path.x(segment + 1) - path.x(segment));
            if (std::abs(mppi::shortestAngularDistance(
                    projection.yaw, next_yaw)) <
                dc_corner_suppression_min_angle_) {
                continue;
            }
            const float distance = arc_lengths[segment] - projection.arc_length;
            if (distance >= 0.0f &&
                distance <= dc_corner_suppression_distance_) {
                dc_corner_suppression_until_ = now + ros::Duration(
                    std::max(0.0, dc_corner_suppression_hold_time_));
            }
            break;
        }
        return !dc_corner_suppression_until_.isZero() &&
            now <= dc_corner_suppression_until_;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::unique_ptr<mppi::MPPIController> controller_;

    mppi::Pose2D robot_pose_;
    mppi::Twist2D robot_speed_;
    std::vector<mppi::Pose2D> global_path_;
    std::atomic<bool> pose_received_{false};
    std::atomic<bool> path_received_{false};

    int control_period_ms_ = 50;
    int thread_count_ = 1;
    float goal_tolerance_ = 0.10f;
    float goal_yaw_tolerance_ = 0.15f;
    float goal_speed_tolerance_ = 0.08f;
    float goal_yaw_rate_tolerance_ = 0.08f;
    float goal_reacquire_distance_ = 0.08f;
    float goal_reacquire_yaw_ = 0.20f;
    int goal_settle_cycles_ = 4;
    int goal_settle_count_ = 0;
    bool goal_reached_ = false;
    mppi::GoalDockingConfig goal_docking_config_;
    float path_resample_spacing_ = 0.10f;
    float path_duplicate_epsilon_ = 0.005f;
    bool use_path_goal_orientation_ = false;
    bool collect_critic_statistics_ = false;

    bool dc_pmppi_enabled_ = false;
    bool dc_corner_suppression_enabled_ = true;
    float dc_corner_suppression_distance_ = 1.0f;
    float dc_corner_suppression_min_angle_ = 0.60f;
    double dc_corner_suppression_hold_time_ = 3.0;
    ros::Time dc_corner_suppression_until_;
    mppi::DisturbanceEstimator disturbance_estimator_;
    mppi::DisturbanceEstimate disturbance_estimate_;
    mppi::Twist2D last_cmd_sent_;
    ros::Time last_odom_receive_time_;
    double odom_timeout_ = 0.30;
    double actuator_delay_ = 0.05;
    float compensation_gain_ = 0.85f;
    bool use_executed_command_ = true;
    bool last_match_used_executed_command_ = false;
    double executed_command_timeout_ = 0.20;

    std::deque<TimedControl> command_history_;
    std::deque<TimedControl> executed_command_history_;

    std::string planning_frame_ = "map";
    std::string odom_topic_ = "/odom1";
    std::string executed_command_topic_ = "/constrained_cmd_vel_stamped";
    bool strict_frame_check_ = true;

    std::mutex data_mutex_;
    std::mutex controller_mutex_;

    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber executed_cmd_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher solve_time_pub_;
    ros::Publisher local_path_pub_;
    ros::Timer control_timer_;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "dc_pmppi_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    MPPIRos1Node node(nh, pnh);
    ros::spin();
    return 0;
}
