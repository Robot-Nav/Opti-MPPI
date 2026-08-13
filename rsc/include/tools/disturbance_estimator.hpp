// ============================================================================
// File: disturbance_estimator.hpp
// Robust odometry-based equivalent disturbance and control-effectiveness estimator.
// ============================================================================
#ifndef MPPI_TOOLS_DISTURBANCE_ESTIMATOR_HPP_
#define MPPI_TOOLS_DISTURBANCE_ESTIMATOR_HPP_

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

#include "models/types.hpp"
#include "tools/math_utils.hpp"

namespace mppi
{

struct DisturbanceEstimatorConfig
{
    float filter_time_constant = 0.20f;
    float effectiveness_time_constant = 1.50f;
    bool use_centered_effectiveness_regression = false;
    float effectiveness_excitation_variance = 0.0025f;
    float longitudinal_actuator_time_constant = 0.10f;
    float yaw_actuator_time_constant = 0.0f;
    float confidence_time_constant = 0.50f;
    float minimum_confidence = 0.25f;
    float pose_velocity_window = 0.10f;
    float max_sample_interval = 0.50f;
    float lateral_deadzone = 0.02f;
    float yaw_rate_deadzone = 0.02f;
    float command_speed_deadzone = 0.40f;
    float command_yaw_rate_deadzone = 0.30f;
    float max_lateral_velocity = 1.00f;
    float max_yaw_rate = 2.00f;
    float max_lateral_innovation = 0.40f;
    float max_yaw_rate_innovation = 0.80f;
    float lateral_std_floor = 0.01f;
    float yaw_rate_std_floor = 0.01f;
    float min_traction_scale = 0.35f;
    float traction_lateral_gain = 1.5f;
    float traction_yaw_gain = 0.8f;
    bool use_odom_twist_lateral_velocity = false;
    bool use_odom_twist_yaw_rate = false;
    bool use_odom_twist_longitudinal_velocity = false;
    float longitudinal_effectiveness = 1.0f;
    float yaw_effectiveness = 1.0f;
};

class DisturbanceEstimator
{
public:
    void setConfig(const DisturbanceEstimatorConfig & config)
    {
        config_ = config;
        config_.filter_time_constant = std::max(config_.filter_time_constant, 0.01f);
        config_.effectiveness_time_constant =
            std::max(config_.effectiveness_time_constant, 0.10f);
        config_.effectiveness_excitation_variance =
            std::max(config_.effectiveness_excitation_variance, 1e-5f);
        config_.longitudinal_actuator_time_constant =
            std::max(config_.longitudinal_actuator_time_constant, 0.0f);
        config_.yaw_actuator_time_constant =
            std::max(config_.yaw_actuator_time_constant, 0.0f);
        config_.confidence_time_constant =
            std::max(config_.confidence_time_constant, 0.05f);
        config_.minimum_confidence =
            clamp(config_.minimum_confidence, 0.0f, 0.95f);
        config_.pose_velocity_window = clamp(config_.pose_velocity_window, 0.02f, 0.50f);
        config_.max_sample_interval = std::max(
            config_.max_sample_interval, config_.pose_velocity_window + 0.01f);
        config_.command_speed_deadzone = std::max(config_.command_speed_deadzone, 0.02f);
        config_.command_yaw_rate_deadzone =
            std::max(config_.command_yaw_rate_deadzone, 0.02f);
        config_.max_lateral_innovation =
            std::max(config_.max_lateral_innovation, config_.lateral_deadzone);
        config_.max_yaw_rate_innovation =
            std::max(config_.max_yaw_rate_innovation, config_.yaw_rate_deadzone);
        config_.min_traction_scale = clamp(config_.min_traction_scale, 0.1f, 1.0f);
        config_.longitudinal_effectiveness =
            clamp(config_.longitudinal_effectiveness, 0.3f, 1.0f);
        config_.yaw_effectiveness = clamp(config_.yaw_effectiveness, 0.3f, 1.0f);
        reset();
    }

    void reset()
    {
        samples_.clear();
        estimate_ = DisturbanceEstimate{};
        lateral_variance_ = 0.0f;
        yaw_variance_ = 0.0f;
        estimated_longitudinal_eta_ = 1.0f;
        estimated_yaw_eta_ = 1.0f;
        modeled_longitudinal_command_ = 0.0f;
        modeled_yaw_command_ = 0.0f;
        longitudinal_command_mean_ = 0.0f;
        longitudinal_measurement_mean_ = 0.0f;
        longitudinal_command_variance_ = 0.0f;
        longitudinal_command_measurement_covariance_ = 0.0f;
        yaw_command_mean_ = 0.0f;
        yaw_measurement_mean_ = 0.0f;
        yaw_command_variance_ = 0.0f;
        yaw_command_measurement_covariance_ = 0.0f;
        confidence_ = 0.0f;
    }

    DisturbanceEstimate update(
        const Pose2D & pose, const Twist2D & measured_speed,
        double stamp_seconds, const Twist2D & matched_command)
    {
        if (!validInput(pose, measured_speed, stamp_seconds)) {
            estimate_.valid = false;
            return estimate_;
        }

        float filter_dt = 0.0f;
        if (!samples_.empty()) {
            const double sample_interval = stamp_seconds - samples_.back().stamp;
            if (sample_interval <= 1e-4 || sample_interval > config_.max_sample_interval) {
                samples_.clear();
                estimate_.valid = false;
            } else {
                filter_dt = static_cast<float>(sample_interval);
            }
        }
        samples_.push_back({pose, stamp_seconds});

        // Keep the oldest sample that still spans approximately the requested window.
        while (samples_.size() > 2 &&
               stamp_seconds - samples_[1].stamp >= config_.pose_velocity_window) {
            samples_.pop_front();
        }
        const double window_dt_double = stamp_seconds - samples_.front().stamp;
        if (filter_dt <= 0.0f ||
            window_dt_double < config_.pose_velocity_window * 0.8) {
            estimate_.valid = false;
            return estimate_;
        }

        const float window_dt = static_cast<float>(window_dt_double);
        const Pose2D & previous_pose = samples_.front().pose;
        const float dx = pose.x - previous_pose.x;
        const float dy = pose.y - previous_pose.y;
        const float dtheta = shortestAngularDistance(previous_pose.theta, pose.theta);
        const float midpoint_yaw = normalizeAngle(previous_pose.theta + 0.5f * dtheta);
        const float vx_world = dx / window_dt;
        const float vy_world = dy / window_dt;
        const float forward_from_pose =
            std::cos(midpoint_yaw) * vx_world + std::sin(midpoint_yaw) * vy_world;
        const float lateral_measurement =
            -std::sin(midpoint_yaw) * vx_world + std::cos(midpoint_yaw) * vy_world;
        const float measured_lateral_velocity =
            config_.use_odom_twist_lateral_velocity &&
            std::isfinite(measured_speed.vy) ? measured_speed.vy : lateral_measurement;
        const float yaw_rate_from_pose = dtheta / window_dt;
        const float measured_yaw_rate =
            config_.use_odom_twist_yaw_rate && std::isfinite(measured_speed.wz) ?
            measured_speed.wz : yaw_rate_from_pose;
        const float measured_forward_velocity =
            config_.use_odom_twist_longitudinal_velocity &&
            std::isfinite(measured_speed.vx) ? measured_speed.vx : forward_from_pose;

        // The command observed at the actuator boundary is still followed by
        // the plant's first-order actuator dynamics.  Model that lag explicitly
        // so it is not misidentified as low traction during acceleration.
        const auto updateActuatorState = [filter_dt](
            float command, float time_constant, float & modeled_command)
        {
            if (time_constant <= 1e-4f) {
                modeled_command = command;
                return;
            }
            const float alpha = 1.0f - std::exp(-filter_dt / time_constant);
            modeled_command += alpha * (command - modeled_command);
        };
        updateActuatorState(
            matched_command.vx, config_.longitudinal_actuator_time_constant,
            modeled_longitudinal_command_);
        updateActuatorState(
            matched_command.wz, config_.yaw_actuator_time_constant,
            modeled_yaw_command_);

        const float effectiveness_beta =
            std::exp(-filter_dt / config_.effectiveness_time_constant);
        const auto updateCenteredRegression = [effectiveness_beta](
            float command, float measurement,
            float & command_mean, float & measurement_mean,
            float & command_variance, float & covariance)
        {
            const float command_delta = command - command_mean;
            const float measurement_delta = measurement - measurement_mean;
            const float alpha = 1.0f - effectiveness_beta;
            command_mean += alpha * command_delta;
            measurement_mean += alpha * measurement_delta;
            // Exponentially weighted Welford update. Centering rejects a
            // slowly varying additive disturbance while retaining the
            // command-to-response slope.
            command_variance = effectiveness_beta * command_variance +
                alpha * command_delta * (command - command_mean);
            covariance = effectiveness_beta * covariance +
                alpha * command_delta * (measurement - measurement_mean);
        };
        updateCenteredRegression(
            modeled_longitudinal_command_, measured_forward_velocity,
            longitudinal_command_mean_, longitudinal_measurement_mean_,
            longitudinal_command_variance_,
            longitudinal_command_measurement_covariance_);
        updateCenteredRegression(
            modeled_yaw_command_, measured_yaw_rate,
            yaw_command_mean_, yaw_measurement_mean_,
            yaw_command_variance_, yaw_command_measurement_covariance_);

        const auto regressionEstimate = [this](
            float variance, float covariance) -> float
        {
            if (!config_.use_centered_effectiveness_regression ||
                variance < config_.effectiveness_excitation_variance) {
                return std::numeric_limits<float>::quiet_NaN();
            }
            const float estimate = covariance / variance;
            return estimate >= 0.25f && estimate <= 1.20f ?
                clamp(estimate, 0.3f, 1.0f) :
                std::numeric_limits<float>::quiet_NaN();
        };

        float observed_longitudinal_eta = regressionEstimate(
            longitudinal_command_variance_,
            longitudinal_command_measurement_covariance_);
        if (!std::isfinite(observed_longitudinal_eta) &&
            std::abs(modeled_longitudinal_command_) >= config_.command_speed_deadzone &&
            measured_forward_velocity * modeled_longitudinal_command_ > 0.0f) {
            observed_longitudinal_eta = clamp(
                std::abs(measured_forward_velocity / modeled_longitudinal_command_),
                0.3f, 1.0f);
        }
        if (std::isfinite(observed_longitudinal_eta)) {
            estimated_longitudinal_eta_ = effectiveness_beta * estimated_longitudinal_eta_ +
                (1.0f - effectiveness_beta) * observed_longitudinal_eta;
        }

        float observed_yaw_eta = regressionEstimate(
            yaw_command_variance_, yaw_command_measurement_covariance_);
        if (!std::isfinite(observed_yaw_eta) &&
            std::abs(modeled_yaw_command_) >= config_.command_yaw_rate_deadzone &&
            measured_yaw_rate * modeled_yaw_command_ > 0.0f) {
            observed_yaw_eta = clamp(
                std::abs(measured_yaw_rate / modeled_yaw_command_),
                0.3f, 1.0f);
        }
        if (std::isfinite(observed_yaw_eta)) {
            estimated_yaw_eta_ = effectiveness_beta * estimated_yaw_eta_ +
                (1.0f - effectiveness_beta) * observed_yaw_eta;
        }

        const float expected_yaw_rate =
            estimated_yaw_eta_ * modeled_yaw_command_;
        const float yaw_residual_measurement = measured_yaw_rate - expected_yaw_rate;
        const float lateral_target =
            std::abs(measured_lateral_velocity) <= config_.lateral_deadzone ?
            0.0f : measured_lateral_velocity;
        const float yaw_target =
            std::abs(yaw_residual_measurement) <= config_.yaw_rate_deadzone ?
            0.0f : yaw_residual_measurement;
        const float beta = std::exp(-filter_dt / config_.filter_time_constant);
        const float lateral_innovation = clamp(
            lateral_target - estimate_.lateral_velocity,
            -config_.max_lateral_innovation, config_.max_lateral_innovation);
        const float yaw_innovation = clamp(
            yaw_target - estimate_.yaw_rate,
            -config_.max_yaw_rate_innovation, config_.max_yaw_rate_innovation);
        estimate_.lateral_velocity += (1.0f - beta) * lateral_innovation;
        estimate_.yaw_rate += (1.0f - beta) * yaw_innovation;

        lateral_variance_ = beta * lateral_variance_ +
            (1.0f - beta) * lateral_innovation * lateral_innovation;
        yaw_variance_ = beta * yaw_variance_ +
            (1.0f - beta) * yaw_innovation * yaw_innovation;

        estimate_.lateral_velocity = clamp(
            estimate_.lateral_velocity,
            -config_.max_lateral_velocity, config_.max_lateral_velocity);
        estimate_.yaw_rate = clamp(
            estimate_.yaw_rate, -config_.max_yaw_rate, config_.max_yaw_rate);
        estimate_.lateral_velocity_std = std::max(
            config_.lateral_std_floor, std::sqrt(lateral_variance_));
        estimate_.yaw_rate_std = std::max(
            config_.yaw_rate_std_floor, std::sqrt(yaw_variance_));
        const float severity =
            config_.traction_lateral_gain * std::abs(estimate_.lateral_velocity) +
            config_.traction_yaw_gain * std::abs(estimate_.yaw_rate) +
            0.20f * (estimate_.lateral_velocity_std + estimate_.yaw_rate_std);
        estimate_.traction_scale = clamp(
            1.0f - severity, config_.min_traction_scale, 1.0f);
        estimate_.longitudinal_eta = clamp(
            config_.longitudinal_effectiveness * estimated_longitudinal_eta_, 0.3f, 1.0f);
        estimate_.yaw_eta = clamp(
            config_.yaw_effectiveness * estimated_yaw_eta_, 0.3f, 1.0f);

        // Confidence rises only after several consistent samples and is
        // reduced when innovations repeatedly approach their robust limits.
        // The controller blends compensation with the nominal model using this
        // value instead of switching the full estimate on abruptly.
        const float lateral_innovation_ratio =
            std::abs(lateral_innovation) /
            std::max(config_.max_lateral_innovation, 1e-3f);
        const float yaw_innovation_ratio =
            std::abs(yaw_innovation) /
            std::max(config_.max_yaw_rate_innovation, 1e-3f);
        const float innovation_reliability = clamp(
            1.0f - 0.5f * std::max(
                lateral_innovation_ratio, yaw_innovation_ratio),
            0.0f, 1.0f);
        const float confidence_alpha =
            1.0f - std::exp(-filter_dt / config_.confidence_time_constant);
        confidence_ += confidence_alpha *
            (innovation_reliability - confidence_);
        estimate_.confidence = clamp(confidence_, 0.0f, 1.0f);
        estimate_.valid =
            estimate_.confidence >= config_.minimum_confidence;
        return estimate_;
    }

    const DisturbanceEstimate & getEstimate() const { return estimate_; }

private:
    struct TimedPose
    {
        Pose2D pose;
        double stamp{0.0};
    };

    static bool validInput(
        const Pose2D & pose, const Twist2D & speed, double stamp)
    {
        return std::isfinite(pose.x) && std::isfinite(pose.y) &&
               std::isfinite(pose.theta) && std::isfinite(speed.vx) &&
               std::isfinite(speed.vy) && std::isfinite(speed.wz) &&
               std::isfinite(stamp);
    }

    DisturbanceEstimatorConfig config_;
    std::deque<TimedPose> samples_;
    DisturbanceEstimate estimate_;
    float lateral_variance_ = 0.0f;
    float yaw_variance_ = 0.0f;
    float estimated_longitudinal_eta_ = 1.0f;
    float estimated_yaw_eta_ = 1.0f;
    float modeled_longitudinal_command_ = 0.0f;
    float modeled_yaw_command_ = 0.0f;
    float longitudinal_command_mean_ = 0.0f;
    float longitudinal_measurement_mean_ = 0.0f;
    float longitudinal_command_variance_ = 0.0f;
    float longitudinal_command_measurement_covariance_ = 0.0f;
    float yaw_command_mean_ = 0.0f;
    float yaw_measurement_mean_ = 0.0f;
    float yaw_command_variance_ = 0.0f;
    float yaw_command_measurement_covariance_ = 0.0f;
    float confidence_ = 0.0f;
};

}  // namespace mppi

#endif  // MPPI_TOOLS_DISTURBANCE_ESTIMATOR_HPP_
