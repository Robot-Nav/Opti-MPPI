#ifndef MPPI_TOOLS_GOAL_DOCKING_HPP_
#define MPPI_TOOLS_GOAL_DOCKING_HPP_

#include <algorithm>
#include <cmath>

#include "models/types.hpp"
#include "tools/math_utils.hpp"

namespace mppi
{

struct GoalDockingConfig
{
    float activation_distance = 0.20f;
    float full_control_distance = 0.12f;
    float position_tolerance = 0.03f;
    float distance_gain = 1.2f;
    float bearing_gain = 2.5f;
    float pose_alignment_gain = 0.8f;
    float final_yaw_gain = 2.0f;
    float maximum_linear_speed = 0.18f;
    float recovery_distance = 1.20f;
    float recovery_speed_threshold = 0.08f;
    float recovery_maximum_linear_speed = 0.35f;
    float maximum_angular_speed = 1.0f;
    float linear_acceleration = 1.2f;
    float angular_acceleration = 2.5f;
    float goal_deceleration = 0.50f;
};

inline float smoothStep(float value)
{
    const float x = clamp(value, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

inline Twist2D applyGoalDocking(
    const Twist2D & nominal,
    const Pose2D & pose,
    const Twist2D & speed,
    const Pose2D & goal,
    const GoalDockingConfig & config,
    float dt,
    const DisturbanceEstimate & disturbance = DisturbanceEstimate{})
{
    const float dx = goal.x - pose.x;
    const float dy = goal.y - pose.y;
    const float distance = std::hypot(dx, dy);
    const bool terminal_recovery =
        distance >= config.full_control_distance &&
        distance < config.recovery_distance &&
        nominal.vx < config.recovery_speed_threshold;
    if ((distance >= config.activation_distance && !terminal_recovery) || dt <= 0.0f) {
        return nominal;
    }

    Twist2D docking;
    const float eta_v = disturbance.valid ?
        clamp(disturbance.longitudinal_eta, 0.3f, 1.0f) : 1.0f;
    const float eta_w = disturbance.valid ?
        clamp(disturbance.yaw_eta, 0.3f, 1.0f) : 1.0f;
    const float yaw_disturbance = disturbance.valid ? disturbance.yaw_rate : 0.0f;
    if (distance <= config.position_tolerance) {
        const float yaw_error = shortestAngularDistance(pose.theta, goal.theta);
        docking.vx = 0.0f;
        docking.wz = clamp(
            (config.final_yaw_gain * yaw_error - yaw_disturbance) / eta_w,
                           -config.maximum_angular_speed,
                           config.maximum_angular_speed);
    } else {
        const float bearing = std::atan2(dy, dx);
        const float braking_speed = std::sqrt(
            2.0f * std::max(0.05f, config.goal_deceleration) * distance);
        const float maximum_speed = terminal_recovery ?
            config.recovery_maximum_linear_speed : config.maximum_linear_speed;
        const float target_speed = std::min({
            maximum_speed,
            config.distance_gain * distance,
            braking_speed});
        const float slip_angle = disturbance.valid ? std::atan2(
            disturbance.lateral_velocity, std::max(target_speed, 0.10f)) : 0.0f;
        const float desired_body_yaw = bearing - slip_angle;
        const float bearing_error = shortestAngularDistance(
            pose.theta, desired_body_yaw);
        const float terminal_orientation_error = shortestAngularDistance(
            desired_body_yaw, goal.theta);
        const float heading_factor = std::pow(
            std::max(0.0f, std::cos(bearing_error)), 2.0f);
        docking.vx = target_speed * heading_factor / eta_v;
        docking.wz = clamp(
            (config.bearing_gain * bearing_error -
             config.pose_alignment_gain * terminal_orientation_error -
             yaw_disturbance) / eta_w,
                           -config.maximum_angular_speed,
                           config.maximum_angular_speed);
    }

    const float denominator = std::max(
        config.activation_distance - config.full_control_distance, 0.01f);
    const float blend = terminal_recovery ? 1.0f : smoothStep(
        (config.activation_distance - distance) / denominator);
    Twist2D command;
    const float desired_vx = (1.0f - blend) * nominal.vx + blend * docking.vx;
    const float desired_wz = (1.0f - blend) * nominal.wz + blend * docking.wz;
    command.vx = clamp(
        desired_vx,
        speed.vx - config.linear_acceleration * dt,
        speed.vx + config.linear_acceleration * dt);
    command.vx = std::max(0.0f, command.vx);
    command.wz = clamp(
        desired_wz,
        speed.wz - config.angular_acceleration * dt,
        speed.wz + config.angular_acceleration * dt);
    command.wz = clamp(command.wz,
                       -config.maximum_angular_speed,
                       config.maximum_angular_speed);
    return command;
}

}  // namespace mppi

#endif  // MPPI_TOOLS_GOAL_DOCKING_HPP_
