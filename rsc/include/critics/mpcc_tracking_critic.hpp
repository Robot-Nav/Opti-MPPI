#ifndef MPPI_CRITICS_MPCC_TRACKING_CRITIC_HPP_
#define MPPI_CRITICS_MPCC_TRACKING_CRITIC_HPP_

#include <algorithm>
#include <cmath>

#include "critics/critic_data.hpp"
#include "critics/critic_function.hpp"
#include "tools/math_utils.hpp"
#include "tools/path_projection.hpp"
#include "tools/path_reference.hpp"

namespace mppi
{

// MPCC-style path-following objective. The projected arc length is the virtual
// progress state; contour, lag, heading and feed-forward control errors are
// evaluated along the complete horizon.
class MPCCTrackingCritic : public CriticFunction
{
public:
    void initialize() override {}

    void setParams(float weight,
                   float contour_weight,
                   float lag_weight,
                   float heading_weight,
                   float velocity_weight,
                   float yaw_rate_weight,
                   float terminal_weight,
                   float contour_scale,
                   float lag_scale,
                   float heading_scale,
                   float velocity_scale,
                   float yaw_rate_scale,
                   float reference_speed,
                   float max_lateral_acceleration,
                   float goal_deceleration,
                   float corner_preview_distance,
                   float corner_deceleration,
                   int trajectory_step)
    {
        weight_ = std::max(0.0f, weight);
        contour_weight_ = std::max(0.0f, contour_weight);
        lag_weight_ = std::max(0.0f, lag_weight);
        heading_weight_ = std::max(0.0f, heading_weight);
        velocity_weight_ = std::max(0.0f, velocity_weight);
        yaw_rate_weight_ = std::max(0.0f, yaw_rate_weight);
        terminal_weight_ = std::max(0.0f, terminal_weight);
        contour_scale_ = std::max(0.01f, contour_scale);
        lag_scale_ = std::max(0.01f, lag_scale);
        heading_scale_ = std::max(0.01f, heading_scale);
        velocity_scale_ = std::max(0.01f, velocity_scale);
        yaw_rate_scale_ = std::max(0.01f, yaw_rate_scale);
        reference_speed_ = std::max(0.0f, reference_speed);
        max_lateral_acceleration_ = std::max(0.05f, max_lateral_acceleration);
        goal_deceleration_ = std::max(0.05f, goal_deceleration);
        corner_preview_distance_ = std::max(0.0f, corner_preview_distance);
        corner_deceleration_ = std::max(0.05f, corner_deceleration);
        trajectory_step_ = std::max(1, trajectory_step);
        setEnabled(weight_ > 0.0f);
    }

    void setThreadCount(unsigned int count) { thread_count_ = std::max(1u, count); }

    void score(CriticData & data) override
    {
        if (!enabled_ || weight_ <= 0.0f || data.path.size() < 2) return;
        const auto arc_lengths = computePathArcLengths(data.path);
        if (arc_lengths.empty() || arc_lengths.back() <= EPSILON) return;

        const PathProjection current = projectPointToPath(
            data.path, arc_lengths, data.state.pose.x, data.state.pose.y);
        if (!current.valid) return;

        const bool contains_goal = localPathContainsGoal(data.path, data.global_goal);
        const size_t batch_size = data.trajectories.x.shape(0);
        const size_t trajectory_length = data.trajectories.x.shape(1);
        const float remaining_from_robot = arc_lengths.back() - current.arc_length;
        const float horizon_distance = reference_speed_ * data.model_dt *
            static_cast<float>(trajectory_length > 0 ? trajectory_length - 1 : 0);
        const float goal_blend = contains_goal ? clamp(
            (horizon_distance + 0.30f - remaining_from_robot) / 0.30f,
            0.0f, 1.0f) : 0.0f;
        std::vector<float> expected_progress(trajectory_length, current.arc_length);
        std::vector<float> expected_speeds(trajectory_length, 0.0f);
        for (size_t j = 0; j < trajectory_length; ++j) {
            expected_speeds[j] = previewReferenceSpeed(
                data.path, arc_lengths, expected_progress[j], contains_goal,
                reference_speed_, max_lateral_acceleration_, goal_deceleration_,
                corner_preview_distance_, corner_deceleration_);
            if (j + 1 < trajectory_length) {
                expected_progress[j + 1] = std::min(
                    arc_lengths.back(), expected_progress[j] +
                    expected_speeds[j] * data.model_dt);
            }
        }
        const int num_threads = static_cast<int>(std::min(
            thread_count_, static_cast<unsigned int>(batch_size)));

#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(num_threads)
#endif
        for (long long sample = 0; sample < static_cast<long long>(batch_size); ++sample) {
            const size_t i = static_cast<size_t>(sample);
            const size_t first_step = std::min<size_t>(
                static_cast<size_t>(trajectory_step_), trajectory_length - 1);
            float accumulated_cost = 0.0f;
            float accumulated_weight = 0.0f;
            size_t search_segment = current.segment_index;

            for (size_t j = first_step; j < trajectory_length;
                 j += static_cast<size_t>(trajectory_step_)) {
                const PathReference expected_reference = samplePathReference(
                    data.path, arc_lengths, expected_progress[j]);
                if (!expected_reference.valid) continue;
                const float expected_speed = expected_speeds[j];

                const PathProjection projection = projectPointToPath(
                    data.path, arc_lengths,
                    data.trajectories.x(i, j), data.trajectories.y(i, j),
                    search_segment,
                    std::min(data.path.size() - 1,
                             search_segment + projection_search_segments_));
                if (!projection.valid) continue;
                search_segment = projection.segment_index;

                const PathReference projected_reference = samplePathReference(
                    data.path, arc_lengths, projection.arc_length);
                const float contour = projection.signed_lateral_error / contour_scale_;
                const float lag = (expected_progress[j] - projection.arc_length) / lag_scale_;
                const float disturbance_decay = data.disturbance.valid ? std::exp(
                    -static_cast<float>(j) * data.model_dt /
                    std::max(data.disturbance_decay_time_constant, 0.05f)) : 0.0f;
                const float slip_angle = std::atan2(
                    disturbance_decay * data.disturbance.lateral_velocity,
                    std::max(expected_speed, 0.10f));
                const float desired_body_yaw = projection.yaw - slip_angle;
                const float heading = shortestAngularDistance(
                    desired_body_yaw, data.trajectories.yaws(i, j)) / heading_scale_;
                const float velocity =
                    (data.state.vx(i, j) - expected_speed) / velocity_scale_;
                const float expected_yaw_rate =
                    expected_speed * projected_reference.curvature;
                const float realized_yaw_rate = data.state.wz(i, j) +
                    disturbance_decay * data.disturbance.yaw_rate;
                const float yaw_rate =
                    (realized_yaw_rate - expected_yaw_rate) / yaw_rate_scale_;
                const float normalized_time = trajectory_length > 1 ?
                    static_cast<float>(j) /
                    static_cast<float>(trajectory_length - 1) : 0.0f;
                const float time_weight = 1.0f + normalized_time;

                accumulated_cost += time_weight * (
                    contour_weight_ * contour * contour +
                    lag_weight_ * lag * lag +
                    heading_weight_ * heading * heading +
                    velocity_weight_ * velocity * velocity +
                    yaw_rate_weight_ * yaw_rate * yaw_rate);
                accumulated_weight += time_weight;

            }

            if (accumulated_weight > EPSILON) {
                data.costs(i) += weight_ * accumulated_cost / accumulated_weight;
            }

            const size_t terminal = trajectory_length - 1;
            const float dx = data.trajectories.x(i, terminal) - data.global_goal.x;
            const float dy = data.trajectories.y(i, terminal) - data.global_goal.y;
            if (goal_blend > 0.0f && terminal_weight_ > 0.0f) {
                const float goal_position = std::hypot(dx, dy) / lag_scale_;
                const float goal_heading = shortestAngularDistance(
                    data.global_goal.theta,
                    data.trajectories.yaws(i, terminal)) / heading_scale_;
                const float terminal_speed = data.state.vx(i, terminal) / velocity_scale_;
                data.costs(i) += weight_ * terminal_weight_ * goal_blend * (
                    goal_position * goal_position +
                    0.5f * goal_heading * goal_heading +
                    0.5f * terminal_speed * terminal_speed);
            }
        }
    }

private:
    float weight_ = 8.0f;
    float contour_weight_ = 1.0f;
    float lag_weight_ = 0.35f;
    float heading_weight_ = 0.70f;
    float velocity_weight_ = 0.25f;
    float yaw_rate_weight_ = 0.20f;
    float terminal_weight_ = 2.0f;
    float contour_scale_ = 0.20f;
    float lag_scale_ = 0.35f;
    float heading_scale_ = 0.40f;
    float velocity_scale_ = 0.40f;
    float yaw_rate_scale_ = 0.80f;
    float reference_speed_ = 0.60f;
    float max_lateral_acceleration_ = 0.60f;
    float goal_deceleration_ = 0.50f;
    float corner_preview_distance_ = 1.20f;
    float corner_deceleration_ = 0.90f;
    int trajectory_step_ = 2;
    size_t projection_search_segments_ = 12;
    unsigned int thread_count_ = 4;
};

}  // namespace mppi

#endif  // MPPI_CRITICS_MPCC_TRACKING_CRITIC_HPP_
