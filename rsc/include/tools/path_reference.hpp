#ifndef MPPI_TOOLS_PATH_REFERENCE_HPP_
#define MPPI_TOOLS_PATH_REFERENCE_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

#include "models/path.hpp"
#include "models/types.hpp"
#include "tools/math_utils.hpp"
#include "tools/path_projection.hpp"

namespace mppi
{

struct PathReference
{
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
    float curvature = 0.0f;
    float arc_length = 0.0f;
    size_t segment_index = 0;
    bool valid = false;
};

inline float smoothMinimum(float lhs, float rhs, float transition)
{
    if (transition <= EPSILON) return std::min(lhs, rhs);
    const float h = clamp(0.5f + 0.5f * (rhs - lhs) / transition, 0.0f, 1.0f);
    return std::max(0.0f,
        rhs * (1.0f - h) + lhs * h - transition * h * (1.0f - h));
}

inline float pathCurvature(
    const Path & path, const std::vector<float> & arc_lengths, size_t segment)
{
    if (path.size() < 3 || arc_lengths.size() != path.size()) return 0.0f;
    const size_t center = std::min(segment + 1, path.size() - 1);
    const size_t first = center > 0 ? center - 1 : center;
    const size_t last = std::min(center + 1, path.size() - 1);
    const float ds = arc_lengths[last] - arc_lengths[first];
    if (ds <= EPSILON) return 0.0f;
    return shortestAngularDistance(path.yaws(first), path.yaws(last)) / ds;
}

inline PathReference samplePathReference(
    const Path & path, const std::vector<float> & arc_lengths, float arc_length)
{
    PathReference reference;
    if (path.size() < 2 || arc_lengths.size() != path.size()) return reference;

    const float bounded_s = clamp(arc_length, 0.0f, arc_lengths.back());
    const auto upper = std::upper_bound(arc_lengths.begin(), arc_lengths.end(), bounded_s);
    const size_t segment = upper == arc_lengths.begin() ? 0 :
        std::min<size_t>(static_cast<size_t>(upper - arc_lengths.begin() - 1),
                         path.size() - 2);
    const float segment_length = arc_lengths[segment + 1] - arc_lengths[segment];
    const float ratio = segment_length > EPSILON ?
        (bounded_s - arc_lengths[segment]) / segment_length : 0.0f;

    reference.x = path.x(segment) + ratio * (path.x(segment + 1) - path.x(segment));
    reference.y = path.y(segment) + ratio * (path.y(segment + 1) - path.y(segment));
    reference.yaw = std::atan2(path.y(segment + 1) - path.y(segment),
                               path.x(segment + 1) - path.x(segment));
    reference.curvature = pathCurvature(path, arc_lengths, segment);
    reference.arc_length = bounded_s;
    reference.segment_index = segment;
    reference.valid = true;
    return reference;
}

inline bool localPathContainsGoal(const Path & path, const Pose2D & global_goal)
{
    if (path.empty()) return false;
    const size_t last = path.size() - 1;
    return std::hypot(path.x(last) - global_goal.x,
                      path.y(last) - global_goal.y) < 0.15f;
}

inline float referenceSpeed(
    const PathReference & reference,
    float remaining_path,
    bool path_contains_goal,
    float cruise_speed,
    float max_lateral_acceleration,
    float goal_deceleration)
{
    float speed = std::max(0.0f, cruise_speed);
    if (std::abs(reference.curvature) > 1e-3f) {
        const float curvature_speed = std::sqrt(
            std::max(0.05f, max_lateral_acceleration) /
            std::abs(reference.curvature));
        speed = smoothMinimum(speed, curvature_speed, 0.08f);
    }
    if (path_contains_goal) {
        const float braking_speed = std::sqrt(
            2.0f * std::max(0.05f, goal_deceleration) *
            std::max(0.0f, remaining_path));
        speed = smoothMinimum(speed, braking_speed, 0.08f);
    }
    return clamp(speed, 0.0f, std::max(0.0f, cruise_speed));
}

// Curvature limits must be propagated backwards so a discontinuous path does
// not request braking only after the vehicle has already entered the corner.
inline float previewReferenceSpeed(
    const Path & path,
    const std::vector<float> & arc_lengths,
    float arc_length,
    bool path_contains_goal,
    float cruise_speed,
    float max_lateral_acceleration,
    float goal_deceleration,
    float preview_distance,
    float corner_deceleration)
{
    const PathReference current = samplePathReference(path, arc_lengths, arc_length);
    if (!current.valid || arc_lengths.empty()) return 0.0f;

    float speed = referenceSpeed(
        current, arc_lengths.back() - current.arc_length, path_contains_goal,
        cruise_speed, max_lateral_acceleration, goal_deceleration);
    const float preview_end = std::min(
        arc_lengths.back(), current.arc_length + std::max(0.0f, preview_distance));
    const auto first = std::lower_bound(
        arc_lengths.begin(), arc_lengths.end(), current.arc_length);
    const auto last = std::upper_bound(
        arc_lengths.begin(), arc_lengths.end(), preview_end);
    for (auto iterator = first; iterator != last; ++iterator) {
        const size_t index = static_cast<size_t>(iterator - arc_lengths.begin());
        const PathReference future = samplePathReference(
            path, arc_lengths, arc_lengths[index]);
        if (!future.valid || std::abs(future.curvature) <= 1e-3f) continue;
        const float corner_speed = referenceSpeed(
            future, arc_lengths.back() - future.arc_length, path_contains_goal,
            cruise_speed, max_lateral_acceleration, goal_deceleration);
        const float distance = std::max(
            0.0f, arc_lengths[index] - current.arc_length);
        const float braking_envelope = std::sqrt(
            corner_speed * corner_speed +
            2.0f * std::max(0.05f, corner_deceleration) * distance);
        speed = smoothMinimum(speed, braking_envelope, 0.04f);
    }
    return clamp(speed, 0.0f, std::max(0.0f, cruise_speed));
}

}  // namespace mppi

#endif  // MPPI_TOOLS_PATH_REFERENCE_HPP_
