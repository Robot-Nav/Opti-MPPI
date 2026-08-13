#ifndef MPPI_TOOLS_PATH_PROJECTION_HPP_
#define MPPI_TOOLS_PATH_PROJECTION_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "models/path.hpp"
#include "tools/math_utils.hpp"

namespace mppi
{

struct PathProjection
{
    size_t segment_index = 0;
    float ratio = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
    float signed_lateral_error = 0.0f;
    float distance = std::numeric_limits<float>::max();
    float arc_length = 0.0f;
    bool valid = false;
};

inline std::vector<float> computePathArcLengths(const Path & path)
{
    std::vector<float> arc(path.size(), 0.0f);
    for (size_t i = 1; i < path.size(); ++i) {
        arc[i] = arc[i - 1] + std::hypot(path.x(i) - path.x(i - 1),
                                        path.y(i) - path.y(i - 1));
    }
    return arc;
}

inline PathProjection projectPointToPath(
    const Path & path,
    const std::vector<float> & arc_lengths,
    float px,
    float py,
    size_t start_segment = 0,
    size_t end_segment_exclusive = std::numeric_limits<size_t>::max())
{
    PathProjection best;
    if (path.size() < 2 || arc_lengths.size() != path.size()) {
        return best;
    }

    const size_t segment_count = path.size() - 1;
    start_segment = std::min(start_segment, segment_count - 1);
    const size_t end_segment = std::min(end_segment_exclusive, segment_count);
    if (start_segment >= end_segment) {
        return best;
    }

    float best_dist_sq = std::numeric_limits<float>::max();
    for (size_t i = start_segment; i < end_segment; ++i) {
        const float x0 = path.x(i);
        const float y0 = path.y(i);
        const float dx = path.x(i + 1) - x0;
        const float dy = path.y(i + 1) - y0;
        const float len_sq = dx * dx + dy * dy;
        if (len_sq <= EPSILON * EPSILON) {
            continue;
        }

        const float ratio = clamp(((px - x0) * dx + (py - y0) * dy) / len_sq,
                                  0.0f, 1.0f);
        const float qx = x0 + ratio * dx;
        const float qy = y0 + ratio * dy;
        const float ex = px - qx;
        const float ey = py - qy;
        const float dist_sq = ex * ex + ey * ey;

        if (dist_sq < best_dist_sq) {
            const float len = std::sqrt(len_sq);
            best_dist_sq = dist_sq;
            best.segment_index = i;
            best.ratio = ratio;
            best.x = qx;
            best.y = qy;
            best.yaw = std::atan2(dy, dx);
            best.signed_lateral_error = (dx * ey - dy * ex) / len;
            best.distance = std::sqrt(dist_sq);
            best.arc_length = arc_lengths[i] + ratio * len;
            best.valid = true;
        }
    }
    return best;
}

}  // namespace mppi

#endif  // MPPI_TOOLS_PATH_PROJECTION_HPP_
