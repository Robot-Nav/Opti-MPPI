// ============================================================================
// 文件：path_align_critic.hpp
// 功能：路径对齐代价，计算轨迹上每个点与路径上最近点距离的平均值
//       支持路径朝向考虑
// ============================================================================
#ifndef MPPI_CRITICS_PATH_ALIGN_CRITIC_HPP_
#define MPPI_CRITICS_PATH_ALIGN_CRITIC_HPP_

#include <cmath>
#include <vector>
#include <algorithm>

#include <xtensor/xtensor.hpp>

#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"
#include "tools/math_utils.hpp"

namespace mppi
{

/**
 * @brief 路径对齐代价：计算轨迹上每个点与路径上最近点的距离的平均值
 * 使用路径点缓存减少搜索范围，支持路径朝向考虑
 */
class PathAlignCritic : public CriticFunction
{
public:
    void initialize() override {}

    /**
     * @param weight 代价权重
     * @param offset 从最远路径点向前偏移的步数
     * @param threshold 距离阈值
     * @param step 轨迹点采样步长（每隔 step 个点计算一次）
     * @param use_path_orientations 是否考虑路径朝向
     */
    void setParams(float weight, int offset, float threshold, int step,
                   bool use_path_orientations = false)
    {
        weight_ = std::max(0.0f, weight);
        offset_from_furthest_ = std::max(0, offset);
        threshold_to_consider_ = std::max(0.0f, threshold);
        traj_point_step_ = std::max(1, step);
        setEnabled(weight_ > 0.0f);
        use_path_orientations_ = use_path_orientations;
    }

    void score(CriticData & data) override
    {
        if (!enabled_ || data.path.size() < 2) return;

        const Pose2D & goal = data.global_goal;
        float dist_to_goal = std::hypot(data.state.pose.x - goal.x, data.state.pose.y - goal.y);
        if (dist_to_goal < threshold_to_consider_) return;

        const size_t batch_size = data.trajectories.x.shape(0);
        const size_t traj_len = data.trajectories.x.shape(1);

        // 路径已裁剪，索引0对应机器人当前位置
        size_t path_segments_count = data.path.size() - 1;
        if (data.furthest_reached_path_point) {
            path_segments_count = *data.furthest_reached_path_point;
        }

        path_segments_count = std::min(
            path_segments_count + static_cast<size_t>(offset_from_furthest_),
            data.path.size() - 1);
        if (path_segments_count < 1) return;

        // 包含索引 path_segments_count，因此数组大小为 +1。
        std::vector<float> path_integrated_distances(path_segments_count + 1, 0.0f);
        for (size_t i = 1; i <= path_segments_count; ++i) {
            float dx = data.path.x(i) - data.path.x(i - 1);
            float dy = data.path.y(i) - data.path.y(i - 1);
            path_integrated_distances[i] = path_integrated_distances[i - 1] + std::hypot(dx, dy);
        }

        for (size_t i = 0; i < batch_size; ++i) {
            float cost = 0.0f;
            int count = 0;
            float traj_integrated_distance = 0.0f;
            size_t path_pt = 0;

            for (size_t j = 0; j < traj_len; j += traj_point_step_) {
                float px = data.trajectories.x(i, j);
                float py = data.trajectories.y(i, j);

                if (j > 0) {
                    float prev_x = data.trajectories.x(i, j - traj_point_step_);
                    float prev_y = data.trajectories.y(i, j - traj_point_step_);
                    traj_integrated_distance += std::hypot(px - prev_x, py - prev_y);
                }

                path_pt = findClosestPathPointByDistance(path_integrated_distances, traj_integrated_distance, path_pt);

                if (path_pt > path_segments_count) path_pt = path_segments_count;

                float path_x = data.path.x(path_pt);
                float path_y = data.path.y(path_pt);
                float point_dist = std::hypot(px - path_x, py - path_y);

                if (use_path_orientations_) {
                    float traj_yaw = data.trajectories.yaws(i, j);
                    float path_yaw = data.path.yaws(path_pt);
                    float angle_diff = std::abs(shortestAngularDistance(traj_yaw, path_yaw));
                    point_dist *= (1.0f + 0.5f * angle_diff);
                }

                cost += point_dist;
                count++;
            }
            if (count > 0) data.costs(i) += weight_ * (cost / count);
        }
    }

    /**
     * @brief 基于累计距离找到最近的路径点
     */
    size_t findClosestPathPointByDistance(const std::vector<float>& path_distances,
                                          float target_distance,
                                          size_t start_idx) const
    {
        size_t closest_idx = start_idx;
        float min_diff = std::abs(path_distances[start_idx] - target_distance);

        for (size_t i = start_idx + 1; i < path_distances.size(); ++i) {
            float diff = std::abs(path_distances[i] - target_distance);
            if (diff < min_diff) {
                min_diff = diff;
                closest_idx = i;
            } else if (diff > min_diff) {
                break;
            }
        }

        return closest_idx;
    }

private:
    float weight_ = 14.0f;
    float threshold_to_consider_ = 0.4f;
    int offset_from_furthest_ = 20;
    int traj_point_step_ = 4;
    bool use_path_orientations_ = false;
};

} // namespace mppi

#endif // MPPI_CRITICS_PATH_ALIGN_CRITIC_HPP_
