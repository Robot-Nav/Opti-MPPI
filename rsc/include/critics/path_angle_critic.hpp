// ============================================================================
// 文件：path_angle_critic.hpp
// 功能：路径角度代价，惩罚轨迹终点与路径参考点之间的朝向偏差
//       支持多种角度模式：偏好前进、无偏好、考虑路径朝向
// ============================================================================
#ifndef MPPI_CRITICS_PATH_ANGLE_CRITIC_HPP_
#define MPPI_CRITICS_PATH_ANGLE_CRITIC_HPP_

#include <cmath>
#include <algorithm>

#include <xtensor/xtensor.hpp>

#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"
#include "tools/math_utils.hpp"

namespace mppi
{

/**
 * @brief 路径角度模式枚举
 */
enum class PathAngleMode {
    FORWARD_PREFERENCE = 0,              // 偏好前进方向
    NO_DIRECTIONAL_PREFERENCE = 1,       // 无方向偏好(允许倒车)
    CONSIDER_FEASIBLE_PATH_ORIENTATIONS = 2  // 考虑路径朝向
};

/**
 * @brief 路径角度代价：惩罚轨迹终点与路径参考点之间的朝向偏差
 */
class PathAngleCritic : public CriticFunction
{
public:
    void initialize() override {}

    /**
     * @param weight 代价权重
     * @param offset 从最远路径点向前偏移的步数
     * @param threshold 距离阈值
     * @param max_angle 最大允许角度偏差（超过此值才惩罚）
     * @param mode 路径角度模式（0=偏好前进，1=无偏好，2=考虑路径朝向）
     */
    void setParams(float weight, int offset, float threshold, float max_angle, int mode = 0)
    {
        weight_ = std::max(0.0f, weight);
        offset_from_furthest_ = std::max(0, offset);
        threshold_to_consider_ = std::max(0.0f, threshold);
        max_angle_ = std::max(0.0f, max_angle);
        setEnabled(weight_ > 0.0f);
        mode_ = static_cast<PathAngleMode>(mode);
    }

    void score(CriticData & data) override
    {
        if (!enabled_ || data.path.size() < 2) return;

        const Pose2D & goal = data.global_goal;
        float dist_to_goal = std::hypot(data.state.pose.x - goal.x, data.state.pose.y - goal.y);
        if (dist_to_goal < threshold_to_consider_) return;

        size_t path_end_idx = data.path.size() - 1;
        if (data.furthest_reached_path_point) {
            path_end_idx = std::min(*data.furthest_reached_path_point + offset_from_furthest_,
                                    data.path.size() - 1);
        }

        float target_x = data.path.x(path_end_idx);
        float target_y = data.path.y(path_end_idx);

        float angle_to_target = std::atan2(target_y - data.state.pose.y,
                                           target_x - data.state.pose.x);
        float robot_angle_diff = std::abs(shortestAngularDistance(data.state.pose.theta, angle_to_target));

        if (mode_ == PathAngleMode::NO_DIRECTIONAL_PREFERENCE) {
            float diff2 = std::abs(shortestAngularDistance(data.state.pose.theta,
                             normalizeAngle(angle_to_target + PI)));
            robot_angle_diff = std::min(robot_angle_diff, diff2);
        }

        (void)robot_angle_diff;

        const size_t batch_size = data.trajectories.yaws.shape(0);
        const size_t traj_len = data.trajectories.yaws.shape(1);

        for (size_t i = 0; i < batch_size; ++i) {
            float last_x = data.trajectories.x(i, traj_len - 1);
            float last_y = data.trajectories.y(i, traj_len - 1);
            float last_yaw = data.trajectories.yaws(i, traj_len - 1);

            float traj_angle_to_target = std::atan2(target_y - last_y, target_x - last_x);
            float angle_diff = 0.0f;

            switch (mode_) {
                case PathAngleMode::FORWARD_PREFERENCE:
                    angle_diff = std::abs(shortestAngularDistance(last_yaw, traj_angle_to_target));
                    break;

                case PathAngleMode::NO_DIRECTIONAL_PREFERENCE:
                    {
                        float diff1 = std::abs(shortestAngularDistance(last_yaw, traj_angle_to_target));
                        float diff2 = std::abs(shortestAngularDistance(last_yaw, normalizeAngle(traj_angle_to_target + PI)));
                        angle_diff = std::min(diff1, diff2);
                    }
                    break;

                case PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS:
                    {
                        // 使用路径朝向修正目标角度（对齐 Nav2）
                        float goal_yaw = data.path.yaws(path_end_idx);
                        float yaws_between = shortestAngularDistance(last_yaw, traj_angle_to_target);
                        float yaws_to_goal = shortestAngularDistance(last_yaw, goal_yaw);

                        // 如果轨迹朝向更接近路径朝向，使用路径朝向
                        if (std::abs(yaws_to_goal) < PI / 2.0f) {
                            angle_diff = std::abs(yaws_to_goal);
                        } else {
                            angle_diff = std::abs(shortestAngularDistance(last_yaw, normalizeAngle(traj_angle_to_target + PI)));
                        }
                    }
                    break;
            }

            const float excess = std::max(0.0f, angle_diff - max_angle_);
            data.costs(i) += weight_ * excess * excess;
        }
    }

private:
    float weight_ = 2.0f;
    float threshold_to_consider_ = 0.4f;
    float max_angle_ = 0.8f;
    int offset_from_furthest_ = 4;
    PathAngleMode mode_ = PathAngleMode::FORWARD_PREFERENCE;
};

} // namespace mppi

#endif // MPPI_CRITICS_PATH_ANGLE_CRITIC_HPP_
