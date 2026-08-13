// ============================================================================
// 文件：prefer_forward_critic.hpp
// 功能：偏好前进代价，惩罚控制序列中负速度（倒车）的出现
// ============================================================================
#ifndef MPPI_CRITICS_PREFER_FORWARD_CRITIC_HPP_
#define MPPI_CRITICS_PREFER_FORWARD_CRITIC_HPP_

#include <algorithm>
#include <cmath>

#include <xtensor/xtensor.hpp>

#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"

namespace mppi
{

/**
 * @brief 偏好前进代价：惩罚控制序列中负速度（倒车）的出现
 */
class PreferForwardCritic : public CriticFunction
{
public:
    void initialize() override {}

    /**
     * @param weight 代价权重
     * @param threshold 距离阈值，大于该值才激活（即远处才惩罚倒车）
     */
    void setParams(float weight, float threshold)
    {
        weight_ = std::max(0.0f, weight);
        threshold_to_consider_ = std::max(0.0f, threshold);
        setEnabled(weight_ > 0.0f);
    }

    void score(CriticData & data) override
    {
        if (!enabled_) return;
        const Pose2D & goal = data.global_goal;
        float dist_to_goal = std::hypot(data.state.pose.x - goal.x, data.state.pose.y - goal.y);
        if (dist_to_goal < threshold_to_consider_) return;

        const size_t batch_size = data.state.vx.shape(0);
        const size_t traj_len = data.state.vx.shape(1);

        for (size_t i = 0; i < batch_size; ++i) {
            float backward_cost = 0.0f;
            for (size_t j = 0; j < traj_len; ++j) {
                if (data.state.vx(i, j) < 0.0f) {
                    backward_cost += std::abs(data.state.vx(i, j)) * data.model_dt;
                }
            }
            data.costs(i) += weight_ * backward_cost;
        }
    }

private:
    float weight_ = 5.0f;
    float threshold_to_consider_ = 0.4f;
};

} // namespace mppi

#endif // MPPI_CRITICS_PREFER_FORWARD_CRITIC_HPP_
