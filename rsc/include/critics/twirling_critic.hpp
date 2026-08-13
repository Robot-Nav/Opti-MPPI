#ifndef MPPI_CRITICS_TWIRLING_CRITIC_HPP_
#define MPPI_CRITICS_TWIRLING_CRITIC_HPP_

#include <algorithm>
#include <cmath>

#include "critics/critic_data.hpp"
#include "critics/critic_function.hpp"

namespace mppi
{

class TwirlingCritic : public CriticFunction
{
public:
    void initialize() override {}

    void setParams(float weight,
                   float goal_threshold = 0.5f,
                   float low_speed_threshold = 0.10f,
                   float free_yaw_rate = 0.40f)
    {
        weight_ = std::max(0.0f, weight);
        goal_threshold_ = std::max(0.0f, goal_threshold);
        low_speed_threshold_ = std::max(0.01f, low_speed_threshold);
        free_yaw_rate_ = std::max(0.0f, free_yaw_rate);
        setEnabled(weight_ > 0.0f);
    }

    void score(CriticData & data) override
    {
        if (!enabled_ || weight_ <= 0.0f) return;

        const float distance_to_goal = std::hypot(
            data.state.pose.x - data.global_goal.x,
            data.state.pose.y - data.global_goal.y);
        if (distance_to_goal < goal_threshold_) return;

        const size_t batch_size = data.state.wz.shape(0);
        const size_t trajectory_length = data.state.wz.shape(1);
        if (trajectory_length == 0) return;

        for (size_t i = 0; i < batch_size; ++i) {
            float cost = 0.0f;
            for (size_t j = 0; j < trajectory_length; ++j) {
                const float speed = std::abs(data.state.vx(i, j));
                const float low_speed_ratio = std::max(
                    0.0f,
                    (low_speed_threshold_ - speed) / low_speed_threshold_);
                const float yaw_excess = std::max(
                    0.0f,
                    std::abs(data.state.wz(i, j)) - free_yaw_rate_);
                cost += low_speed_ratio * yaw_excess * yaw_excess * data.model_dt;
            }
            data.costs(i) += weight_ * cost;
        }
    }

private:
    float weight_ = 0.0f;
    float goal_threshold_ = 0.5f;
    float low_speed_threshold_ = 0.10f;
    float free_yaw_rate_ = 0.40f;
};

}  // namespace mppi

#endif  // MPPI_CRITICS_TWIRLING_CRITIC_HPP_
