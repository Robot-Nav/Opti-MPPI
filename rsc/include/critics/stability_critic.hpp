// ============================================================================
// 文件：stability_critic.hpp
// 功能：低附着稳定性代价，抑制过大的合加速度、横向加速度和横摆加速度
// ============================================================================
#ifndef MPPI_CRITICS_STABILITY_CRITIC_HPP_
#define MPPI_CRITICS_STABILITY_CRITIC_HPP_

#include <algorithm>
#include <cmath>

#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"

namespace mppi
{

class StabilityCritic : public CriticFunction
{
public:
    void initialize() override {}

    void setParams(float weight, float longitudinal_accel_limit,
                   float lateral_accel_limit, float yaw_accel_limit,
                   float excess_penalty)
    {
        weight_ = std::max(0.0f, weight);
        longitudinal_accel_limit_ = std::max(0.1f, longitudinal_accel_limit);
        lateral_accel_limit_ = std::max(0.1f, lateral_accel_limit);
        yaw_accel_limit_ = std::max(0.1f, yaw_accel_limit);
        excess_penalty_ = std::max(0.0f, excess_penalty);
    }

    void score(CriticData & data) override
    {
        if (!enabled_ || weight_ <= 0.0f || data.model_dt <= 0.0f) return;

        const size_t batch_size = data.state.vx.shape(0);
        const size_t traj_len = data.state.vx.shape(1);
        if (traj_len < 2) return;

        // traction_scale 是能力指标而非物理摩擦系数。降低它会保守地收紧
        // 可接受的加速度包络，从而避免湿滑/晃动下激进转向。
        const float traction = std::clamp(data.traction_scale, 0.1f, 1.0f);
        const float ax_limit = std::max(0.25f, longitudinal_accel_limit_ * traction);
        const float ay_limit = std::max(0.25f, lateral_accel_limit_ * traction);
        const float aw_limit = std::max(0.25f, yaw_accel_limit_ * traction);

        for (size_t i = 0; i < batch_size; ++i) {
            float cost = 0.0f;
            for (size_t j = 1; j < traj_len; ++j) {
                const float ax = (data.state.vx(i, j) - data.state.vx(i, j - 1)) / data.model_dt;
                const float ay = data.state.vx(i, j) * data.state.wz(i, j);
                const float aw = (data.state.wz(i, j) - data.state.wz(i, j - 1)) / data.model_dt;

                const float combined = std::hypot(ax / ax_limit, ay / ay_limit);
                const float yaw_usage = std::abs(aw) / aw_limit;
                const float excess = std::max(0.0f, combined - 1.0f);
                const float yaw_excess = std::max(0.0f, yaw_usage - 1.0f);

                cost += (combined * combined + 0.25f * yaw_usage * yaw_usage
                         + excess_penalty_ * (excess * excess + yaw_excess * yaw_excess))
                        * data.model_dt;
            }
            data.costs(i) += weight_ * cost / static_cast<float>(traj_len - 1);
        }
    }

private:
    float weight_ = 4.0f;
    float longitudinal_accel_limit_ = 1.5f;
    float lateral_accel_limit_ = 1.5f;
    float yaw_accel_limit_ = 2.5f;
    float excess_penalty_ = 8.0f;
};

} // namespace mppi

#endif // MPPI_CRITICS_STABILITY_CRITIC_HPP_
