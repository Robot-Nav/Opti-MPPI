#ifndef MPPI_CRITICS_CONTROL_RATE_CRITIC_HPP_
#define MPPI_CRITICS_CONTROL_RATE_CRITIC_HPP_

#include <algorithm>
#include <cmath>

#include "critics/critic_data.hpp"
#include "critics/critic_function.hpp"

namespace mppi
{

class ControlRateCritic : public CriticFunction
{
public:
    void initialize() override {}

    void setParams(float weight,
                   float vx_weight,
                   float wz_weight,
                   float vx_rate_scale,
                   float wz_rate_scale,
                   int trajectory_step)
    {
        weight_ = std::max(0.0f, weight);
        vx_weight_ = std::max(0.0f, vx_weight);
        wz_weight_ = std::max(0.0f, wz_weight);
        vx_rate_scale_ = std::max(0.01f, vx_rate_scale);
        wz_rate_scale_ = std::max(0.01f, wz_rate_scale);
        trajectory_step_ = std::max(1, trajectory_step);
        setEnabled(weight_ > 0.0f);
    }

    void score(CriticData & data) override
    {
        if (!enabled_ || weight_ <= 0.0f || data.model_dt <= 0.0f) return;

        const size_t batch_size = data.state.cvx.shape(0);
        const size_t trajectory_length = data.state.cvx.shape(1);
        if (trajectory_length < 2) return;

        for (size_t i = 0; i < batch_size; ++i) {
            float cost = 0.0f;
            size_t count = 0;
            float previous_vx = data.previous_control.vx;
            float previous_wz = data.previous_control.wz;

            for (size_t j = 1; j < trajectory_length;
                 j += static_cast<size_t>(trajectory_step_)) {
                const float rate_dt = data.model_dt *
                    static_cast<float>(j == 1 ? 1 : trajectory_step_);
                // Smooth the actuator command, not the model-realized speed.
                // Under eta<1 the latter attenuates command changes and would
                // make the DC case artificially cheap, allowing cmd_vel jitter.
                const float vx_rate =
                    (data.state.cvx(i, j) - previous_vx) / rate_dt;
                const float wz_rate =
                    (data.state.cwz(i, j) - previous_wz) / rate_dt;
                const float normalized_vx_rate = vx_rate / vx_rate_scale_;
                const float normalized_wz_rate = wz_rate / wz_rate_scale_;

                cost += vx_weight_ * normalized_vx_rate * normalized_vx_rate +
                        wz_weight_ * normalized_wz_rate * normalized_wz_rate;
                previous_vx = data.state.cvx(i, j);
                previous_wz = data.state.cwz(i, j);
                ++count;
            }

            if (count > 0) {
                data.costs(i) += weight_ * cost / static_cast<float>(count);
            }
        }
    }

private:
    float weight_ = 2.0f;
    float vx_weight_ = 0.5f;
    float wz_weight_ = 1.0f;
    float vx_rate_scale_ = 1.5f;
    float wz_rate_scale_ = 2.5f;
    int trajectory_step_ = 1;
};

}  // namespace mppi

#endif  // MPPI_CRITICS_CONTROL_RATE_CRITIC_HPP_
