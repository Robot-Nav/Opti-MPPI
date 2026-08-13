#ifndef MPPI_CRITICS_VELOCITY_DEADBAND_CRITIC_HPP_
#define MPPI_CRITICS_VELOCITY_DEADBAND_CRITIC_HPP_

#include <algorithm>
#include <cmath>

#include "critics/critic_data.hpp"
#include "critics/critic_function.hpp"
#include "motion_models.hpp"

namespace mppi
{

class VelocityDeadbandCritic : public CriticFunction
{
public:
    void initialize() override {}

    void setParams(float weight,
                   float deadband_vx,
                   float deadband_vy,
                   float deadband_wz,
                   float zero_tolerance = 1e-3f)
    {
        weight_ = std::max(0.0f, weight);
        deadband_vx_ = std::max(0.0f, deadband_vx);
        deadband_vy_ = std::max(0.0f, deadband_vy);
        deadband_wz_ = std::max(0.0f, deadband_wz);
        zero_tolerance_ = std::max(0.0f, zero_tolerance);
        setEnabled(weight_ > 0.0f &&
                   (deadband_vx_ > zero_tolerance_ ||
                    deadband_vy_ > zero_tolerance_ ||
                    deadband_wz_ > zero_tolerance_));
    }

    void score(CriticData & data) override
    {
        if (!enabled_ || weight_ <= 0.0f) return;

        const bool holonomic = data.motion_model && data.motion_model->isHolonomic();
        const size_t batch_size = data.state.vx.shape(0);
        const size_t trajectory_length = data.state.vx.shape(1);

        for (size_t i = 0; i < batch_size; ++i) {
            float cost = 0.0f;
            for (size_t j = 0; j < trajectory_length; ++j) {
                cost += deadbandPenalty(data.state.vx(i, j), deadband_vx_);
                if (holonomic) {
                    cost += deadbandPenalty(data.state.vy(i, j), deadband_vy_);
                }
                cost += deadbandPenalty(data.state.wz(i, j), deadband_wz_);
            }
            data.costs(i) += weight_ * cost * data.model_dt;
        }
    }

private:
    float deadbandPenalty(float value, float deadband) const
    {
        const float magnitude = std::abs(value);
        if (deadband <= zero_tolerance_ || magnitude <= zero_tolerance_ ||
            magnitude >= deadband) {
            return 0.0f;
        }
        const float normalized = (deadband - magnitude) / deadband;
        return normalized * normalized;
    }

    float weight_ = 0.0f;
    float deadband_vx_ = 0.0f;
    float deadband_vy_ = 0.0f;
    float deadband_wz_ = 0.0f;
    float zero_tolerance_ = 1e-3f;
};

}  // namespace mppi

#endif  // MPPI_CRITICS_VELOCITY_DEADBAND_CRITIC_HPP_
