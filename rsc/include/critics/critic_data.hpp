#ifndef MPPI_CRITICS_CRITIC_DATA_HPP_
#define MPPI_CRITICS_CRITIC_DATA_HPP_

#include <memory>
#include <optional>

#include <xtensor/xtensor.hpp>

#include "models/path.hpp"
#include "models/state.hpp"
#include "models/trajectories.hpp"
#include "models/types.hpp"

namespace mppi
{

class MotionModel;

struct CriticData
{
    const State & state;
    const Trajectories & trajectories;
    const Path & path;                         // 当前裁剪后的局部路径
    const Pose2D global_goal;                  // 完整全局路径终点
    const Twist2D previous_control;            // 上一周期实际下发控制
    xt::xtensor<float, 1> & costs;
    const float model_dt;
    std::shared_ptr<MotionModel> motion_model;
    std::optional<size_t> furthest_reached_path_point;
    float traction_scale = 1.0f;
    DisturbanceEstimate disturbance;
    float disturbance_decay_time_constant = 1.0f;
};

}  // namespace mppi

#endif  // MPPI_CRITICS_CRITIC_DATA_HPP_
