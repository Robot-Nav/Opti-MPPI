// ============================================================================
// 文件：state.hpp
// 功能：状态容器数据结构定义
// ============================================================================
#ifndef MPPI_MODELS_STATE_HPP_
#define MPPI_MODELS_STATE_HPP_

#include <xtensor/xtensor.hpp>
#include "models/types.hpp"

namespace mppi
{

/**
 * @brief 状态容器，存储所有采样轨迹的控制量和速度
 */
struct State
{
    xt::xtensor<float, 2> vx;   ///< 速度 x (batch, time)
    xt::xtensor<float, 2> vy;   ///< 速度 y (batch, time)
    xt::xtensor<float, 2> wz;   ///< 角速度 (batch, time)
    xt::xtensor<float, 2> cvx;  ///< 含噪声的控制量 vx
    xt::xtensor<float, 2> cvy;  ///< 含噪声的控制量 vy
    xt::xtensor<float, 2> cwz;  ///< 含噪声的控制量 wz
    Pose2D pose;                 ///< 当前机器人位姿
    Twist2D speed;               ///< 当前机器人速度

    /**
     * @brief 重置状态容器大小并清零
     * @param batch_size 采样批次数
     * @param time_steps 时间步数
     */
    void reset(unsigned int batch_size, unsigned int time_steps)
    {
        vx = xt::zeros<float>({batch_size, time_steps});
        vy = xt::zeros<float>({batch_size, time_steps});
        wz = xt::zeros<float>({batch_size, time_steps});
        cvx = xt::zeros<float>({batch_size, time_steps});
        cvy = xt::zeros<float>({batch_size, time_steps});
        cwz = xt::zeros<float>({batch_size, time_steps});
    }
};

} // namespace mppi

#endif // MPPI_MODELS_STATE_HPP_
