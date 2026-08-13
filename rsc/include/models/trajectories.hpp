// ============================================================================
// 文件：trajectories.hpp
// 功能：轨迹容器数据结构定义
// ============================================================================
#ifndef MPPI_MODELS_TRAJECTORIES_HPP_
#define MPPI_MODELS_TRAJECTORIES_HPP_

#include <xtensor/xtensor.hpp>

namespace mppi
{

/**
 * @brief 轨迹容器，存储所有采样轨迹的位置和朝向
 */
struct Trajectories
{
    xt::xtensor<float, 2> x;      ///< x 坐标 (batch, time)
    xt::xtensor<float, 2> y;      ///< y 坐标 (batch, time)
    xt::xtensor<float, 2> yaws;   ///< 朝向角 (batch, time)
    xt::xtensor<float, 2> times;  ///< 每个点的时间戳 (batch, time)

    /**
     * @brief 重置轨迹容器大小并清零
     * @param batch_size 采样批次数
     * @param time_steps 时间步数
     */
    void reset(unsigned int batch_size, unsigned int time_steps)
    {
        x = xt::zeros<float>({batch_size, time_steps});
        y = xt::zeros<float>({batch_size, time_steps});
        yaws = xt::zeros<float>({batch_size, time_steps});
        times = xt::zeros<float>({batch_size, time_steps});
    }
};

} // namespace mppi

#endif // MPPI_MODELS_TRAJECTORIES_HPP_
