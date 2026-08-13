// ============================================================================
// 文件：control_sequence.hpp
// 功能：控制序列数据结构定义
// ============================================================================
#ifndef MPPI_MODELS_CONTROL_SEQUENCE_HPP_
#define MPPI_MODELS_CONTROL_SEQUENCE_HPP_

#include <xtensor/xtensor.hpp>

namespace mppi
{

/**
 * @brief 控制序列（每个时间步的控制量）
 */
struct ControlSequence
{
    xt::xtensor<float, 1> vx;
    xt::xtensor<float, 1> vy;
    xt::xtensor<float, 1> wz;

    /**
     * @brief 重置控制序列大小并清零
     * @param time_steps 时间步数
     */
    void reset(unsigned int time_steps)
    {
        vx = xt::zeros<float>({time_steps});
        vy = xt::zeros<float>({time_steps});
        wz = xt::zeros<float>({time_steps});
    }
};

} // namespace mppi

#endif // MPPI_MODELS_CONTROL_SEQUENCE_HPP_
