// ============================================================================
// 文件：constraints.hpp
// 功能：控制量约束与采样噪声参数定义
// ============================================================================
#ifndef MPPI_MODELS_CONSTRAINTS_HPP_
#define MPPI_MODELS_CONSTRAINTS_HPP_

namespace mppi
{

/**
 * @brief 控制量约束
 */
struct ControlConstraints
{
    float vx_max = 1.0f;
    float vx_min = 0.0f;
    float vy_max = 0.0f;
    float wz_max = 1.0f;
    // 加速度限制
    float ax_max = 1.2f;
    float ay_max = 0.0f;
    float az_max = 2.5f;
};

/**
 * @brief 采样噪声标准差
 */
struct SamplingStd
{
    float vx = 0.2f;
    float vy = 0.0f;
    float wz = 0.4f;
};

} // namespace mppi

#endif // MPPI_MODELS_CONSTRAINTS_HPP_
