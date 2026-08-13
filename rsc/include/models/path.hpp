// ============================================================================
// 文件：path.hpp
// 功能：全局路径数据结构定义
// ============================================================================
#ifndef MPPI_MODELS_PATH_HPP_
#define MPPI_MODELS_PATH_HPP_

#include <xtensor/xtensor.hpp>
#include "models/types.hpp"

namespace mppi
{

/**
 * @brief 全局路径（离散点序列）
 */
struct Path
{
    xt::xtensor<float, 1> x;      ///< x 坐标
    xt::xtensor<float, 1> y;      ///< y 坐标
    xt::xtensor<float, 1> yaws;   ///< 朝向角

    /**
     * @brief 重置路径大小并清零
     * @param size 路径点数
     */
    void reset(unsigned int size)
    {
        x = xt::zeros<float>({size});
        y = xt::zeros<float>({size});
        yaws = xt::zeros<float>({size});
    }

    /** @return 路径点数 */
    size_t size() const { return x.shape(0); }
    /** @return 路径是否为空 */
    bool empty() const { return x.shape(0) == 0; }

    /**
     * @brief 获取路径终点位姿
     * @return 终点位姿（若路径为空则返回原点）
     */
    Pose2D getGoal() const
    {
        if (empty()) return Pose2D();
        size_t last = size() - 1;
        return Pose2D(x(last), y(last), yaws(last));
    }
};

} // namespace mppi

#endif // MPPI_MODELS_PATH_HPP_
