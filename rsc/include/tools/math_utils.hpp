// ============================================================================
// 文件：math_utils.hpp
// 功能：基础数学常量与工具函数
// ============================================================================
#ifndef MPPI_TOOLS_MATH_UTILS_HPP_
#define MPPI_TOOLS_MATH_UTILS_HPP_

#include <cmath>
#include <algorithm>

namespace mppi
{

// 常量定义
constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;
constexpr float EPSILON = 1e-6f;

/**
 * @brief 将角度规范化到 [-PI, PI] 区间
 * @param angle 输入角度（弧度）
 * @return 规范化后的角度
 */
inline float normalizeAngle(float angle)
{
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

/**
 * @brief 计算两个角度之间的最短有符号距离
 * @param from 起始角度
 * @param to 目标角度
 * @return 从 from 到 to 的最短角度差（弧度，范围 [-PI, PI]）
 */
inline float shortestAngularDistance(float from, float to)
{
    return normalizeAngle(to - from);
}

/**
 * @brief 将值限制在指定范围内
 * @param val 输入值
 * @param min_val 最小值
 * @param max_val 最大值
 * @return 限制后的值
 */
inline float clamp(float val, float min_val, float max_val)
{
    return std::max(min_val, std::min(val, max_val));
}

} // namespace mppi

#endif // MPPI_TOOLS_MATH_UTILS_HPP_
