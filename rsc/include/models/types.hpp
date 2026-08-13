// ============================================================================
// 文件：types.hpp
// 功能：基础数据类型定义（点、位姿、速度、控制量等）
// ============================================================================
#ifndef MPPI_MODELS_TYPES_HPP_
#define MPPI_MODELS_TYPES_HPP_

namespace mppi
{

/**
 * @brief 二维点结构
 */
struct Point2D
{
    float x = 0.0f;
    float y = 0.0f;
    Point2D() = default;
    Point2D(float x_, float y_) : x(x_), y(y_) {}
};

/**
 * @brief 二维位姿（位置 + 朝向）
 */
struct Pose2D
{
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
    Pose2D() = default;
    Pose2D(float x_, float y_, float theta_) : x(x_), y(y_), theta(theta_) {}
};

/**
 * @brief 二维速度（线速度 + 角速度）
 */
struct Twist2D
{
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    Twist2D() = default;
    Twist2D(float vx_, float vy_, float wz_) : vx(vx_), vy(vy_), wz(wz_) {}
};

/**
 * @brief 单步控制量
 */
struct Control
{
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
};

/**
 * @brief 在线估计的甲板运动/低附着等效扰动
 *
 * lateral_velocity 与 yaw_rate 是车体系中的等效模型残差；std 字段描述
 * 估计不确定性，供 rollout 进行随机动力学采样。traction_scale 不是摩擦
 * 系数，而是 [0, 1] 内的保守附着能力指标。
 */
struct DisturbanceEstimate
{
    float lateral_velocity = 0.0f;      // 等效侧向残差（侧滑+甲板运动+定位误差混合）
    float yaw_rate = 0.0f;              // 等效横摆残差
    float lateral_velocity_std = 0.0f;  // 侧向残差标准差
    float yaw_rate_std = 0.0f;          // 横摆残差标准差
    float traction_scale = 1.0f;        // 附着能力指标 [0,1]
    float longitudinal_eta = 1.0f;      // 纵向控制有效系数η_v（湿滑<1）
    float yaw_eta = 1.0f;              // 横摆控制有效系数η_ω（湿滑<1）
    float confidence = 0.0f;            // 估计可信度 [0,1]，用于渐进启用补偿
    bool valid = false;
};

} // namespace mppi

#endif // MPPI_MODELS_TYPES_HPP_
