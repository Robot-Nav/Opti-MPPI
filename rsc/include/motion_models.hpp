// ============================================================================
// 文件：motion_models.hpp
// 功能：运动模型基类及派生类（差速、全向、阿克曼），定义运动学预测接口
// ============================================================================
#ifndef MPPI_MOTION_MODELS_HPP_
#define MPPI_MOTION_MODELS_HPP_

#include <cmath>
#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "models/state.hpp"
#include "models/constraints.hpp"
#include "tools/math_utils.hpp"

namespace mppi
{

/**
 * @brief 运动模型基类，定义运动学预测接口
 */
class MotionModel
{
public:
    MotionModel() = default;
    virtual ~MotionModel() = default;

    /**
     * @brief 根据控制量预测速度（包含加速度约束）
     * @param state 状态容器，其中 cvx/cvy/cwz 为含噪声控制量，本函数将 vx/vy/wz 后移一位
     */
    virtual void predict(
        State & state, float longitudinal_effectiveness = 1.0f,
        float yaw_effectiveness = 1.0f)
    {
        using namespace xt::placeholders;

        longitudinal_effectiveness = clamp(longitudinal_effectiveness, 0.3f, 1.0f);
        yaw_effectiveness = clamp(yaw_effectiveness, 0.3f, 1.0f);

        // 第一时刻使用当前速度
        for (size_t i = 0; i < state.vx.shape(0); ++i) {
            state.vx(i, 0) = state.speed.vx;
            state.wz(i, 0) = state.speed.wz;
            if (isHolonomic()) {
                state.vy(i, 0) = state.speed.vy;
            }
        }

        // 后续时刻：应用加速度约束的速度传播
        for (size_t i = 0; i < state.vx.shape(0); ++i) {
            for (size_t j = 1; j < state.vx.shape(1); ++j) {
                float prev_vx = state.vx(i, j - 1);
                float prev_vy = isHolonomic() ? state.vy(i, j - 1) : 0.0f;
                float prev_wz = state.wz(i, j - 1);

                // Effectiveness scales the actuator target, not the already measured
                // current speed. This avoids applying eta twice at the first rollout step.
                float ctrl_vx = longitudinal_effectiveness * state.cvx(i, j - 1);
                float ctrl_vy = isHolonomic() ?
                    longitudinal_effectiveness * state.cvy(i, j - 1) : 0.0f;
                float ctrl_wz = yaw_effectiveness * state.cwz(i, j - 1);

                state.vx(i, j) = applyAccelerationConstraint(ctrl_vx, prev_vx, ax_max_, model_dt_);
                state.wz(i, j) = applyAccelerationConstraint(ctrl_wz, prev_wz, az_max_, model_dt_);
                if (isHolonomic()) {
                    state.vy(i, j) = applyAccelerationConstraint(ctrl_vy, prev_vy, ay_max_, model_dt_);
                }
            }
        }
    }

    /** @return 是否为全向移动模型 */
    virtual bool isHolonomic() const = 0;

    /**
     * @brief 对含噪声控制量施加运动学约束（如最小转弯半径）
     */
    virtual void applyConstraints(xt::xtensor<float, 2> & /*cvx*/,
                                   xt::xtensor<float, 2> & /*cvy*/,
                                   xt::xtensor<float, 2> & /*cwz*/) {}

    /** @brief 对最终输出的单步控制量施加模型约束 */
    virtual void applySingleControlConstraints(Control & /*control*/) const {}

    /**
     * @brief 设置加速度约束参数
     */
    void setAccelerationConstraints(float ax_max, float ay_max, float az_max, float model_dt)
    {
        ax_max_ = ax_max;
        ay_max_ = ay_max;
        az_max_ = az_max;
        model_dt_ = model_dt;
    }

protected:
    /**
     * @brief 应用加速度约束
     * @param desired 期望速度
     * @param current 当前速度
     * @param max_accel 最大加速度
     * @param dt 时间步长
     * @return 约束后的速度
     */
    float applyAccelerationConstraint(float desired, float current, float max_accel, float dt) const
    {
        float max_delta = max_accel * dt;
        float delta = desired - current;
        delta = clamp(delta, -max_delta, max_delta);
        return current + delta;
    }

    float ax_max_ = 1.2f;   // 最大线加速度
    float ay_max_ = 0.0f;   // 非完整三轮模型禁止主动横向速度
    float az_max_ = 2.5f;   // 最大角加速度
    float model_dt_ = 0.05f; // 时间步长
};

/**
 * @brief 差速运动模型（非全向）
 */
class DiffDriveMotionModel : public MotionModel
{
public:
    DiffDriveMotionModel() = default;
    bool isHolonomic() const override { return false; }
};

/**
 * @brief 全向运动模型
 */
class OmniMotionModel : public MotionModel
{
public:
    OmniMotionModel() = default;
    bool isHolonomic() const override { return true; }
};

/**
 * @brief 阿克曼运动模型（非全向，带最小转弯半径约束）
 */
class AckermannMotionModel : public MotionModel
{
public:
    /**
     * @param min_turning_r 最小转弯半径
     * @param max_lateral_acceleration 最大横向加速度（v*|wz|）
     */
    explicit AckermannMotionModel(float min_turning_r = 0.5f,
                                  float max_lateral_acceleration = 0.6f)
        : min_turning_r_(std::max(min_turning_r, EPSILON)),
          max_lateral_acceleration_(std::max(
              max_lateral_acceleration, EPSILON)) {}

    bool isHolonomic() const override { return false; }

    /**
     * @brief 加速度传播后再次投影到Ackermann可行域。
     *
     * vx与wz分别限加速度会在启动/制动阶段暂时产生无限曲率，因此不能只约束
     * 采样控制量；预测状态本身也必须满足相同的转弯半径和横向加速度边界。
     */
    void predict(State & state, float longitudinal_effectiveness = 1.0f,
                 float yaw_effectiveness = 1.0f) override
    {
        MotionModel::predict(state, longitudinal_effectiveness, yaw_effectiveness);
        for (size_t i = 0; i < state.vx.shape(0); ++i) {
            for (size_t j = 0; j < state.vx.shape(1); ++j) {
                const float max_abs_wz = feasibleYawRate(state.vx(i, j));
                state.wz(i, j) = clamp(
                    state.wz(i, j), -max_abs_wz, max_abs_wz);
            }
        }
    }

    /**
     * @brief 施加最小转弯半径约束
     */
    void applyConstraints(xt::xtensor<float, 2> & cvx,
                          xt::xtensor<float, 2> & /*cvy*/,
                          xt::xtensor<float, 2> & cwz) override
    {
        for (size_t i = 0; i < cvx.shape(0); ++i) {
            for (size_t j = 0; j < cvx.shape(1); ++j) {
                float vx_val = cvx(i, j);
                float wz_val = cwz(i, j);
                const float max_abs_wz = feasibleYawRate(vx_val);
                if (max_abs_wz <= EPSILON) {
                    cwz(i, j) = 0.0f;
                } else if (std::abs(wz_val) > EPSILON) {
                    cwz(i, j) = clamp(wz_val, -max_abs_wz, max_abs_wz);
                }
            }
        }
    }

    void applySingleControlConstraints(Control & control) const override
    {
        const float max_abs_wz = feasibleYawRate(control.vx);
        if (max_abs_wz <= EPSILON) {
            control.wz = 0.0f;
            return;
        }
        control.wz = clamp(control.wz, -max_abs_wz, max_abs_wz);
    }

private:
    float feasibleYawRate(float vx) const
    {
        const float speed = std::abs(vx);
        if (speed <= EPSILON) return 0.0f;
        const float curvature_limit = speed / min_turning_r_;
        const float lateral_acceleration_limit =
            max_lateral_acceleration_ / speed;
        return std::min(curvature_limit, lateral_acceleration_limit);
    }

    float min_turning_r_{0.5f};
    float max_lateral_acceleration_{0.6f};
};

} // namespace mppi

#endif // MPPI_MOTION_MODELS_HPP_
