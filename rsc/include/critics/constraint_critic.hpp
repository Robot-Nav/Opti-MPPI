// ============================================================================
// 文件：constraint_critic.hpp
// 功能：约束代价，激励机器人在运动学和动力学约束范围内移动
//       支持差速驱动、全向、阿克曼等多种运动模型
// ============================================================================
#ifndef MPPI_CRITICS_CONSTRAINT_CRITIC_HPP_
#define MPPI_CRITICS_CONSTRAINT_CRITIC_HPP_

#include <algorithm>
#include <cmath>

#include <xtensor/xtensor.hpp>

#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"
#include "tools/math_utils.hpp"

namespace mppi
{

/**
 * @brief 约束代价：激励机器人在运动学和动力学约束范围内移动
 */
class ConstraintCritic : public CriticFunction
{
public:
    void initialize() override {}

    /**
     * @param weight 代价权重
     * @param vx_max 最大线速度
     * @param vx_min 最小线速度（允许负值表示倒车）
     * @param vy_max 最大横向速度（全向模型）
     * @param wz_max 最大角速度
     * @param min_turning_radius 最小转弯半径（阿克曼模型）
     * @param model_type 运动模型类型：0=DiffDrive, 1=Omni, 2=Ackermann
     */
    void setParams(float weight, float vx_max, float vx_min, float vy_max, float wz_max,
                   float min_turning_radius = 0.5f, int model_type = 0)
    {
        weight_ = std::max(0.0f, weight);
        setEnabled(weight_ > 0.0f);
        vx_max_ = vx_max;
        vx_min_ = vx_min;
        vy_max_ = vy_max;
        wz_max_ = wz_max;
        min_turning_radius_ = min_turning_radius;
        model_type_ = static_cast<MotionModelType>(model_type);
    }

    void score(CriticData & data) override
    {
        if (!enabled_) return;
        const size_t batch_size = data.state.vx.shape(0);
        const size_t traj_len = data.state.vx.shape(1);

        for (size_t i = 0; i < batch_size; ++i) {
            float cost = 0.0f;

            for (size_t j = 0; j < traj_len; ++j) {
                float vx = data.state.vx(i, j);
                float vy = data.state.vy(i, j);
                float wz = data.state.wz(i, j);

                switch (model_type_) {
                    case MotionModelType::DIFF_DRIVE:
                        {
                            float vx_violation = std::max(0.0f, vx - vx_max_) +
                                                std::max(0.0f, vx_min_ - vx);
                            float wz_violation = std::max(0.0f, std::abs(wz) - wz_max_);
                            cost += (vx_violation + wz_violation) * data.model_dt;
                        }
                        break;

                    case MotionModelType::OMNI:
                        {
                            float vel_total = std::copysign(std::hypot(vx, vy), vx);
                            float vel_violation = std::max(0.0f, vel_total - vx_max_) +
                                                 std::max(0.0f, vx_min_ - vel_total);
                            float wz_violation = std::max(0.0f, std::abs(wz) - wz_max_);
                            cost += (vel_violation + wz_violation) * data.model_dt;
                        }
                        break;

                    case MotionModelType::ACKERMANN:
                        {
                            float vx_violation = std::max(0.0f, vx - vx_max_) +
                                                std::max(0.0f, vx_min_ - vx);
                            float wz_violation = std::max(0.0f, std::abs(wz) - wz_max_);

                            float turning_radius_violation = 0.0f;
                            if (std::abs(vx) > EPSILON && std::abs(wz) > EPSILON) {
                                float radius = std::abs(vx / wz);
                                turning_radius_violation = std::max(0.0f, min_turning_radius_ - radius);
                            }

                            cost += (vx_violation + wz_violation + turning_radius_violation) * data.model_dt;
                        }
                        break;
                }
            }

            data.costs(i) += weight_ * cost;
        }
    }

private:
    enum class MotionModelType {
        DIFF_DRIVE = 0,
        OMNI = 1,
        ACKERMANN = 2
    };

    float weight_ = 4.0f;
    float vx_max_ = 1.0f;
    float vx_min_ = 0.0f;
    float vy_max_ = 0.0f;
    float wz_max_ = 1.0f;
    float min_turning_radius_ = 0.5f;
    MotionModelType model_type_ = MotionModelType::DIFF_DRIVE;
};

} // namespace mppi

#endif // MPPI_CRITICS_CONSTRAINT_CRITIC_HPP_
