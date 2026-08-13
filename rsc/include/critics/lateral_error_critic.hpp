// ============================================================================
// 文件：lateral_error_critic.hpp
// 功能：LQR最优横向误差代价，用LQR Riccati解P矩阵对CTE/HE进行最优加权
// ============================================================================
#ifndef MPPI_CRITICS_LATERAL_ERROR_CRITIC_HPP_
#define MPPI_CRITICS_LATERAL_ERROR_CRITIC_HPP_

#include <cmath>
#include <array>
#include <algorithm>

#include <xtensor/xtensor.hpp>

#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"
#include "tools/math_utils.hpp"

namespace mppi
{

/**
 * @brief LQR最优横向误差代价
 *
 * 对每条轨迹逐点计算：
 *   - CTE（Cross-Track Error）：轨迹点到路径的横向距离
 *   - HE  （Heading Error）：轨迹朝向与路径朝向偏差
 * 然后用 LQR Riccati 解 P 矩阵进行最优二次加权：
 *   cost = [cte, he] * P * [cte; he]
 *
 * LQR 状态空间（线性化横向动力学）：
 *   ẋ = A·x + B·u,  x = [cte, he]ᵀ, u = Δωz
 *   A = [[0, v], [0, 0]],  B = [[0], [1]]
 *   Q = diag(Q_cte, Q_he),  R = [R_wz]
 */
class LateralErrorCritic : public CriticFunction
{
public:
    void initialize() override
    {
        // 初始化时根据 Q/R 和标称速度求解 P 矩阵
        solvePMatrix(nominal_speed_);
    }

    /**
     * @param weight         总权重缩放
     * @param Q_cte          CTE 状态权重
     * @param Q_he           HE  状态权重
     * @param R_wz           控制权重
     * @param nominal_speed  标称速度（用于 LQR 增益计算）
     * @param threshold      距目标点小于此值时关闭
     * @param traj_step      轨迹采样步长
     */
    void setParams(float weight, float Q_cte, float Q_he, float R_wz,
                   float nominal_speed, float threshold, int traj_step)
    {
        weight_ = std::max(0.0f, weight);
        Q_cte_ = Q_cte;
        setEnabled(weight_ > 0.0f);
        Q_he_ = Q_he;
        R_wz_ = R_wz;
        nominal_speed_ = nominal_speed;
        threshold_to_consider_ = threshold;
        traj_point_step_ = std::max(1, traj_step);
        solvePMatrix(nominal_speed_);
        last_solve_speed_ = std::max(std::abs(nominal_speed_), 0.1f);
    }

    void setModelDt(float model_dt)
    {
        model_dt_ = std::max(model_dt, 1e-3f);
        solvePMatrix(last_solve_speed_);
    }

    /**
     * @brief 动态更新速度（每帧调用），重新求解 P 矩阵
     */
    void updateSpeed(float current_speed)
    {
        float v = std::max(std::abs(current_speed), 0.1f);
        // 仅速度变化超过 10% 时才重解，减少计算
        if (std::abs(v - last_solve_speed_) / last_solve_speed_ > 0.1f) {
            solvePMatrix(v);
            last_solve_speed_ = v;
        }
    }

    void score(CriticData & data) override
    {
        if (!enabled_ || data.path.size() < 2) return;

        const Pose2D & goal = data.global_goal;
        float dist_to_goal = std::hypot(data.state.pose.x - goal.x,
                                        data.state.pose.y - goal.y);
        if (dist_to_goal < threshold_to_consider_) return;

        // 更新 LQR P 矩阵（基于当前速度）
        updateSpeed(data.state.speed.vx);

        const size_t batch_size = data.trajectories.x.shape(0);
        const size_t traj_len   = data.trajectories.x.shape(1);
        const size_t path_size  = data.path.size();

        // 预计算路径段方向
        // path_yaws 已存储，直接使用

        for (size_t i = 0; i < batch_size; ++i) {
            float cost = 0.0f;
            int count = 0;
            size_t path_pt = 0;   // 路径搜索起点（单调递增）

            for (size_t j = 0; j < traj_len; j += traj_point_step_) {
                float px = data.trajectories.x(i, j);
                float py = data.trajectories.y(i, j);
                float pyaw = data.trajectories.yaws(i, j);

                // ---- 找最近路径点（局部搜索，起点单调递增） ----
                float min_dist = std::numeric_limits<float>::max();
                size_t closest = path_pt;
                // 搜索范围：从上次位置往前搜一段
                size_t search_end = std::min(path_pt + 20u, path_size);
                for (size_t k = path_pt; k < search_end; ++k) {
                    float dx = px - data.path.x(k);
                    float dy = py - data.path.y(k);
                    float d = dx * dx + dy * dy;
                    if (d < min_dist) {
                        min_dist = d;
                        closest = k;
                    }
                }
                path_pt = closest;

                // ---- 计算 CTE 和 HE ----
                float path_yaw = data.path.yaws(closest);
                float dx = px - data.path.x(closest);
                float dy = py - data.path.y(closest);

                // CTE：车体到路径点的横向距离（沿路径法线方向）
                // 负号使左偏为正（符合Frenet惯例）
                float cte = -std::sin(path_yaw) * dx + std::cos(path_yaw) * dy;

                // HE：朝向偏差
                float he = shortestAngularDistance(path_yaw, pyaw);

                // ---- LQR 最优二次代价：[cte, he] * P * [cte; he] ----
                float p_cost = P_[0] * cte * cte
                             + P_[1] * cte * he
                             + P_[2] * he * cte
                             + P_[3] * he * he;

                cost += p_cost;
                count++;
            }

            if (count > 0) {
                data.costs(i) += weight_ * (cost / count);
            }
        }
    }

private:
    /**
     * @brief 求解离散代数Riccati方程（DARE），得到 P 矩阵
     *
     * 连续系统：ẋ = Ac·x + Bc·u
     *   Ac = [[0, v], [0, 0]],  Bc = [[0], [1]]
     *
     * 离散化（ZOH, dt=model_dt_）：
     *   Ad = I + Ac·dt = [[1, v·dt], [0, 1]]
     *   Bd = Bc·dt = [[0], [dt]]
     *
     * DARE 迭代：
     *   P_{k+1} = Adᵀ·Pk·Ad - Adᵀ·Pk·Bd·(R + Bdᵀ·Pk·Bd)⁻¹·Bdᵀ·Pk·Ad + Qd
     *   Qd = Q·dt（连续Q到离散Q的近似映射）
     *
     * 存储为 P_ = {P11, P12, P21, P22}
     */
    void solvePMatrix(float v)
    {
        const float dt = model_dt_;

        // 离散系统矩阵
        // Ad = [[1, v*dt], [0, 1]]
        // Bd = [[0], [dt]]
        float ad11 = 1.0f, ad12 = v * dt;
        float ad21 = 0.0f, ad22 = 1.0f;
        float bd1 = 0.0f, bd2 = dt;

        // 离散 Q, R
        float qd11 = Q_cte_ * dt;
        float qd22 = Q_he_ * dt;
        float rd   = R_wz_ * dt;

        // 初始化 P = Qd
        float p11 = qd11, p12 = 0.0f;
        float p21 = 0.0f,   p22 = qd22;

        // DARE 迭代；低速时收敛较慢，最多迭代 100 次。
        for (int iter = 0; iter < 100; ++iter) {
            // S = Bdᵀ·P·Bd + Rd  (标量)
            float s = bd1 * (bd1 * p11 + bd2 * p12)
                    + bd2 * (bd1 * p21 + bd2 * p22) + rd;
            if (std::abs(s) < EPSILON) break;
            float s_inv = 1.0f / s;

            // temp = P·Bd  (2x1)
            float tb1 = bd1 * p11 + bd2 * p12;
            float tb2 = bd1 * p21 + bd2 * p22;

            // P_new = Adᵀ·P·Ad - K·S·Kᵀ + Qd
            //       = Adᵀ·P·Ad - (Adᵀ·P·Bd)·(Bdᵀ·P·Ad)/S + Qd
            //       = Adᵀ·(P - P·Bd·Bdᵀ·P/S)·Ad + Qd
            float p_new11 = ad11 * (p11 - tb1 * tb1 * s_inv) * ad11
                          + ad21 * (p12 - tb1 * tb2 * s_inv) * ad11
                          + ad11 * (p21 - tb2 * tb1 * s_inv) * ad21
                          + ad21 * (p22 - tb2 * tb2 * s_inv) * ad21
                          + qd11;
            float p_new12 = ad11 * (p11 - tb1 * tb1 * s_inv) * ad12
                          + ad21 * (p12 - tb1 * tb2 * s_inv) * ad12
                          + ad11 * (p21 - tb2 * tb1 * s_inv) * ad22
                          + ad21 * (p22 - tb2 * tb2 * s_inv) * ad22;
            float p_new22 = ad12 * (p11 - tb1 * tb1 * s_inv) * ad12
                          + ad22 * (p12 - tb1 * tb2 * s_inv) * ad12
                          + ad12 * (p21 - tb2 * tb1 * s_inv) * ad22
                          + ad22 * (p22 - tb2 * tb2 * s_inv) * ad22
                          + qd22;

            // 检查收敛
            float diff = std::abs(p_new11 - p11) + std::abs(p_new12 - p12)
                       + std::abs(p_new22 - p22);
            p11 = p_new11;
            p12 = p_new12;
            p21 = p_new12;   // P 对称
            p22 = p_new22;

            if (diff < 1e-6f) break;
        }

        P_[0] = p11;  P_[1] = p12;
        P_[2] = p21;  P_[3] = p22;
    }

    // 代价参数
    float weight_ = 15.0f;
    float Q_cte_ = 5.0f;       // CTE 状态权重
    float Q_he_  = 3.0f;       // HE  状态权重
    float R_wz_  = 1.0f;       // 控制权重
    float nominal_speed_ = 0.8f;   // 标称速度
    float threshold_to_consider_ = 0.5f;
    int   traj_point_step_ = 3;

    // LQR Riccati 解 (2x2 对称矩阵，行优先存储)
    std::array<float, 4> P_ = {{1.0f, 0.0f, 0.0f, 1.0f}};

    float last_solve_speed_ = 0.8f;
    float model_dt_ = 0.05f;
};

} // namespace mppi

#endif // MPPI_CRITICS_LATERAL_ERROR_CRITIC_HPP_
