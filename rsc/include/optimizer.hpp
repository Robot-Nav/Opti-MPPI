// ============================================================================
// 文件：optimizer.hpp
// 功能：MPPI 优化器核心，执行采样、评分、加权更新
// ============================================================================
#ifndef MPPI_OPTIMIZER_HPP_
#define MPPI_OPTIMIZER_HPP_

#include <cmath>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <optional>
#include <array>
#include <memory>
#include <limits>
#include <numeric>
#include <utility>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xnoalias.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xsort.hpp>

#include "models/types.hpp"
#include "models/constraints.hpp"
#include "models/control_sequence.hpp"
#include "models/state.hpp"
#include "models/trajectories.hpp"
#include "models/path.hpp"
#include "models/optimizer_settings.hpp"
#include "motion_models.hpp"
#include "critics/critic_data.hpp"
#include "critics/critic_manager.hpp"
#include "tools/noise_generator.hpp"
#include "tools/math_utils.hpp"
#include "tools/path_projection.hpp"
#include "tools/path_reference.hpp"

namespace mppi
{

/**
 * @brief MPPI 优化器核心，执行采样、评分、加权更新
 */
class Optimizer
{
public:
    /**
     * @brief 初始化优化器
     * @param settings 优化器设置
     * @param motion_model 运动模型
     * @param critic_manager 代价函数管理器
     */
    void initialize(const OptimizerSettings & settings,
                    std::shared_ptr<MotionModel> motion_model,
                    CriticManager * critic_manager)
    {
        settings_ = settings;
        if (settings_.batch_size == 0 || settings_.time_steps < 2) {
            throw std::invalid_argument("MPPI requires batch_size > 0 and time_steps >= 2");
        }
        if (settings_.model_dt <= 0.0f || settings_.sampling_std.vx <= EPSILON ||
            settings_.sampling_std.wz <= EPSILON) {
            throw std::invalid_argument("MPPI requires positive model_dt, vx_std and wz_std");
        }
        settings_.thread_count = std::max(1u, settings_.thread_count);
        settings_.min_traction_scale = clamp(settings_.min_traction_scale, 0.1f, 1.0f);
        settings_.traction_speed_exponent = std::max(0.0f, settings_.traction_speed_exponent);
        settings_.disturbance_sigma_clip = std::max(0.0f, settings_.disturbance_sigma_clip);
        settings_.disturbance_decay_time_constant = std::max(0.05f, settings_.disturbance_decay_time_constant);
        settings_.furthest_reached_quantile = clamp(settings_.furthest_reached_quantile, 0.5f, 1.0f);
        settings_.noise_correlation = clamp(settings_.noise_correlation, 0.0f, 0.99f);
        settings_.control_derivative_noise_scale = clamp(
            settings_.control_derivative_noise_scale, 0.001f, 1.0f);
        settings_.path_corner_preview_distance = std::max(
            0.0f, settings_.path_corner_preview_distance);
        settings_.path_corner_deceleration = std::max(
            0.05f, settings_.path_corner_deceleration);
        settings_.heading_alignment_start = clamp(
            settings_.heading_alignment_start, 0.0f, PI);
        settings_.heading_alignment_stop = clamp(
            settings_.heading_alignment_stop,
            settings_.heading_alignment_start + 0.05f, PI);
        settings_.disturbance_heading_gate_scale = std::max(
            1.0f, settings_.disturbance_heading_gate_scale);
        settings_.corner_alignment_distance = std::max(
            0.0f, settings_.corner_alignment_distance);
        settings_.corner_approach_distance = std::max(
            settings_.corner_alignment_distance,
            settings_.corner_approach_distance);
        settings_.corner_alignment_min_angle = clamp(
            settings_.corner_alignment_min_angle, 0.0f, PI);
        settings_.corner_alignment_yaw_tolerance = clamp(
            settings_.corner_alignment_yaw_tolerance, 0.0f, PI);
        settings_.corner_alignment_gain = std::max(
            0.0f, settings_.corner_alignment_gain);
        settings_.corner_alignment_max_yaw_rate = std::max(
            0.0f, settings_.corner_alignment_max_yaw_rate);
        settings_.nominal_sample_count = std::min(settings_.nominal_sample_count, settings_.batch_size);
        settings_.elite_sample_count = std::min(
            settings_.elite_sample_count, settings_.batch_size);
        settings_.elite_update_rate = clamp(
            settings_.elite_update_rate, 0.05f, 1.0f);
        settings_.adaptive_temperature_min =
            std::max(settings_.adaptive_temperature_min, 1e-3f);
        settings_.adaptive_temperature_max = std::max(
            settings_.adaptive_temperature_max, settings_.adaptive_temperature_min);
        settings_.target_ess_ratio = clamp(settings_.target_ess_ratio, 0.01f, 1.0f);
        settings_.guided_sampling_ratio = clamp(settings_.guided_sampling_ratio, 0.0f, 0.8f);
        settings_.guided_sampling_blend = clamp(settings_.guided_sampling_blend, 0.0f, 1.0f);
        settings_.curvature_consistent_speed = std::max(
            0.0f, settings_.curvature_consistent_speed);
        settings_.ancillary_lateral_gain = std::max(
            0.0f, settings_.ancillary_lateral_gain);
        settings_.ancillary_heading_gain = std::max(
            0.0f, settings_.ancillary_heading_gain);
        settings_.ancillary_max_yaw_rate = std::max(
            0.0f, settings_.ancillary_max_yaw_rate);
        settings_.ancillary_nominal_scale = clamp(
            settings_.ancillary_nominal_scale, 0.0f, 1.0f);
        settings_.ancillary_disturbance_scale = clamp(
            settings_.ancillary_disturbance_scale, 0.0f, 1.0f);
        settings_.ancillary_curvature_activation = std::max(
            0.0f, settings_.ancillary_curvature_activation);
        settings_.traction_speed_exponent = std::max(
            0.0f, settings_.traction_speed_exponent);
        settings_.traction_acceleration_exponent = std::max(
            0.0f, settings_.traction_acceleration_exponent);
        settings_.path_reference_speed = std::max(0.0f, settings_.path_reference_speed);
        settings_.path_max_lateral_acceleration =
            std::max(0.05f, settings_.path_max_lateral_acceleration);
        settings_.goal_deceleration = std::max(0.05f, settings_.goal_deceleration);
        motion_model_ = motion_model;
        critic_manager_ = critic_manager;

        motion_model_->setAccelerationConstraints(
            settings_.base_constraints.ax_max,
            settings_.base_constraints.ay_max,
            settings_.base_constraints.az_max,
            settings_.model_dt
        );

        noise_generator_.initialize(settings_, isHolonomic());
        reset();
    }

    /** @brief 重置优化器内部状态 */
    void reset()
    {
        state_.reset(settings_.batch_size, settings_.time_steps);
        control_sequence_.reset(settings_.time_steps);
        settings_.constraints = settings_.base_constraints;
        costs_ = xt::zeros<float>({settings_.batch_size});
        generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);
        reference_control_sequence_.reset(settings_.time_steps);
        last_control_ = {0.0f, 0.0f, 0.0f};
        control_history_[0] = {0.0f, 0.0f, 0.0f};
        control_history_[1] = {0.0f, 0.0f, 0.0f};
        control_history_[2] = {0.0f, 0.0f, 0.0f};
        control_history_[3] = {0.0f, 0.0f, 0.0f};
        global_goal_ = Pose2D{};
        last_effective_sample_size_ = 0.0f;
        last_min_cost_ = 0.0f;
        last_max_cost_ = 0.0f;
    }

    /**
     * @brief 计算最优控制指令（主入口）
     * @param robot_pose 当前机器人位姿
     * @param robot_speed 当前机器人速度
     * @param path 全局路径
     * @return 最优控制量（Twist2D）
     */
    Twist2D evalControl(const Pose2D & robot_pose, const Twist2D & robot_speed, const Path & path)
    {
        prepare(robot_pose, robot_speed, path);

        float prev_min_cost = std::numeric_limits<float>::max();

        for (size_t iter = 0; iter < settings_.iteration_count; ++iter) {
            optimize();

            float current_min_cost = xt::amin(costs_)(0);
            if (std::abs(prev_min_cost - current_min_cost) < 0.01f && current_min_cost < 100.0f) {
                break;
            }
            prev_min_cost = current_min_cost;

            size_t best_idx = xt::argmin(costs_)(0);
            if (!std::isfinite(costs_(best_idx))) {
                throw std::runtime_error("Optimizer failed to compute finite tracking cost");
            }
        }

        // 应用 Savitzky-Golay 滤波器平滑控制序列
        if (settings_.use_sg_filter) {
            savitskyGolayFilter(control_sequence_, control_history_, settings_);
            // SG 滤波可能产生越界或破坏阿克曼曲率约束，滤波后必须再次约束。
            applyControlSequenceConstraints();
        }

        Twist2D control = getControlFromSequence();

        last_control_ = control;
        last_speed_ = robot_speed;

        updateControlHistory();

        if (settings_.shift_control_sequence) {
            shiftControlSequence();
        }
        return control;
    }

    /** @return 生成的采样轨迹（用于可视化） */
    Trajectories & getGeneratedTrajectories() { return generated_trajectories_; }

    /**
     * @brief 获取最优轨迹（根据更新后的控制序列重新积分）
     * @return 最优轨迹矩阵 (time_steps x 3) [x, y, yaw]
     */
    xt::xtensor<float, 2> getOptimizedTrajectory()
    {
        State nominal_state;
        nominal_state.reset(1u, settings_.time_steps);
        nominal_state.pose = state_.pose;
        nominal_state.speed = state_.speed;
        for (size_t j = 0; j < settings_.time_steps; ++j) {
            nominal_state.cvx(0, j) = control_sequence_.vx(j);
            nominal_state.cwz(0, j) = control_sequence_.wz(j);
            nominal_state.cvy(0, j) = isHolonomic() ? control_sequence_.vy(j) : 0.0f;
        }
        motion_model_->applyConstraints(
            nominal_state.cvx, nominal_state.cvy, nominal_state.cwz);
        const float eta_v = disturbance_.valid ? disturbance_.longitudinal_eta : 1.0f;
        const float eta_w = disturbance_.valid ? disturbance_.yaw_eta : 1.0f;
        motion_model_->predict(nominal_state, eta_v, eta_w);

        Trajectories nominal_trajectories;
        nominal_trajectories.reset(1u, settings_.time_steps);
        integrateStateVelocities(nominal_trajectories, nominal_state);

        xt::xtensor<float, 2> trajectory = xt::zeros<float>(
            std::vector<size_t>{settings_.time_steps, 3u});
        for (size_t j = 0; j < settings_.time_steps; ++j) {
            trajectory(j, 0) = nominal_trajectories.x(0, j);
            trajectory(j, 1) = nominal_trajectories.y(0, j);
            trajectory(j, 2) = nominal_trajectories.yaws(0, j);
        }
        return trajectory;
    }

    /** @return 优化器设置 */
    const OptimizerSettings & getSettings() const { return settings_; }
    float getLastEffectiveSampleSize() const { return last_effective_sample_size_; }
    float getLastMinCost() const { return last_min_cost_; }
    float getLastMaxCost() const { return last_max_cost_; }
    void notifyPathUpdated() { last_closest_path_idx_ = 0; }

    /** Keep the warm start and smoothness state consistent with the command actually sent. */
    void setAppliedControl(const Twist2D & command)
    {
        last_control_ = command;
        if (settings_.time_steps > 0) {
            control_sequence_.vx(0) = command.vx;
            control_sequence_.wz(0) = command.wz;
            if (isHolonomic()) control_sequence_.vy(0) = command.vy;
        }
        control_history_[0] = {command.vx, command.vy, command.wz};
    }

    /**
     * @brief 设置侧向扰动估计值（DC-PMPPI 核心）
     * @param vy_dist 车体系侧向滑移速度估计 (m/s)
     * @param wz_dist 横摆角速度扰动估计 (rad/s)
     *
     * 用于修正 rollout 预测模型，使采样轨迹更接近湿滑/晃动下的真实车辆运动。
     * 当 vy_dist=0 且 wz_dist=0 时，等价于标准 MPPI，便于 A/B 对比实验。
     */
    void setDisturbanceEstimate(float vy_dist, float wz_dist)
    {
        DisturbanceEstimate estimate;
        estimate.lateral_velocity = vy_dist;
        estimate.yaw_rate = wz_dist;
        estimate.valid = true;
        setDisturbanceEstimate(estimate);
    }

    void setDisturbanceEstimate(const DisturbanceEstimate & estimate)
    {
        disturbance_ = estimate;
        disturbance_.lateral_velocity = clamp(disturbance_.lateral_velocity, -1.0f, 1.0f);
        disturbance_.yaw_rate = clamp(disturbance_.yaw_rate, -2.0f, 2.0f);
        disturbance_.lateral_velocity_std = clamp(disturbance_.lateral_velocity_std, 0.0f, 0.5f);
        disturbance_.yaw_rate_std = clamp(disturbance_.yaw_rate_std, 0.0f, 1.0f);
        disturbance_.traction_scale = clamp(disturbance_.traction_scale,
            settings_.min_traction_scale, 1.0f);
        disturbance_.confidence = clamp(disturbance_.confidence, 0.0f, 1.0f);
    }

    /** @return 当前侧向扰动估计值 */
    float getEstimatedLateralVel() const { return disturbance_.lateral_velocity; }
    /** @return 当前横摆扰动估计值 */
    float getEstimatedYawRateDist() const { return disturbance_.yaw_rate; }

private:
    /**
     * @brief 准备本次优化：更新状态中的位姿和速度，存储路径（裁剪后）
     */
    void prepare(const Pose2D & robot_pose, const Twist2D & robot_speed, const Path & path)
    {
        state_.pose = robot_pose;
        state_.speed = robot_speed;
        global_goal_ = path.getGoal();

        // 裁剪路径，使索引0对应机器人当前位置
        // prunePath 内部使用进度索引+航向匹配，返回裁剪后路径
        path_ = prunePath(path, robot_pose, settings_.prune_distance);
        settings_.constraints = settings_.base_constraints;
        const float traction = disturbance_.valid ?
            clamp(disturbance_.traction_scale, settings_.min_traction_scale, 1.0f) : 1.0f;
        const float speed_scale = std::pow(traction, settings_.traction_speed_exponent);
        const float acceleration_scale = std::pow(
            traction, settings_.traction_acceleration_exponent);
        settings_.constraints.vx_max *= speed_scale;
        settings_.constraints.vx_min *= speed_scale;
        settings_.constraints.vy_max *= speed_scale;
        settings_.constraints.wz_max *= speed_scale;
        settings_.constraints.ax_max *= acceleration_scale;
        settings_.constraints.ay_max *= acceleration_scale;
        settings_.constraints.az_max *= acceleration_scale;
        motion_model_->setAccelerationConstraints(
            settings_.constraints.ax_max, settings_.constraints.ay_max,
            settings_.constraints.az_max, settings_.model_dt);
        buildReferenceControlSequence();
        costs_ = xt::zeros<float>({settings_.batch_size});
    }

    /**
     * @brief 单次优化迭代：生成轨迹、评分、更新控制序列
     */
    void optimize()
    {
        generateNoisedTrajectories();
        CriticData data{state_, generated_trajectories_, path_, global_goal_,
                        last_control_, costs_, settings_.model_dt,
                        motion_model_, std::nullopt};
        data.traction_scale = disturbance_.valid ? disturbance_.traction_scale : 1.0f;
        data.disturbance = disturbance_;
        data.disturbance_decay_time_constant = settings_.disturbance_decay_time_constant;

        critic_manager_->evalTrajectoriesScores(data);

        for (size_t i = 0; i < costs_.shape(0); ++i) {
            if (!std::isfinite(costs_(i))) {
                costs_(i) = 10000.0f;
            }
        }

        updateControlSequence();

        // 每次迭代后立即约束控制序列，保证下一轮迭代的起点合法
        applyControlSequenceConstraints();
    }

    /**
     * @brief 查找路径上最远可达点索引
     */
    std::optional<size_t> findFurthestReachedPathPoint()
    {
        if (path_.size() < 2 || generated_trajectories_.x.shape(0) == 0) {
            return std::nullopt;
        }

        const size_t batch_size = generated_trajectories_.x.shape(0);
        const size_t path_size = path_.size();
        std::vector<size_t> reached_indices;
        reached_indices.reserve(batch_size);

        for (size_t i = 0; i < batch_size; ++i) {
            const float terminal_x = generated_trajectories_.x(i, settings_.time_steps - 1);
            const float terminal_y = generated_trajectories_.y(i, settings_.time_steps - 1);
            size_t closest_index = 0;
            float minimum_distance_squared = std::numeric_limits<float>::max();

            for (size_t j = 0; j < path_size; ++j) {
                const float dx = terminal_x - path_.x(j);
                const float dy = terminal_y - path_.y(j);
                const float distance_squared = dx * dx + dy * dy;
                if (distance_squared < minimum_distance_squared) {
                    minimum_distance_squared = distance_squared;
                    closest_index = j;
                }
            }
            reached_indices.push_back(closest_index);
        }

        const size_t quantile_index = std::min(
            reached_indices.size() - 1,
            static_cast<size_t>(std::floor(
                settings_.furthest_reached_quantile *
                static_cast<float>(reached_indices.size() - 1))));
        std::nth_element(reached_indices.begin(),
                         reached_indices.begin() + quantile_index,
                         reached_indices.end());
        return reached_indices[quantile_index];
    }

    /**
     * @brief 裁剪路径：移除机器人当前位置之前的路径点
     */
    Path prunePath(const Path & path, const Pose2D & robot_pose,
                   float prune_distance = 1.0f)
    {
        if (path.empty()) return path;
        if (path.size() == 1) return path;
        if (prune_distance < 0.0f) prune_distance = settings_.prune_distance;

        const size_t path_size = path.size();
        if (last_closest_path_idx_ >= path_size) last_closest_path_idx_ = 0;

        // 搜索窗口使用物理距离而不是固定点数。
        const float backward_distance = 1.0f;
        const float forward_distance = std::max(2.0f * prune_distance, 5.0f);
        size_t begin_index = last_closest_path_idx_;
        float accumulated = 0.0f;
        while (begin_index > 0 && accumulated < backward_distance) {
            accumulated += std::hypot(path.x(begin_index) - path.x(begin_index - 1),
                                      path.y(begin_index) - path.y(begin_index - 1));
            --begin_index;
        }

        size_t end_index = last_closest_path_idx_;
        accumulated = 0.0f;
        while (end_index + 1 < path_size && accumulated < forward_distance) {
            accumulated += std::hypot(path.x(end_index + 1) - path.x(end_index),
                                      path.y(end_index + 1) - path.y(end_index));
            ++end_index;
        }

        auto find_closest = [&](size_t begin, size_t end) {
            size_t best_index = begin;
            float best_cost = std::numeric_limits<float>::max();
            const float heading_weight = 0.10f;
            for (size_t k = begin; k <= end; ++k) {
                const float dx = robot_pose.x - path.x(k);
                const float dy = robot_pose.y - path.y(k);
                const float heading_error = std::abs(
                    shortestAngularDistance(path.yaws(k), robot_pose.theta));
                const float match_cost = dx * dx + dy * dy +
                    heading_weight * heading_error * heading_error;
                if (match_cost < best_cost) {
                    best_cost = match_cost;
                    best_index = k;
                }
            }
            return std::make_pair(best_index, best_cost);
        };

        auto closest = find_closest(begin_index, end_index);
        // 定位跳变、新路径或长时间丢帧时，局部窗口可能找不到当前点，回退全局搜索。
        if (closest.second > 4.0f) {
            closest = find_closest(0, path_size - 1);
        }
        const size_t closest_index = closest.first;
        last_closest_path_idx_ = closest_index;

        // Keep the final segment when the closest point is the goal itself.
        // A one-point local path has no tangent, curvature or longitudinal
        // direction and would disable all path-following costs near the goal.
        const size_t slice_start = closest_index + 1 < path_size ?
            closest_index : closest_index - 1;
        float path_length = 0.0f;
        size_t maximum_index = closest_index;
        for (size_t i = closest_index + 1; i < path_size; ++i) {
            path_length += std::hypot(path.x(i) - path.x(i - 1),
                                      path.y(i) - path.y(i - 1));
            maximum_index = i;
            if (path_length >= prune_distance) break;
        }

        Path pruned_path;
        const size_t new_size = maximum_index - slice_start + 1;
        pruned_path.reset(static_cast<unsigned int>(new_size));
        for (size_t i = 0; i < new_size; ++i) {
            pruned_path.x(i) = path.x(slice_start + i);
            pruned_path.y(i) = path.y(slice_start + i);
            pruned_path.yaws(i) = path.yaws(slice_start + i);
        }
        return pruned_path;
    }

    /** @brief 生成带噪声的轨迹（采样） */
    void generateNoisedTrajectories()
    {
        noise_generator_.setNoisedControls(state_, control_sequence_);
        applyGuidedSampling(state_);
        applyControlConstraintsBatch(state_);
        motion_model_->applyConstraints(state_.cvx, state_.cvy, state_.cwz);
        updateStateVelocities(state_);
        integrateStateVelocities(generated_trajectories_, state_);
        noise_generator_.generateNextNoises();
    }

    /**
     * @brief 对一批含噪声控制量施加边界约束
     */
    void applyControlConstraintsBatch(State & state)
    {
        auto & s = settings_;
        state.cvx = xt::clip(state.cvx, s.constraints.vx_min, s.constraints.vx_max);
        state.cwz = xt::clip(state.cwz, -s.constraints.wz_max, s.constraints.wz_max);
        if (isHolonomic()) {
            state.cvy = xt::clip(state.cvy, -s.constraints.vy_max, s.constraints.vy_max);
        }
    }

    void buildReferenceControlSequence()
    {
        if (path_.size() < 2) return;
        const auto arc_lengths = computePathArcLengths(path_);
        if (arc_lengths.empty() || arc_lengths.back() <= EPSILON) return;
        const PathProjection current = projectPointToPath(
            path_, arc_lengths, state_.pose.x, state_.pose.y);
        if (!current.valid) return;

        const bool contains_goal = localPathContainsGoal(path_, global_goal_);
        const float initial_heading_error = shortestAngularDistance(
            current.yaw, state_.pose.theta);
        const float eta_v = disturbance_.valid ?
            clamp(disturbance_.longitudinal_eta, 0.3f, 1.0f) : 1.0f;
        const float eta_w = disturbance_.valid ?
            clamp(disturbance_.yaw_eta, 0.3f, 1.0f) : 1.0f;
        float progress = current.arc_length;
        for (size_t j = 0; j < settings_.time_steps; ++j) {
            const PathReference reference = samplePathReference(path_, arc_lengths, progress);
            const float speed = previewReferenceSpeed(
                path_, arc_lengths, progress, contains_goal,
                settings_.path_reference_speed,
                settings_.path_max_lateral_acceleration,
                settings_.goal_deceleration,
                settings_.path_corner_preview_distance,
                settings_.path_corner_deceleration);
            const float decay = std::exp(
                -1.5f * static_cast<float>(j) * settings_.model_dt);
            const float disturbance_decay = disturbance_.valid ? std::exp(
                -static_cast<float>(j) * settings_.model_dt /
                settings_.disturbance_decay_time_constant) : 0.0f;
            const float lateral_disturbance =
                disturbance_.lateral_velocity * disturbance_decay;
            const float yaw_disturbance = disturbance_.yaw_rate * disturbance_decay;
            // A persistent lateral velocity requires a small crab-angle offset.
            const float slip_angle = std::atan2(
                lateral_disturbance, std::max(speed, 0.10f));
            const float correction = -decay * (
                settings_.guide_lateral_gain * speed * current.signed_lateral_error +
                settings_.guide_heading_gain * (initial_heading_error + slip_angle));
            reference_control_sequence_.vx(j) = clamp(
                speed / eta_v,
                settings_.constraints.vx_min, settings_.constraints.vx_max);
            reference_control_sequence_.vy(j) = 0.0f;
            reference_control_sequence_.wz(j) = clamp(
                (speed * reference.curvature + correction - yaw_disturbance) / eta_w,
                -settings_.constraints.wz_max, settings_.constraints.wz_max);
            progress = std::min(
                arc_lengths.back(), progress + speed * settings_.model_dt);
        }
    }

    void applyGuidedSampling(State & state) const
    {
        if (settings_.guided_sampling_ratio <= 0.0f ||
            settings_.guided_sampling_blend <= 0.0f) {
            return;
        }
        const size_t first = std::min<size_t>(
            settings_.nominal_sample_count, settings_.batch_size);
        const size_t guided_count = std::min<size_t>(
            settings_.batch_size - first,
            static_cast<size_t>(std::lround(
                settings_.guided_sampling_ratio * settings_.batch_size)));
        const float blend = settings_.guided_sampling_blend;
        for (size_t i = first; i < first + guided_count; ++i) {
            const float sample_blend = i == first ? 1.0f : blend;
            for (size_t j = 0; j < settings_.time_steps; ++j) {
                state.cvx(i, j) = (1.0f - sample_blend) * state.cvx(i, j) +
                    sample_blend * reference_control_sequence_.vx(j);
                state.cwz(i, j) = (1.0f - sample_blend) * state.cwz(i, j) +
                    sample_blend * reference_control_sequence_.wz(j);
            }
        }
    }

    /**
     * @brief 更新状态中的速度序列（由运动模型预测，包含加速度约束）
     */
    void updateStateVelocities(State & state) const
    {
        const float eta_v = disturbance_.valid ? disturbance_.longitudinal_eta : 1.0f;
        const float eta_w = disturbance_.valid ? disturbance_.yaw_eta : 1.0f;
        motion_model_->predict(state, eta_v, eta_w);
    }

    /**
     * @brief 根据速度序列积分得到轨迹（多线程并行）
     */
    void integrateStateVelocities(Trajectories & trajectories, const State & state) const
    {
        const float initial_yaw = state.pose.theta;
        const float pose_x = state.pose.x;
        const float pose_y = state.pose.y;
        const float model_dt = settings_.model_dt;
        const size_t time_steps = state.vx.shape(1);

        // DC-PMPPI: 捕获扰动估计值供所有线程共享（rollout 期间只读）
        const float vy_dist_mean = disturbance_.valid ? disturbance_.lateral_velocity : 0.0f;
        const float wz_dist_mean = disturbance_.valid ? disturbance_.yaw_rate : 0.0f;
        const float vy_dist_std = (settings_.sample_disturbance_uncertainty && disturbance_.valid) ?
            disturbance_.lateral_velocity_std : 0.0f;
        const float wz_dist_std = (settings_.sample_disturbance_uncertainty && disturbance_.valid) ?
            disturbance_.yaw_rate_std : 0.0f;
        std::vector<float> disturbance_decay(time_steps, 1.0f);
        for (size_t j = 1; j < time_steps; ++j) {
            const float prediction_time = static_cast<float>(j - 1) * model_dt;
            disturbance_decay[j] = std::exp(
                -prediction_time / settings_.disturbance_decay_time_constant);
        }

        const int num_threads = static_cast<int>(std::max(1u, settings_.thread_count));
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(num_threads)
#endif
        for (long long sample = 0; sample < static_cast<long long>(settings_.batch_size); ++sample) {
                const size_t i = static_cast<size_t>(sample);
                const float vy_sigma = disturbanceSigmaPoint(i, 0);
                const float wz_sigma = disturbanceSigmaPoint(i, 1);
                const float vy_dist = clamp(
                    vy_dist_mean + vy_sigma * vy_dist_std, -1.0f, 1.0f);
                const float wz_dist = clamp(
                    wz_dist_mean + wz_sigma * wz_dist_std, -2.0f, 2.0f);
                float yaw = initial_yaw;
                trajectories.x(i, 0) = pose_x;
                trajectories.y(i, 0) = pose_y;
                trajectories.yaws(i, 0) = yaw;
                trajectories.times(i, 0) = 0.0f;

                for (size_t j = 1; j < time_steps; ++j) {
                    const float decayed_wz_dist = wz_dist * disturbance_decay[j];
                    const float decayed_vy_dist = vy_dist * disturbance_decay[j];
                    yaw += (state.wz(i, j-1) + decayed_wz_dist) * model_dt;
                    float cos_yaw = std::cos(yaw);
                    float sin_yaw = std::sin(yaw);

                    float vx = state.vx(i, j-1);
                    // DC-PMPPI: 非全向模型使用估计的侧向扰动速度补偿模型失配
                    float vy = isHolonomic() ? state.vy(i, j-1) : decayed_vy_dist;

                    trajectories.x(i, j) = trajectories.x(i, j - 1) +
                        (vx * cos_yaw - vy * sin_yaw) * model_dt;
                    trajectories.y(i, j) = trajectories.y(i, j - 1) +
                        (vx * sin_yaw + vy * cos_yaw) * model_dt;
                    trajectories.yaws(i, j) = yaw;
                    trajectories.times(i, j) = j * model_dt;
                }
        }
    }

    /**
     * @brief 根据代价加权更新控制序列（支持自适应温度和均值归一化）
     */
    void updateControlSequence()
    {
        // 添加控制成本项：惩罚偏离当前控制序列的噪声
        auto bounded_noises_vx = state_.cvx - xt::view(control_sequence_.vx, xt::newaxis(), xt::all());
        auto bounded_noises_wz = state_.cwz - xt::view(control_sequence_.wz, xt::newaxis(), xt::all());

        costs_ += settings_.gamma / (settings_.sampling_std.vx * settings_.sampling_std.vx) *
                  xt::sum(xt::view(control_sequence_.vx, xt::newaxis(), xt::all()) * bounded_noises_vx, 1);
        costs_ += settings_.gamma / (settings_.sampling_std.wz * settings_.sampling_std.wz) *
                  xt::sum(xt::view(control_sequence_.wz, xt::newaxis(), xt::all()) * bounded_noises_wz, 1);

        if (isHolonomic()) {
            auto bounded_noises_vy = state_.cvy - xt::view(control_sequence_.vy, xt::newaxis(), xt::all());
            costs_ += settings_.gamma / (settings_.sampling_std.vy * settings_.sampling_std.vy) *
                      xt::sum(xt::view(control_sequence_.vy, xt::newaxis(), xt::all()) * bounded_noises_vy, 1);
        }

        // 计算softmax权重
        float min_cost = xt::amin(costs_)(0);
        float max_cost = xt::amax(costs_)(0);
        float T = settings_.temperature;

        // ESS-targeted temperature keeps sampling useful when critic scales change.
        if (settings_.adaptive_temperature) {
            T = findTemperatureForTargetEss(min_cost);
        }
        T = std::max(T, 1e-3f);

        const float reference_cost = settings_.use_mean_normalization ?
            xt::mean(costs_)(0) : min_cost;
        auto log_weights = -(costs_ - reference_cost) / T;
        const float max_log_weight = xt::amax(log_weights)(0);
        xt::xtensor<float, 1> weights = xt::exp(log_weights - max_log_weight);

        // CEM-style elite truncation with MPPI weights retained inside the
        // elite set.  This rejects the long tail of weak rollouts without
        // discarding the relative cost information as an equal-weight CEM
        // update would.  K=0 keeps the original all-sample MPPI behavior.
        const size_t elite_count = std::min<size_t>(
            settings_.elite_sample_count, settings_.batch_size);
        if (elite_count > 0 && elite_count < settings_.batch_size) {
            std::vector<size_t> ranked_indices(settings_.batch_size);
            std::iota(ranked_indices.begin(), ranked_indices.end(), 0u);
            std::nth_element(
                ranked_indices.begin(),
                ranked_indices.begin() + elite_count,
                ranked_indices.end(),
                [this](size_t lhs, size_t rhs) {
                    return costs_(lhs) < costs_(rhs);
                });
            std::vector<unsigned char> is_elite(settings_.batch_size, 0u);
            for (size_t rank = 0; rank < elite_count; ++rank) {
                is_elite[ranked_indices[rank]] = 1u;
            }
            for (size_t i = 0; i < settings_.batch_size; ++i) {
                if (!is_elite[i]) {
                    weights(i) = 0.0f;
                }
            }
        }

        float sum_weights = xt::sum(weights)(0);

        if (!std::isfinite(sum_weights) || sum_weights < EPSILON) {
            weights = xt::zeros<float>({settings_.batch_size});
            const size_t fallback_count = elite_count > 0 ?
                elite_count : settings_.batch_size;
            std::vector<size_t> ranked_indices(settings_.batch_size);
            std::iota(ranked_indices.begin(), ranked_indices.end(), 0u);
            std::partial_sort(
                ranked_indices.begin(),
                ranked_indices.begin() + fallback_count,
                ranked_indices.end(),
                [this](size_t lhs, size_t rhs) {
                    return costs_(lhs) < costs_(rhs);
                });
            for (size_t rank = 0; rank < fallback_count; ++rank) {
                weights(ranked_indices[rank]) = 1.0f;
            }
            sum_weights = static_cast<float>(fallback_count);
        }
        weights /= sum_weights;
        float squared_weight_sum = 0.0f;
        for (size_t i = 0; i < settings_.batch_size; ++i) {
            squared_weight_sum += weights(i) * weights(i);
        }
        last_effective_sample_size_ = squared_weight_sum > EPSILON ?
            1.0f / squared_weight_sum : 0.0f;
        last_min_cost_ = min_cost;
        last_max_cost_ = max_cost;

        for (size_t j = 0; j < settings_.time_steps; ++j) {
            float sum_vx = 0.0f, sum_wz = 0.0f, sum_vy = 0.0f;
            for (size_t i = 0; i < settings_.batch_size; ++i) {
                sum_vx += weights(i) * state_.cvx(i, j);
                sum_wz += weights(i) * state_.cwz(i, j);
                if (isHolonomic()) sum_vy += weights(i) * state_.cvy(i, j);
            }
            const float update_rate = elite_count > 0 ?
                settings_.elite_update_rate : 1.0f;
            control_sequence_.vx(j) += update_rate *
                (sum_vx - control_sequence_.vx(j));
            control_sequence_.wz(j) += update_rate *
                (sum_wz - control_sequence_.wz(j));
            if (isHolonomic()) {
                control_sequence_.vy(j) += update_rate *
                    (sum_vy - control_sequence_.vy(j));
            }
        }
    }

    /** @brief 对控制序列施加边界约束 */
    void applyControlSequenceConstraints()
    {
        auto & s = settings_;
        for (size_t j = 0; j < settings_.time_steps; ++j) {
            control_sequence_.vx(j) = clamp(control_sequence_.vx(j), s.constraints.vx_min, s.constraints.vx_max);
            control_sequence_.wz(j) = clamp(control_sequence_.wz(j), -s.constraints.wz_max, s.constraints.wz_max);
            if (isHolonomic()) control_sequence_.vy(j) = clamp(control_sequence_.vy(j), -s.constraints.vy_max, s.constraints.vy_max);
            Control constrained{control_sequence_.vx(j),
                                isHolonomic() ? control_sequence_.vy(j) : 0.0f,
                                control_sequence_.wz(j)};
            motion_model_->applySingleControlConstraints(constrained);
            control_sequence_.vx(j) = constrained.vx;
            control_sequence_.wz(j) = constrained.wz;
            if (isHolonomic()) control_sequence_.vy(j) = constrained.vy;
        }
        // 最后一个控制不参与 T 个状态点的积分，避免其随机值在下一周期被左移到有效区间。
        if (settings_.time_steps > 1) {
            const size_t last = settings_.time_steps - 1;
            control_sequence_.vx(last) = control_sequence_.vx(last - 1);
            control_sequence_.wz(last) = control_sequence_.wz(last - 1);
            if (isHolonomic()) control_sequence_.vy(last) = control_sequence_.vy(last - 1);
        }
    }

    /** @brief 将控制序列左移一位（用于时序平滑） */
    void shiftControlSequence()
    {
        const size_t last_index = settings_.time_steps - 1;
        const float tail_vx = control_sequence_.vx(last_index);
        const float tail_vy = isHolonomic() ? control_sequence_.vy(last_index) : 0.0f;
        const float tail_wz = control_sequence_.wz(last_index);
        for (size_t j = 0; j < last_index; ++j) {
            control_sequence_.vx(j) = control_sequence_.vx(j + 1);
            control_sequence_.wz(j) = control_sequence_.wz(j + 1);
            if (isHolonomic()) control_sequence_.vy(j) = control_sequence_.vy(j + 1);
        }
        control_sequence_.vx(last_index) = tail_vx;
        control_sequence_.wz(last_index) = tail_wz;
        if (isHolonomic()) control_sequence_.vy(last_index) = tail_vy;
    }

    /** @return 当前控制序列的第一个控制量 */
    Twist2D getControlFromSequence()
    {
        const float dt = settings_.model_dt;
        float desired_vx = control_sequence_.vx(0);
        float desired_wz = control_sequence_.wz(0);
        bool requires_heading_alignment = false;
        if (path_.size() >= 2) {
            const auto arc_lengths = computePathArcLengths(path_);
            const PathProjection projection = projectPointToPath(
                path_, arc_lengths, state_.pose.x, state_.pose.y);
            if (projection.valid) {
                float target_yaw = projection.yaw;
                bool corner_zone = false;
                bool corner_approach = false;
                for (size_t segment = projection.segment_index + 1;
                     segment + 1 < path_.size(); ++segment) {
                    const float next_yaw = std::atan2(
                        path_.y(segment + 1) - path_.y(segment),
                        path_.x(segment + 1) - path_.x(segment));
                    if (std::abs(shortestAngularDistance(
                            projection.yaw, next_yaw)) <
                        settings_.corner_alignment_min_angle) {
                        continue;
                    }
                    const float distance_to_corner =
                        arc_lengths[segment] - projection.arc_length;
                    corner_approach = distance_to_corner >= 0.0f &&
                        distance_to_corner <= settings_.corner_approach_distance;
                    if (distance_to_corner >= 0.0f &&
                        distance_to_corner <= settings_.corner_alignment_distance) {
                        target_yaw = next_yaw;
                        corner_zone = true;
                    }
                    break;
                }

                const float slip_angle = disturbance_.valid ? std::atan2(
                    disturbance_.lateral_velocity,
                    std::max(std::abs(desired_vx), 0.10f)) : 0.0f;
                const float desired_body_yaw = target_yaw - slip_angle;
                const float heading_error = shortestAngularDistance(
                    desired_body_yaw, state_.pose.theta);
                float curvature_activity = 1.0f;
                if (disturbance_.valid &&
                    settings_.ancillary_curvature_activation > EPSILON &&
                    projection.segment_index + 2 < path_.size()) {
                    const size_t segment = projection.segment_index;
                    const float first_yaw = std::atan2(
                        path_.y(segment + 1) - path_.y(segment),
                        path_.x(segment + 1) - path_.x(segment));
                    const float second_yaw = std::atan2(
                        path_.y(segment + 2) - path_.y(segment + 1),
                        path_.x(segment + 2) - path_.x(segment + 1));
                    const float segment_length = std::hypot(
                        path_.x(segment + 2) - path_.x(segment + 1),
                        path_.y(segment + 2) - path_.y(segment + 1));
                    const float local_curvature = std::abs(
                        shortestAngularDistance(first_yaw, second_yaw)) /
                        std::max(segment_length, 1e-3f);
                    const float normalized_curvature = clamp(
                        local_curvature /
                            settings_.ancillary_curvature_activation,
                        0.0f, 1.0f);
                    curvature_activity =
                        normalized_curvature * normalized_curvature *
                        (3.0f - 2.0f * normalized_curvature);
                }
                const float feedback_scale = disturbance_.valid ?
                    settings_.ancillary_disturbance_scale *
                        curvature_activity :
                    settings_.ancillary_nominal_scale;
                if (settings_.ancillary_lateral_gain > 0.0f ||
                    settings_.ancillary_heading_gain > 0.0f) {
                    const float correction = feedback_scale * clamp(
                        -settings_.ancillary_lateral_gain * desired_vx *
                            projection.signed_lateral_error -
                        settings_.ancillary_heading_gain * heading_error,
                        -settings_.ancillary_max_yaw_rate,
                        settings_.ancillary_max_yaw_rate);
                    desired_wz += correction;
                }

                // Follow the incoming edge up to a discontinuous corner.  If
                // MPPI starts turning early, the heading gate otherwise turns
                // the approach into a long arc with unnecessary contour error.
                if (corner_approach && !corner_zone) {
                    const float eta_w = disturbance_.valid ?
                        clamp(disturbance_.yaw_eta, 0.3f, 1.0f) : 1.0f;
                    const float yaw_disturbance = disturbance_.valid ?
                        disturbance_.yaw_rate : 0.0f;
                    desired_wz = clamp(
                        (-settings_.guide_lateral_gain * desired_vx *
                            projection.signed_lateral_error -
                         settings_.guide_heading_gain * heading_error -
                         yaw_disturbance) / eta_w,
                        -settings_.corner_alignment_max_yaw_rate,
                        settings_.corner_alignment_max_yaw_rate);
                }

                const float absolute_heading_error = std::abs(heading_error);
                const float heading_gate_scale = disturbance_.valid ?
                    settings_.disturbance_heading_gate_scale : 1.0f;
                const float alignment_start = std::min(
                    PI - 0.10f,
                    settings_.heading_alignment_start * heading_gate_scale);
                const float alignment_stop = std::min(
                    PI, std::max(alignment_start + 0.05f,
                        settings_.heading_alignment_stop * heading_gate_scale));
                requires_heading_alignment =
                    absolute_heading_error > alignment_start;
                const float alignment = clamp(
                    (absolute_heading_error - alignment_start) /
                    std::max(0.05f, alignment_stop - alignment_start),
                    0.0f, 1.0f);
                const float heading_gate = std::pow(
                    std::cos(0.5f * PI * alignment), 2.0f);
                desired_vx *= heading_gate;

                if (corner_zone) {
                    const float eta_w = disturbance_.valid ?
                        clamp(disturbance_.yaw_eta, 0.3f, 1.0f) : 1.0f;
                    const float yaw_disturbance = disturbance_.valid ?
                        disturbance_.yaw_rate : 0.0f;
                    desired_wz = clamp(
                        (-settings_.corner_alignment_gain * heading_error -
                         yaw_disturbance) / eta_w,
                        -settings_.corner_alignment_max_yaw_rate,
                        settings_.corner_alignment_max_yaw_rate);
                    if (absolute_heading_error >
                        settings_.corner_alignment_yaw_tolerance) {
                        desired_vx = 0.0f;
                    }
                }
            }
        }
        if (localPathContainsGoal(path_, global_goal_)) {
            const float distance_to_goal = std::hypot(
                state_.pose.x - global_goal_.x,
                state_.pose.y - global_goal_.y);
            const float braking_speed = std::sqrt(
                2.0f * settings_.goal_deceleration * distance_to_goal);
            desired_vx = smoothMinimum(desired_vx, braking_speed, 0.04f);
        }
        Control control;
        control.vx = clamp(desired_vx,
                           state_.speed.vx - settings_.constraints.ax_max * dt,
                           state_.speed.vx + settings_.constraints.ax_max * dt);
        control.vy = isHolonomic() ? clamp(control_sequence_.vy(0),
                           state_.speed.vy - settings_.constraints.ay_max * dt,
                           state_.speed.vy + settings_.constraints.ay_max * dt) : 0.0f;
        control.wz = clamp(desired_wz,
                           state_.speed.wz - settings_.constraints.az_max * dt,
                           state_.speed.wz + settings_.constraints.az_max * dt);
        control.vx = clamp(control.vx, settings_.constraints.vx_min, settings_.constraints.vx_max);
        control.wz = clamp(control.wz, -settings_.constraints.wz_max, settings_.constraints.wz_max);

        // Independent v/w acceleration limits can create extreme curvature at startup.
        // Preserve the optimized reference curvature only in the low-speed region.
        const float reference_vx = reference_control_sequence_.vx(0);
        if (settings_.curvature_consistent_speed > EPSILON &&
            !requires_heading_alignment &&
            std::abs(reference_vx) > 0.05f &&
            std::abs(control.vx) < settings_.curvature_consistent_speed) {
            const float reference_curvature =
                reference_control_sequence_.wz(0) / reference_vx;
            const float curvature_consistent_wz = reference_curvature * control.vx;
            const float blend = clamp(
                (settings_.curvature_consistent_speed - std::abs(control.vx)) /
                (0.67f * settings_.curvature_consistent_speed), 0.0f, 1.0f);
            control.wz = (1.0f - blend) * control.wz +
                blend * curvature_consistent_wz;
            control.wz = clamp(
                control.wz,
                state_.speed.wz - settings_.constraints.az_max * dt,
                state_.speed.wz + settings_.constraints.az_max * dt);
        }
        motion_model_->applySingleControlConstraints(control);
        control_sequence_.vx(0) = control.vx;
        control_sequence_.wz(0) = control.wz;
        if (isHolonomic()) control_sequence_.vy(0) = control.vy;
        return Twist2D(control.vx, control.vy, control.wz);
    }

    /** @return 是否为全向模型 */
    bool isHolonomic() const { return motion_model_->isHolonomic(); }

    /** @brief 更新控制历史（用于SG滤波） */
    void updateControlHistory()
    {
        control_history_[3] = control_history_[2];
        control_history_[2] = control_history_[1];
        control_history_[1] = control_history_[0];
        control_history_[0] = {control_sequence_.vx(0),
                               isHolonomic() ? control_sequence_.vy(0) : 0.0f,
                               control_sequence_.wz(0)};
    }

    /**
     * @brief Savitzky-Golay滤波器（对整个控制序列滤波）
     */
    void savitskyGolayFilter(ControlSequence & control_sequence,
                             const std::array<Control, 4> & control_history,
                             const OptimizerSettings & settings)
    {
        // 序列太短时跳过滤波，避免过度平滑
        if (settings.time_steps < 10) return;

        const std::vector<float> sg_coeffs = {-0.085714f, 0.342857f, 0.485714f, 0.342857f, -0.085714f};
        const int half_window = 2;

        std::vector<float> vx_temp(control_sequence.vx.begin(), control_sequence.vx.end());
        std::vector<float> vy_temp(control_sequence.vy.begin(), control_sequence.vy.end());
        std::vector<float> wz_temp(control_sequence.wz.begin(), control_sequence.wz.end());

        for (size_t i = 0; i < settings.time_steps; ++i) {
            // vx滤波
            float vx_filtered = 0.0f;
            for (int j = -half_window; j <= half_window; ++j) {
                int idx = static_cast<int>(i) + j;
                float value;
                if (idx < 0) {
                    int hist_idx = -idx - 1;
                    value = (hist_idx < 4) ? control_history[hist_idx].vx : vx_temp[0];
                } else if (idx >= static_cast<int>(settings.time_steps)) {
                    value = vx_temp.back();
                } else {
                    value = vx_temp[idx];
                }
                vx_filtered += sg_coeffs[j + half_window] * value;
            }
            control_sequence.vx(i) = vx_filtered;

            // wz滤波
            float wz_filtered = 0.0f;
            for (int j = -half_window; j <= half_window; ++j) {
                int idx = static_cast<int>(i) + j;
                float value;
                if (idx < 0) {
                    int hist_idx = -idx - 1;
                    value = (hist_idx < 4) ? control_history[hist_idx].wz : wz_temp[0];
                } else if (idx >= static_cast<int>(settings.time_steps)) {
                    value = wz_temp.back();
                } else {
                    value = wz_temp[idx];
                }
                wz_filtered += sg_coeffs[j + half_window] * value;
            }
            control_sequence.wz(i) = wz_filtered;

            // vy滤波（全向模型）
            if (isHolonomic()) {
                float vy_filtered = 0.0f;
                for (int j = -half_window; j <= half_window; ++j) {
                    int idx = static_cast<int>(i) + j;
                    float value;
                    if (idx < 0) {
                        int hist_idx = -idx - 1;
                        value = (hist_idx < 4) ? control_history[hist_idx].vy : vy_temp[0];
                    } else if (idx >= static_cast<int>(settings.time_steps)) {
                        value = vy_temp.back();
                    } else {
                        value = vy_temp[idx];
                    }
                    vy_filtered += sg_coeffs[j + half_window] * value;
                }
                control_sequence.vy(i) = vy_filtered;
            }
        }
    }

private:
    float findTemperatureForTargetEss(float min_cost) const
    {
        const float target_ess = settings_.target_ess_ratio *
            static_cast<float>(settings_.batch_size);
        auto effective_sample_size = [&](float temperature) {
            float sum = 0.0f;
            float squared_sum = 0.0f;
            for (size_t i = 0; i < settings_.batch_size; ++i) {
                const float weight = std::exp(
                    -(costs_(i) - min_cost) / std::max(temperature, 1e-3f));
                sum += weight;
                squared_sum += weight * weight;
            }
            return squared_sum > EPSILON ? sum * sum / squared_sum : 0.0f;
        };

        float low = settings_.adaptive_temperature_min;
        float high = settings_.adaptive_temperature_max;
        if (effective_sample_size(low) >= target_ess) return low;
        if (effective_sample_size(high) <= target_ess) return high;
        for (int iteration = 0; iteration < 20; ++iteration) {
            const float middle = 0.5f * (low + high);
            if (effective_sample_size(middle) < target_ess) low = middle;
            else high = middle;
        }
        return high;
    }

    float disturbanceSigmaPoint(size_t sample_index, size_t dimension) const
    {
        if (!settings_.sample_disturbance_uncertainty ||
            settings_.disturbance_sigma_clip <= EPSILON) return 0.0f;
        if (sample_index < settings_.nominal_sample_count) return 0.0f;
        // 五点等权集合的均值为 0、方差为 1。
        static constexpr float sigma_points[5] =
            {-1.41421356f, -0.70710678f, 0.0f, 0.70710678f, 1.41421356f};
        const size_t adjusted_index = sample_index - settings_.nominal_sample_count;
        const size_t index = dimension == 0 ?
            adjusted_index % 5 : (adjusted_index / 5) % 5;
        return clamp(sigma_points[index],
                     -settings_.disturbance_sigma_clip,
                     settings_.disturbance_sigma_clip);
    }

    OptimizerSettings settings_;
    std::shared_ptr<MotionModel> motion_model_;
    CriticManager * critic_manager_ = nullptr;
    State state_;
    ControlSequence control_sequence_;
    ControlSequence reference_control_sequence_;
    Trajectories generated_trajectories_;
    Path path_;
    Pose2D global_goal_;
    xt::xtensor<float, 1> costs_;
    NoiseGenerator noise_generator_;
    Twist2D last_control_;
    Twist2D last_speed_;
    float last_effective_sample_size_ = 0.0f;
    float last_min_cost_ = 0.0f;
    float last_max_cost_ = 0.0f;
    size_t last_closest_path_idx_ = 0;       // 路径进度索引（防止发卡弯跳变）

    DisturbanceEstimate disturbance_;

    std::array<Control, 4> control_history_;
};

} // namespace mppi

#endif // MPPI_OPTIMIZER_HPP_
