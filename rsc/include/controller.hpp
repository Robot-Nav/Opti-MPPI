// ============================================================================
// 文件：controller.hpp
// 功能：DC-PMPPI 控制器对外接口类，封装优化器和代价函数管理器
//       精简版：仅保留全局路径纯跟踪 + 目标到达 + 扰动补偿
// ============================================================================
#ifndef MPPI_CONTROLLER_HPP_
#define MPPI_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>
#include <vector>
#include <memory>
#include <string>
#include <iostream>
#include <stdexcept>
#include <limits>

#include "models/types.hpp"
#include "models/constraints.hpp"
#include "models/control_sequence.hpp"
#include "models/state.hpp"
#include "models/trajectories.hpp"
#include "models/path.hpp"
#include "models/optimizer_settings.hpp"
#include "motion_models.hpp"
#include "tools/math_utils.hpp"
#include "optimizer.hpp"
#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"
#include "critics/critic_manager.hpp"
#include "critics/mpcc_tracking_critic.hpp"
#include "critics/control_rate_critic.hpp"

namespace mppi
{

/**
 * @brief DC-PMPPI 控制器对外接口类，封装优化器和代价函数管理器
 */
class MPPIController
{
public:
    /**
     * @brief 初始化控制器
     * @param settings 优化器设置
     * @param motion_model_type 运动模型类型："DiffDrive", "Omni", "Ackermann"
     * @param ackermann_min_turning_radius 阿克曼模型的最小转弯半径
     */
    void initialize(const OptimizerSettings & settings,
                    const std::string & motion_model_type = "DiffDrive",
                    float ackermann_min_turning_radius = 0.5f)
    {
        settings_ = settings;

        if (motion_model_type == "DiffDrive") motion_model_ = std::make_shared<DiffDriveMotionModel>();
        else if (motion_model_type == "Omni") motion_model_ = std::make_shared<OmniMotionModel>();
        else if (motion_model_type == "Ackermann") {
            motion_model_ = std::make_shared<AckermannMotionModel>(
                ackermann_min_turning_radius,
                settings.path_max_lateral_acceleration);
        }
        else throw std::runtime_error("Unknown motion model");

        critic_manager_ = std::make_unique<CriticManager>();

        auto mpcc_tracking_critic = std::make_unique<MPCCTrackingCritic>();
        mpcc_tracking_critic->setName("MPCCTrackingCritic");
        mpcc_tracking_critic_ = mpcc_tracking_critic.get();
        critic_manager_->addCritic(std::move(mpcc_tracking_critic));

        auto control_rate_critic = std::make_unique<ControlRateCritic>();
        control_rate_critic->setName("ControlRateCritic");
        control_rate_critic_ = control_rate_critic.get();
        critic_manager_->addCritic(std::move(control_rate_critic));

        critic_manager_->initializeCritics();

        optimizer_ = std::make_unique<Optimizer>();
        optimizer_->initialize(settings_, motion_model_, critic_manager_.get());
    }

    /**
     * @brief 设置全局路径（通过 Pose2D 列表）
     */
    void setPath(const std::vector<Pose2D> & path)
    {
        Path new_path;
        new_path.reset(static_cast<unsigned int>(path.size()));
        for (size_t i = 0; i < path.size(); ++i) {
            new_path.x(i) = path[i].x;
            new_path.y(i) = path[i].y;
            new_path.yaws(i) = path[i].theta;
        }
        setPath(new_path);
    }

    void setPath(const Path & path)
    {
        const bool is_new_path = !pathsEquivalent(path_, path);
        path_ = path;
        if (is_new_path && optimizer_) optimizer_->notifyPathUpdated();
    }

    /**
     * @brief 设置侧向扰动估计值（DC-PMPPI 核心）
     */
    void setDisturbanceEstimate(float vy_dist, float wz_dist) {
        optimizer_->setDisturbanceEstimate(vy_dist, wz_dist);
    }

    void setDisturbanceEstimate(const DisturbanceEstimate & estimate) {
        optimizer_->setDisturbanceEstimate(estimate);
    }

    /** @return 当前侧向扰动估计值 */
    float getEstimatedLateralVel() const { return optimizer_->getEstimatedLateralVel(); }
    /** @return 当前横摆扰动估计值 */
    float getEstimatedYawRateDist() const { return optimizer_->getEstimatedYawRateDist(); }

    /**
     * @brief 计算速度指令（主循环调用）
     */
    Twist2D computeVelocityCommands(const Pose2D & robot_pose, const Twist2D & robot_speed)
    {
        if (path_.empty()) return Twist2D();
        return optimizer_->evalControl(robot_pose, robot_speed, path_);
    }

    // —— Getters ——
    MPCCTrackingCritic* getMPCCTrackingCritic() const { return mpcc_tracking_critic_; }
    ControlRateCritic* getControlRateCritic() const { return control_rate_critic_; }

    /** @return 最近一次评分的各Critic统计（代价贡献+耗时） */
    const std::vector<CriticStatistics>& getCriticStatistics() const {
        return critic_manager_->getStatistics();
    }

    Trajectories & getGeneratedTrajectories() { return optimizer_->getGeneratedTrajectories(); }
    xt::xtensor<float, 2> getOptimizedTrajectory() { return optimizer_->getOptimizedTrajectory(); }
    float getEffectiveSampleSize() const { return optimizer_->getLastEffectiveSampleSize(); }
    float getLastMinCost() const { return optimizer_->getLastMinCost(); }
    float getLastMaxCost() const { return optimizer_->getLastMaxCost(); }
    void setAppliedControl(const Twist2D & command) { optimizer_->setAppliedControl(command); }
    /** @brief 对终端稳定器等优化器后处理命令再次施加车辆运动学约束。 */
    Twist2D enforceMotionConstraints(const Twist2D & command) const {
        Control constrained{command.vx, command.vy, command.wz};
        motion_model_->applySingleControlConstraints(constrained);
        return Twist2D(constrained.vx, constrained.vy, constrained.wz);
    }
    void setCollectCriticStatistics(bool enabled) {
        critic_manager_->setCollectStatistics(enabled);
    }
    void reset() { optimizer_->reset(); }
    bool isHolonomic() const { return motion_model_->isHolonomic(); }
    Path& getPath() { return path_; }

private:
    bool pathsEquivalent(const Path & lhs, const Path & rhs) const
    {
        if (lhs.size() != rhs.size()) return false;
        if (lhs.empty()) return rhs.empty();
        const float position_tolerance = 0.02f;
        const float yaw_tolerance = 0.05f;
        const size_t samples = std::min<size_t>(20, lhs.size());
        for (size_t k = 0; k < samples; ++k) {
            const size_t index = samples == 1 ? 0 :
                k * (lhs.size() - 1) / (samples - 1);
            if (std::hypot(lhs.x(index) - rhs.x(index),
                           lhs.y(index) - rhs.y(index)) > position_tolerance) {
                return false;
            }
            if (std::abs(shortestAngularDistance(
                    lhs.yaws(index), rhs.yaws(index))) > yaw_tolerance) {
                return false;
            }
        }
        return true;
    }

    OptimizerSettings settings_;
    std::shared_ptr<MotionModel> motion_model_;
    std::unique_ptr<CriticManager> critic_manager_;
    std::unique_ptr<Optimizer> optimizer_;
    Path path_;

    // —— Critic 原始指针 ——
    MPCCTrackingCritic* mpcc_tracking_critic_ = nullptr;
    ControlRateCritic* control_rate_critic_ = nullptr;
};

} // namespace mppi

#endif // MPPI_CONTROLLER_HPP_
