#include <gtest/gtest.h>

#include <cmath>
#include <chrono>
#include <iostream>
#include <limits>
#include <numeric>
#include <vector>

#include <xtensor/xrandom.hpp>

#include "controller.hpp"
#include "critics/control_rate_critic.hpp"
#include "tools/disturbance_estimator.hpp"
#include "tools/noise_generator.hpp"
#include "tools/goal_docking.hpp"
#include "tools/path_reference.hpp"

namespace
{

mppi::DisturbanceEstimator makeEstimator()
{
    mppi::DisturbanceEstimatorConfig config;
    config.filter_time_constant = 0.20f;
    config.effectiveness_time_constant = 0.80f;
    config.pose_velocity_window = 0.10f;
    config.lateral_deadzone = 0.005f;
    config.yaw_rate_deadzone = 0.005f;
    config.use_odom_twist_yaw_rate = false;
    config.use_odom_twist_longitudinal_velocity = false;
    mppi::DisturbanceEstimator estimator;
    estimator.setConfig(config);
    return estimator;
}

TEST(DisturbanceEstimator, DryStraightMotionStaysNominal)
{
    auto estimator = makeEstimator();
    mppi::DisturbanceEstimate estimate;
    for (int i = 0; i <= 400; ++i) {
        const float time = 0.01f * static_cast<float>(i);
        estimate = estimator.update(
            {0.60f * time, 0.0f, 0.0f},
            {0.60f, 0.0f, 0.0f}, time,
            {0.60f, 0.0f, 0.0f});
    }
    ASSERT_TRUE(estimate.valid);
    EXPECT_NEAR(estimate.lateral_velocity, 0.0f, 0.005f);
    EXPECT_NEAR(estimate.yaw_rate, 0.0f, 0.005f);
    EXPECT_NEAR(estimate.longitudinal_eta, 1.0f, 0.02f);
}

TEST(DisturbanceEstimator, LearnsPersistentLowControlEffectiveness)
{
    auto estimator = makeEstimator();
    mppi::DisturbanceEstimate estimate;
    for (int i = 0; i <= 500; ++i) {
        const float time = 0.01f * static_cast<float>(i);
        constexpr float actual_speed = 0.42f;
        constexpr float actual_yaw_rate = 0.35f;
        const float yaw = actual_yaw_rate * time;
        const float radius = actual_speed / actual_yaw_rate;
        estimate = estimator.update(
            {radius * std::sin(yaw), radius * (1.0f - std::cos(yaw)), yaw},
            {actual_speed, 0.0f, actual_yaw_rate}, time,
            {0.60f, 0.0f, 0.50f});
    }
    ASSERT_TRUE(estimate.valid);
    EXPECT_NEAR(estimate.longitudinal_eta, 0.70f, 0.03f);
    EXPECT_NEAR(estimate.yaw_eta, 0.70f, 0.03f);
    EXPECT_NEAR(estimate.yaw_rate, 0.0f, 0.03f);
}

TEST(DisturbanceEstimator, CenteredRegressionSeparatesYawEfficiencyAndBias)
{
    mppi::DisturbanceEstimatorConfig config;
    config.filter_time_constant = 0.15f;
    config.effectiveness_time_constant = 0.80f;
    config.pose_velocity_window = 0.10f;
    config.command_yaw_rate_deadzone = 0.10f;
    config.yaw_rate_deadzone = 0.005f;
    config.use_odom_twist_yaw_rate = true;
    config.use_centered_effectiveness_regression = true;
    config.effectiveness_excitation_variance = 0.0025f;
    mppi::DisturbanceEstimator estimator;
    estimator.setConfig(config);

    mppi::DisturbanceEstimate estimate;
    float yaw = 0.0f;
    constexpr float dt = 0.01f;
    constexpr float true_eta = 0.70f;
    constexpr float additive_yaw_rate = 0.12f;
    for (int i = 0; i <= 1200; ++i) {
        const float time = dt * static_cast<float>(i);
        const float command = 0.50f * std::sin(2.0f * mppi::PI * 0.35f * time);
        const float measured_yaw_rate =
            true_eta * command + additive_yaw_rate;
        yaw += measured_yaw_rate * dt;
        estimate = estimator.update(
            {0.0f, 0.0f, yaw},
            {0.0f, 0.0f, measured_yaw_rate}, time,
            {0.0f, 0.0f, command});
    }

    ASSERT_TRUE(estimate.valid);
    EXPECT_NEAR(estimate.yaw_eta, true_eta, 0.04f);
    EXPECT_NEAR(estimate.yaw_rate, additive_yaw_rate, 0.03f);
}

TEST(DisturbanceEstimator, TracksPersistentLateralDisturbance)
{
    auto estimator = makeEstimator();
    mppi::DisturbanceEstimate estimate;
    for (int i = 0; i <= 400; ++i) {
        const float time = 0.01f * static_cast<float>(i);
        estimate = estimator.update(
            {0.50f * time, 0.12f * time, 0.0f},
            {0.50f, 0.12f, 0.0f}, time,
            {0.50f, 0.0f, 0.0f});
    }
    ASSERT_TRUE(estimate.valid);
    EXPECT_NEAR(estimate.lateral_velocity, 0.12f, 0.02f);
    EXPECT_LT(estimate.traction_scale, 0.90f);
}

TEST(DisturbanceEstimator, UsesOdomTwistForGazeboLateralVelocity)
{
    mppi::DisturbanceEstimatorConfig config;
    config.filter_time_constant = 0.10f;
    config.pose_velocity_window = 0.10f;
    config.lateral_deadzone = 0.005f;
    config.use_odom_twist_lateral_velocity = true;
    mppi::DisturbanceEstimator estimator;
    estimator.setConfig(config);

    mppi::DisturbanceEstimate estimate;
    for (int i = 0; i <= 200; ++i) {
        const float time = 0.01f * static_cast<float>(i);
        estimate = estimator.update(
            {0.0f, 0.0f, 0.0f}, {0.0f, 0.18f, 0.0f}, time, {});
    }

    ASSERT_TRUE(estimate.valid);
    EXPECT_NEAR(estimate.lateral_velocity, 0.18f, 0.015f);
}

TEST(DisturbanceEstimator, AccumulatesPersistentSlipAboveDeadzone)
{
    mppi::DisturbanceEstimatorConfig config;
    config.filter_time_constant = 0.30f;
    config.pose_velocity_window = 0.10f;
    config.lateral_deadzone = 0.02f;
    config.use_odom_twist_lateral_velocity = true;
    mppi::DisturbanceEstimator estimator;
    estimator.setConfig(config);

    mppi::DisturbanceEstimate estimate;
    for (int i = 0; i <= 300; ++i) {
        const float time = 0.01f * static_cast<float>(i);
        estimate = estimator.update(
            {0.0f, 0.0f, 0.0f}, {0.0f, 0.04f, 0.0f}, time, {});
    }

    ASSERT_TRUE(estimate.valid);
    EXPECT_NEAR(estimate.lateral_velocity, 0.04f, 0.01f);
}

TEST(DisturbanceEstimator, ConfidenceGatesStartupTransient)
{
    mppi::DisturbanceEstimatorConfig config;
    config.filter_time_constant = 0.20f;
    config.confidence_time_constant = 0.50f;
    config.minimum_confidence = 0.25f;
    config.pose_velocity_window = 0.10f;
    config.use_odom_twist_lateral_velocity = true;
    mppi::DisturbanceEstimator estimator;
    estimator.setConfig(config);

    mppi::DisturbanceEstimate estimate;
    for (int i = 0; i <= 12; ++i) {
        const float time = 0.01f * static_cast<float>(i);
        estimate = estimator.update(
            {0.50f * time, 0.0f, 0.0f},
            {0.50f, 0.0f, 0.0f}, time,
            {0.50f, 0.0f, 0.0f});
    }
    EXPECT_FALSE(estimate.valid);
    EXPECT_LT(estimate.confidence, config.minimum_confidence);

    for (int i = 13; i <= 150; ++i) {
        const float time = 0.01f * static_cast<float>(i);
        estimate = estimator.update(
            {0.50f * time, 0.0f, 0.0f},
            {0.50f, 0.0f, 0.0f}, time,
            {0.50f, 0.0f, 0.0f});
    }
    EXPECT_TRUE(estimate.valid);
    EXPECT_GT(estimate.confidence, 0.80f);
}

TEST(DisturbanceEstimator, ModelsLongitudinalActuatorLag)
{
    mppi::DisturbanceEstimatorConfig config;
    config.filter_time_constant = 0.20f;
    config.effectiveness_time_constant = 0.80f;
    config.longitudinal_actuator_time_constant = 0.10f;
    config.pose_velocity_window = 0.10f;
    config.command_speed_deadzone = 0.20f;
    config.use_odom_twist_longitudinal_velocity = true;
    mppi::DisturbanceEstimator estimator;
    estimator.setConfig(config);

    mppi::DisturbanceEstimate estimate;
    constexpr float command = 0.80f;
    constexpr float actuator_tau = 0.10f;
    for (int i = 0; i <= 250; ++i) {
        const float time = 0.01f * static_cast<float>(i);
        const float actual_speed =
            command * (1.0f - std::exp(-time / actuator_tau));
        estimate = estimator.update(
            {0.0f, 0.0f, 0.0f},
            {actual_speed, 0.0f, 0.0f}, time,
            {command, 0.0f, 0.0f});
    }

    ASSERT_TRUE(estimate.valid);
    EXPECT_GT(estimate.longitudinal_eta, 0.97f);
}

TEST(ControlRateCritic, PenalizesActuatorCommandsInsteadOfRealizedVelocity)
{
    mppi::State state;
    state.reset(1u, 3u);
    state.cwz(0, 1) = 0.50f;
    state.cwz(0, 2) = 1.00f;
    // A low-effectiveness model could realize no/low yaw response here.  The
    // actuator command must still carry a non-zero smoothness cost.
    state.wz(0, 0) = 0.0f;
    state.wz(0, 1) = 0.0f;
    state.wz(0, 2) = 0.0f;

    mppi::Trajectories trajectories;
    trajectories.reset(1u, 3u);
    mppi::Path path;
    path.reset(0u);
    xt::xtensor<float, 1> costs = xt::zeros<float>({1u});
    mppi::CriticData data{
        state, trajectories, path, {}, {}, costs, 1.0f, nullptr,
        std::nullopt};

    mppi::ControlRateCritic critic;
    critic.setParams(1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1);
    critic.score(data);

    EXPECT_NEAR(costs(0), 0.25f, 1e-5f);
}

TEST(NoiseGenerator, ReusesAndShiftsNoiseAcrossControlCycles)
{
    mppi::OptimizerSettings settings;
    settings.batch_size = 4;
    settings.time_steps = 5;
    settings.random_seed = 73;
    settings.sampling_std.vx = 0.20f;
    settings.sampling_std.wz = 0.35f;
    settings.noise_correlation = 0.85f;
    settings.reuse_noise_sequence = true;
    settings.antithetic_sampling = false;
    settings.nominal_sample_count = 0;

    mppi::NoiseGenerator generator;
    generator.initialize(settings, false);
    mppi::ControlSequence controls;
    controls.reset(settings.time_steps);
    mppi::State before;
    mppi::State after;
    before.reset(settings.batch_size, settings.time_steps);
    after.reset(settings.batch_size, settings.time_steps);
    generator.setNoisedControls(before, controls);
    generator.generateNextNoises();
    generator.setNoisedControls(after, controls);

    for (size_t i = 0; i < settings.batch_size; ++i) {
        for (size_t j = 0; j + 1 < settings.time_steps; ++j) {
            EXPECT_FLOAT_EQ(after.cvx(i, j), before.cvx(i, j + 1));
            EXPECT_FLOAT_EQ(after.cwz(i, j), before.cwz(i, j + 1));
        }
    }
}

TEST(NoiseGenerator, KernelSupportSamplingReducesTemporalVariation)
{
    mppi::OptimizerSettings white_settings;
    white_settings.batch_size = 32;
    white_settings.time_steps = 40;
    white_settings.random_seed = 91;
    white_settings.sampling_std.vx = 0.20f;
    white_settings.sampling_std.wz = 0.35f;
    white_settings.noise_correlation = 0.0f;
    white_settings.antithetic_sampling = false;
    white_settings.nominal_sample_count = 0;

    mppi::OptimizerSettings kernel_settings = white_settings;
    kernel_settings.sampling_support_points = 10;
    kernel_settings.sampling_kernel_sigma = 1.0f;

    mppi::ControlSequence controls;
    controls.reset(white_settings.time_steps);
    mppi::State white;
    mppi::State kernel;
    white.reset(white_settings.batch_size, white_settings.time_steps);
    kernel.reset(kernel_settings.batch_size, kernel_settings.time_steps);

    mppi::NoiseGenerator white_generator;
    white_generator.initialize(white_settings, false);
    white_generator.setNoisedControls(white, controls);
    mppi::NoiseGenerator kernel_generator;
    kernel_generator.initialize(kernel_settings, false);
    kernel_generator.setNoisedControls(kernel, controls);

    const auto temporal_variation = [](const mppi::State & state) {
        double sum = 0.0;
        for (size_t i = 0; i < state.cwz.shape()[0]; ++i) {
            for (size_t j = 1; j < state.cwz.shape()[1]; ++j) {
                const double delta = state.cwz(i, j) - state.cwz(i, j - 1);
                sum += delta * delta;
            }
        }
        return sum;
    };

    EXPECT_LT(temporal_variation(kernel), 0.25 * temporal_variation(white));
}

TEST(NoiseGenerator, DerivativeSamplingProducesSmootherControlIncrements)
{
    mppi::OptimizerSettings absolute_settings;
    absolute_settings.batch_size = 32;
    absolute_settings.time_steps = 40;
    absolute_settings.random_seed = 113;
    absolute_settings.sampling_std.vx = 0.20f;
    absolute_settings.sampling_std.wz = 0.35f;
    absolute_settings.noise_correlation = 0.0f;
    absolute_settings.antithetic_sampling = false;
    absolute_settings.nominal_sample_count = 0;

    mppi::OptimizerSettings derivative_settings = absolute_settings;
    derivative_settings.sample_control_derivatives = true;
    derivative_settings.control_derivative_noise_scale = 0.10f;

    mppi::ControlSequence controls;
    controls.reset(absolute_settings.time_steps);
    controls.vx.fill(0.50f);
    controls.wz.fill(0.20f);
    mppi::State absolute;
    mppi::State derivative;
    absolute.reset(absolute_settings.batch_size, absolute_settings.time_steps);
    derivative.reset(derivative_settings.batch_size, derivative_settings.time_steps);

    mppi::NoiseGenerator absolute_generator;
    absolute_generator.initialize(absolute_settings, false);
    absolute_generator.setNoisedControls(absolute, controls);
    mppi::NoiseGenerator derivative_generator;
    derivative_generator.initialize(derivative_settings, false);
    derivative_generator.setNoisedControls(derivative, controls);

    const auto increment_energy = [](const mppi::State & state) {
        double sum = 0.0;
        for (size_t i = 0; i < state.cwz.shape()[0]; ++i) {
            for (size_t j = 1; j < state.cwz.shape()[1]; ++j) {
                const double delta = state.cwz(i, j) - state.cwz(i, j - 1);
                sum += delta * delta;
            }
        }
        return sum;
    };

    EXPECT_LT(increment_energy(derivative),
              0.05 * increment_energy(absolute));
}

mppi::Path makeSemicirclePath(float radius, int segments)
{
    mppi::Path path;
    path.reset(static_cast<unsigned int>(segments + 1));
    for (int i = 0; i <= segments; ++i) {
        const float angle = mppi::PI * static_cast<float>(i) /
            static_cast<float>(segments);
        path.x(i) = radius * std::sin(angle);
        path.y(i) = radius * (1.0f - std::cos(angle));
        path.yaws(i) = angle;
    }
    return path;
}

TEST(PathReference, SemicircleFeedForwardAndGoalSpeedAreConsistent)
{
    const mppi::Path path = makeSemicirclePath(1.0f, 64);
    const auto arc_lengths = mppi::computePathArcLengths(path);
    const mppi::PathReference midpoint = mppi::samplePathReference(
        path, arc_lengths, 0.5f * arc_lengths.back());
    ASSERT_TRUE(midpoint.valid);
    EXPECT_NEAR(midpoint.curvature, 1.0f, 0.05f);

    const float cruise = mppi::referenceSpeed(
        midpoint, 2.0f, false, 0.60f, 0.60f, 0.50f);
    const float approach = mppi::referenceSpeed(
        midpoint, 0.20f, true, 0.60f, 0.60f, 0.50f);
    const float at_goal = mppi::referenceSpeed(
        midpoint, 0.0f, true, 0.60f, 0.60f, 0.50f);
    EXPECT_NEAR(cruise, 0.60f, 0.03f);
    EXPECT_LT(approach, cruise);
    EXPECT_GT(approach, 0.0f);
    EXPECT_NEAR(at_goal, 0.0f, 1e-5f);
    EXPECT_NEAR(cruise * midpoint.curvature, 0.60f, 0.05f);
}

TEST(MPPIOptimizer, TracksSemicircleWithBoundedContourError)
{
    xt::random::seed(42);
    mppi::OptimizerSettings settings;
    settings.batch_size = 400;
    settings.time_steps = 40;
    settings.thread_count = 4;
    settings.model_dt = 0.05f;
    settings.adaptive_temperature = true;
    settings.target_ess_ratio = 0.15f;
    settings.base_constraints.vx_min = 0.0f;
    settings.base_constraints.vx_max = 0.8f;
    settings.base_constraints.vy_max = 0.0f;
    settings.base_constraints.wz_max = 2.0f;
    settings.base_constraints.ax_max = 1.2f;
    settings.base_constraints.az_max = 2.5f;
    settings.constraints = settings.base_constraints;
    settings.sampling_std.vx = 0.20f;
    settings.sampling_std.vy = 0.0f;
    settings.sampling_std.wz = 0.35f;

    mppi::MPPIController controller;
    controller.initialize(settings, "DiffDrive");
    const mppi::Path path = makeSemicirclePath(1.0f, 64);
    controller.setPath(path);

    mppi::Pose2D pose(0.0f, 0.0f, 0.0f);
    mppi::Twist2D speed;
    mppi::GoalDockingConfig docking_config;
    const mppi::Pose2D goal = path.getGoal();
    float squared_contour_error = 0.0f;
    float max_contour_error = 0.0f;
    float minimum_goal_error = std::numeric_limits<float>::max();
    mppi::Twist2D last_command;
    int samples = 0;
    int settled_cycles = 0;
    for (int step = 0; step < 220; ++step) {
        const mppi::Twist2D nominal = controller.computeVelocityCommands(pose, speed);
        const mppi::Twist2D command = mppi::applyGoalDocking(
            nominal, pose, speed, goal, docking_config, settings.model_dt);
        last_command = command;
        speed = command;
        pose.theta = mppi::normalizeAngle(pose.theta + command.wz * settings.model_dt);
        pose.x += command.vx * std::cos(pose.theta) * settings.model_dt;
        pose.y += command.vx * std::sin(pose.theta) * settings.model_dt;
        controller.setAppliedControl(command);

        const float contour_error = std::abs(
            std::hypot(pose.x, pose.y - 1.0f) - 1.0f);
        squared_contour_error += contour_error * contour_error;
        max_contour_error = std::max(max_contour_error, contour_error);
        minimum_goal_error = std::min(
            minimum_goal_error, std::hypot(pose.x, pose.y - 2.0f));
        ++samples;
        const float yaw_error = std::abs(
            mppi::shortestAngularDistance(pose.theta, goal.theta));
        if (std::hypot(pose.x, pose.y - 2.0f) <= docking_config.position_tolerance &&
            yaw_error <= 0.10f && command.vx <= 0.04f &&
            std::abs(command.wz) <= 0.08f) {
            ++settled_cycles;
        } else {
            settled_cycles = 0;
        }
        if (settled_cycles >= 4) break;
    }
    const float rmse = std::sqrt(squared_contour_error / static_cast<float>(samples));
    const float goal_error = std::hypot(pose.x, pose.y - 2.0f);
    std::cout << "[ tracking ] semicircle RMSE=" << rmse
              << "m, max=" << max_contour_error
              << "m, goal=" << goal_error
              << "m, min_goal=" << minimum_goal_error
              << "m, pose=(" << pose.x << "," << pose.y << "," << pose.theta
              << "), cmd=(" << last_command.vx << "," << last_command.wz << ")"
              << std::endl;
    EXPECT_LT(rmse, 0.12f);
    EXPECT_LT(max_contour_error, 0.25f);
    EXPECT_LT(goal_error, 0.03f);
    EXPECT_GE(settled_cycles, 4);
}

TEST(GoalDocking, RecoversWhenNominalPlanStopsAwayFromGoal)
{
    mppi::GoalDockingConfig config;
    const mppi::Pose2D pose(0.0f, 0.0f, 0.0f);
    const mppi::Pose2D goal(0.8f, 0.0f, 0.0f);
    mppi::Twist2D nominal;
    nominal.vx = 0.01f;
    mppi::Twist2D speed;

    const mppi::Twist2D command = mppi::applyGoalDocking(
        nominal, pose, speed, goal, config, 0.05f);

    EXPECT_GT(command.vx, nominal.vx);
    EXPECT_LE(command.vx, config.linear_acceleration * 0.05f + 1e-6f);
    EXPECT_NEAR(command.wz, 0.0f, 1e-6f);

    const mppi::Twist2D near_command = mppi::applyGoalDocking(
        nominal, pose, speed, {0.18f, 0.0f, 0.0f}, config, 0.05f);
    EXPECT_NEAR(near_command.vx, config.linear_acceleration * 0.05f, 1e-6f);
}

TEST(GoalDocking, CompensatesLateralAndYawDisturbance)
{
    mppi::GoalDockingConfig config;
    const mppi::Pose2D pose(0.0f, 0.0f, 0.0f);
    const mppi::Pose2D goal(0.5f, 0.0f, 0.0f);
    mppi::Twist2D nominal;
    nominal.vx = 0.01f;
    mppi::Twist2D speed;
    mppi::DisturbanceEstimate disturbance;
    disturbance.lateral_velocity = 0.10f;
    disturbance.yaw_rate = 0.05f;
    disturbance.longitudinal_eta = 0.8f;
    disturbance.yaw_eta = 0.7f;
    disturbance.valid = true;

    const mppi::Twist2D command = mppi::applyGoalDocking(
        nominal, pose, speed, goal, config, 0.05f, disturbance);

    EXPECT_GT(command.vx, nominal.vx);
    EXPECT_LT(command.wz, -0.01f);
}

TEST(GoalDocking, CouplesFinalOrientationDuringPositionApproach)
{
    mppi::GoalDockingConfig config;
    mppi::Twist2D nominal;
    nominal.vx = 0.01f;

    const mppi::Twist2D command = mppi::applyGoalDocking(
        nominal, {0.0f, 0.0f, 0.0f}, {}, {0.5f, 0.0f, 0.5f * mppi::PI},
        config, 0.05f);

    EXPECT_GT(command.vx, nominal.vx);
    EXPECT_LT(command.wz, 0.0f);
}

struct DisturbedTrackingResult
{
    float rmse = 0.0f;
    float max_error = 0.0f;
};

DisturbedTrackingResult runPersistentSlipTracking(bool compensate)
{
    xt::random::seed(73);
    mppi::OptimizerSettings settings;
    settings.batch_size = 300;
    settings.time_steps = 36;
    settings.thread_count = 4;
    settings.model_dt = 0.05f;
    settings.adaptive_temperature = true;
    settings.target_ess_ratio = 0.15f;
    settings.guided_sampling_ratio = 0.30f;
    settings.disturbance_decay_time_constant = 2.0f;
    // Isolate the learned disturbance model from the nominal ancillary feedback.
    settings.ancillary_lateral_gain = 0.0f;
    settings.base_constraints.vx_min = 0.0f;
    settings.base_constraints.vx_max = 0.8f;
    settings.base_constraints.vy_max = 0.0f;
    settings.base_constraints.wz_max = 2.0f;
    settings.base_constraints.ax_max = 1.2f;
    settings.base_constraints.az_max = 2.5f;
    settings.constraints = settings.base_constraints;
    settings.sampling_std.vx = 0.20f;
    settings.sampling_std.vy = 0.0f;
    settings.sampling_std.wz = 0.35f;

    mppi::MPPIController controller;
    controller.initialize(settings, "DiffDrive");
    controller.setPath(makeSemicirclePath(1.0f, 64));

    constexpr float longitudinal_eta = 0.85f;
    constexpr float yaw_eta = 0.70f;
    constexpr float lateral_disturbance = 0.055f;
    constexpr float yaw_disturbance = 0.035f;
    mppi::DisturbanceEstimate estimate;
    estimate.lateral_velocity = lateral_disturbance;
    estimate.yaw_rate = yaw_disturbance;
    estimate.longitudinal_eta = longitudinal_eta;
    estimate.yaw_eta = yaw_eta;
    estimate.traction_scale = 1.0f;
    estimate.valid = true;

    mppi::Pose2D pose;
    mppi::Twist2D measured_speed;
    float squared_error = 0.0f;
    float max_error = 0.0f;
    constexpr int steps = 90;
    for (int step = 0; step < steps; ++step) {
        controller.setDisturbanceEstimate(
            compensate ? estimate : mppi::DisturbanceEstimate{});
        const mppi::Twist2D command =
            controller.computeVelocityCommands(pose, measured_speed);

        measured_speed.vx = longitudinal_eta * command.vx;
        measured_speed.vy = lateral_disturbance;
        measured_speed.wz = yaw_eta * command.wz + yaw_disturbance;
        pose.theta = mppi::normalizeAngle(
            pose.theta + measured_speed.wz * settings.model_dt);
        pose.x += (measured_speed.vx * std::cos(pose.theta) -
                   measured_speed.vy * std::sin(pose.theta)) * settings.model_dt;
        pose.y += (measured_speed.vx * std::sin(pose.theta) +
                   measured_speed.vy * std::cos(pose.theta)) * settings.model_dt;
        controller.setAppliedControl(command);

        const float contour_error = std::abs(
            std::hypot(pose.x, pose.y - 1.0f) - 1.0f);
        squared_error += contour_error * contour_error;
        max_error = std::max(max_error, contour_error);
    }
    return {
        std::sqrt(squared_error / static_cast<float>(steps)),
        max_error};
}

TEST(MPPIOptimizer, DisturbanceCompensationReducesPersistentSlipError)
{
    const DisturbedTrackingResult standard = runPersistentSlipTracking(false);
    const DisturbedTrackingResult compensated = runPersistentSlipTracking(true);
    std::cout << "[ disturbance A/B ] standard RMSE=" << standard.rmse
              << "m, DC RMSE=" << compensated.rmse
              << "m, standard max=" << standard.max_error
              << "m, DC max=" << compensated.max_error << "m" << std::endl;
    EXPECT_LT(compensated.rmse, 0.90f * standard.rmse);
    EXPECT_LT(compensated.max_error, standard.max_error);
}

TEST(MPPIOptimizer, PureTrackingSmokeAndBenchmark)
{
    mppi::OptimizerSettings settings;
    settings.batch_size = 400;
    settings.time_steps = 40;
    settings.thread_count = 4;
    settings.model_dt = 0.05f;
    settings.adaptive_temperature = true;
    settings.target_ess_ratio = 0.15f;
    settings.antithetic_sampling = true;
    settings.base_constraints.vx_min = 0.0f;
    settings.base_constraints.vx_max = 0.8f;
    settings.base_constraints.vy_max = 0.0f;
    settings.base_constraints.wz_max = 2.0f;
    settings.constraints = settings.base_constraints;
    settings.sampling_std.vx = 0.20f;
    settings.sampling_std.vy = 0.0f;
    settings.sampling_std.wz = 0.35f;

    mppi::MPPIController controller;
    controller.initialize(settings, "DiffDrive");
    std::vector<mppi::Pose2D> path;
    for (int i = 0; i <= 50; ++i) {
        path.emplace_back(0.10f * static_cast<float>(i), 0.0f, 0.0f);
    }
    controller.setPath(path);

    mppi::Pose2D pose;
    mppi::Twist2D speed;
    constexpr int iterations = 30;
    std::vector<double> solve_times;
    solve_times.reserve(iterations);
    for (int i = 0; i < iterations; ++i) {
        const auto start = std::chrono::steady_clock::now();
        const mppi::Twist2D command = controller.computeVelocityCommands(pose, speed);
        const auto finish = std::chrono::steady_clock::now();
        solve_times.push_back(
            std::chrono::duration<double, std::milli>(finish - start).count());
        ASSERT_TRUE(std::isfinite(command.vx));
        ASSERT_TRUE(std::isfinite(command.wz));
        EXPECT_GE(command.vx, settings.base_constraints.vx_min - 1e-5f);
        EXPECT_LE(command.vx, settings.base_constraints.vx_max + 1e-5f);
        EXPECT_LE(std::abs(command.wz), settings.base_constraints.wz_max + 1e-5f);
        speed = command;
        pose.x += command.vx * settings.model_dt;
        pose.theta += command.wz * settings.model_dt;
    }
    std::sort(solve_times.begin(), solve_times.end());
    const double average_ms = std::accumulate(
        solve_times.begin(), solve_times.end(), 0.0) /
        static_cast<double>(solve_times.size());
    const size_t p95_index = static_cast<size_t>(
        std::ceil(0.95 * static_cast<double>(solve_times.size()))) - 1;
    const double p95_ms = solve_times[p95_index];
    const double maximum_ms = solve_times.back();
    std::cout << "[ benchmark ] MPCC-MPPI 400x40 solve: average="
              << average_ms << "ms, p95=" << p95_ms
              << "ms, max=" << maximum_ms << "ms" << std::endl;
    EXPECT_LT(p95_ms, 50.0);
    EXPECT_LT(maximum_ms, 50.0);
}

}  // namespace

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
