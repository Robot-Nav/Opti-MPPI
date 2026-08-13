#ifndef MPPI_TOOLS_NOISE_GENERATOR_HPP_
#define MPPI_TOOLS_NOISE_GENERATOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>

#include <xtensor/xtensor.hpp>
#include <xtensor/xnoalias.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xview.hpp>

#include "models/control_sequence.hpp"
#include "models/optimizer_settings.hpp"
#include "models/state.hpp"

namespace mppi
{

/**
 * @brief MPPI 高斯噪声生成器。
 *
 * 原异步版本存在读写同一噪声矩阵的数据竞争：后台线程在 ready_=false 后
 * 才真正写矩阵，而控制线程将 !ready_ 当成“生成完成”。此版本使用同步生成
 * 与互斥保护，优先保证控制确定性。若后续需要异步优化，应使用双缓冲交换，
 * 不要让生成线程和控制线程同时访问同一矩阵。
 */
class NoiseGenerator
{
public:
    NoiseGenerator() = default;
    ~NoiseGenerator() = default;

    void initialize(const OptimizerSettings & settings, bool is_holonomic)
    {
        std::lock_guard<std::mutex> lock(noise_mutex_);
        settings_ = settings;
        is_holonomic_ = is_holonomic;
        xt::random::seed(settings_.random_seed);
        allocateNoises();
        generateNoisedControlsUnlocked();
    }

    void shutdown() {}

    void generateNextNoises()
    {
        std::lock_guard<std::mutex> lock(noise_mutex_);
        if (settings_.reuse_noise_sequence && settings_.time_steps > 1) {
            if (settings_.sampling_support_points >= 2 &&
                settings_.sampling_support_points < settings_.time_steps) {
                advanceKernelNoisesUnlocked();
            } else {
                advanceNoisesUnlocked();
            }
        } else {
            generateNoisedControlsUnlocked();
        }
    }

    void setNoisedControls(State & state,
                           const ControlSequence & control_sequence)
    {
        std::lock_guard<std::mutex> lock(noise_mutex_);
        if (settings_.sample_control_derivatives) {
            setDerivativeNoisedControlsUnlocked(state, control_sequence);
            return;
        }
        xt::noalias(state.cvx) =
            xt::view(control_sequence.vx, xt::newaxis(), xt::all()) + noises_vx_;
        xt::noalias(state.cwz) =
            xt::view(control_sequence.wz, xt::newaxis(), xt::all()) + noises_wz_;
        if (is_holonomic_) {
            xt::noalias(state.cvy) =
                xt::view(control_sequence.vy, xt::newaxis(), xt::all()) + noises_vy_;
        } else {
            xt::noalias(state.cvy) = xt::zeros<float>(
                {settings_.batch_size, settings_.time_steps});
        }
    }

    void reset(const OptimizerSettings & settings, bool is_holonomic)
    {
        initialize(settings, is_holonomic);
    }

    void setHolonomic(bool is_holonomic)
    {
        std::lock_guard<std::mutex> lock(noise_mutex_);
        is_holonomic_ = is_holonomic;
    }

private:
    void setDerivativeNoisedControlsUnlocked(
        State & state, const ControlSequence & control_sequence) const
    {
        const size_t batch_size = settings_.batch_size;
        const size_t time_steps = settings_.time_steps;
        const float scale = std::max(
            0.001f, std::min(1.0f,
                            settings_.control_derivative_noise_scale));

#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(settings_.thread_count)
#endif
        for (long long sample = 0;
             sample < static_cast<long long>(batch_size); ++sample) {
            const size_t i = static_cast<size_t>(sample);
            state.cvx(i, 0) =
                control_sequence.vx(0) + scale * noises_vx_(i, 0);
            state.cwz(i, 0) =
                control_sequence.wz(0) + scale * noises_wz_(i, 0);
            state.cvy(i, 0) = is_holonomic_ ?
                control_sequence.vy(0) + scale * noises_vy_(i, 0) : 0.0f;

            for (size_t j = 1; j < time_steps; ++j) {
                // Preserve the nominal sequence's increment and perturb that
                // increment.  The sampled absolute control is therefore the
                // integral of a smooth variation sequence.
                state.cvx(i, j) = state.cvx(i, j - 1) +
                    (control_sequence.vx(j) - control_sequence.vx(j - 1)) +
                    scale * noises_vx_(i, j);
                state.cwz(i, j) = state.cwz(i, j - 1) +
                    (control_sequence.wz(j) - control_sequence.wz(j - 1)) +
                    scale * noises_wz_(i, j);
                state.cvy(i, j) = is_holonomic_ ?
                    state.cvy(i, j - 1) +
                    (control_sequence.vy(j) - control_sequence.vy(j - 1)) +
                    scale * noises_vy_(i, j) : 0.0f;
            }
        }
    }

    void advanceKernelNoisesUnlocked()
    {
        const auto & settings = settings_;
        xt::xtensor<float, 2> shifted_vx = noises_vx_;
        xt::xtensor<float, 2> shifted_wz = noises_wz_;
        xt::xtensor<float, 2> shifted_vy = noises_vy_;
        const size_t last = settings.time_steps - 1;
        for (size_t i = 0; i < settings.batch_size; ++i) {
            for (size_t j = 0; j < last; ++j) {
                shifted_vx(i, j) = noises_vx_(i, j + 1);
                shifted_wz(i, j) = noises_wz_(i, j + 1);
                if (is_holonomic_) {
                    shifted_vy(i, j) = noises_vy_(i, j + 1);
                }
            }
            // Zero-order continuation is used only as the prior at the new
            // horizon tail; the fresh kernel sample supplies exploration.
            shifted_vx(i, last) = noises_vx_(i, last);
            shifted_wz(i, last) = noises_wz_(i, last);
            if (is_holonomic_) {
                shifted_vy(i, last) = noises_vy_(i, last);
            }
        }

        generateNoisedControlsUnlocked();
        const float cycle_correlation = std::max(
            0.0f, std::min(0.999f, settings.noise_cycle_correlation));
        const float innovation_scale = std::sqrt(
            std::max(0.0f, 1.0f -
                     cycle_correlation * cycle_correlation));
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(settings.thread_count)
#endif
        for (long long sample = 0;
             sample < static_cast<long long>(settings.batch_size); ++sample) {
            const size_t i = static_cast<size_t>(sample);
            for (size_t j = 0; j < settings.time_steps; ++j) {
                noises_vx_(i, j) =
                    cycle_correlation * shifted_vx(i, j) +
                    innovation_scale * noises_vx_(i, j);
                noises_wz_(i, j) =
                    cycle_correlation * shifted_wz(i, j) +
                    innovation_scale * noises_wz_(i, j);
                if (is_holonomic_) {
                    noises_vy_(i, j) =
                        cycle_correlation * shifted_vy(i, j) +
                        innovation_scale * noises_vy_(i, j);
                }
            }
        }
        applyAntitheticAndNominalConstraintsUnlocked();
    }

    void advanceNoisesUnlocked()
    {
        const auto & settings = settings_;
        const size_t last = settings.time_steps - 1;
        for (size_t i = 0; i < settings.batch_size; ++i) {
            for (size_t j = 0; j < last; ++j) {
                noises_vx_(i, j) = noises_vx_(i, j + 1);
                noises_wz_(i, j) = noises_wz_(i, j + 1);
                if (is_holonomic_) {
                    noises_vy_(i, j) = noises_vy_(i, j + 1);
                }
            }
        }

        const auto tail_vx = xt::random::randn<float>(
            {settings.batch_size}, 0.0f, settings.sampling_std.vx);
        const auto tail_wz = xt::random::randn<float>(
            {settings.batch_size}, 0.0f, settings.sampling_std.wz);
        xt::xtensor<float, 1> tail_vy = xt::zeros<float>(
            {settings.batch_size});
        if (is_holonomic_) {
            tail_vy = xt::random::randn<float>(
                {settings.batch_size}, 0.0f, settings.sampling_std.vy);
        }

        const float beta = std::max(
            0.0f, std::min(0.99f, settings.noise_correlation));
        const float innovation_scale = std::sqrt(
            std::max(0.0f, 1.0f - beta * beta));
        for (size_t i = 0; i < settings.batch_size; ++i) {
            noises_vx_(i, last) =
                beta * noises_vx_(i, last - 1) + innovation_scale * tail_vx(i);
            noises_wz_(i, last) =
                beta * noises_wz_(i, last - 1) + innovation_scale * tail_wz(i);
            if (is_holonomic_) {
                noises_vy_(i, last) =
                    beta * noises_vy_(i, last - 1) + innovation_scale * tail_vy(i);
            }
        }

        applyAntitheticAndNominalConstraintsUnlocked();
    }

    void allocateNoises()
    {
        noises_vx_ = xt::zeros<float>(
            {settings_.batch_size, settings_.time_steps});
        noises_vy_ = xt::zeros<float>(
            {settings_.batch_size, settings_.time_steps});
        noises_wz_ = xt::zeros<float>(
            {settings_.batch_size, settings_.time_steps});
    }

    void generateNoisedControlsUnlocked()
    {
        const auto & settings = settings_;
        xt::noalias(noises_vx_) = xt::random::randn<float>(
            {settings.batch_size, settings.time_steps},
            0.0f, settings.sampling_std.vx);
        xt::noalias(noises_wz_) = xt::random::randn<float>(
            {settings.batch_size, settings.time_steps},
            0.0f, settings.sampling_std.wz);
        if (is_holonomic_) {
            xt::noalias(noises_vy_) = xt::random::randn<float>(
                {settings.batch_size, settings.time_steps},
                0.0f, settings.sampling_std.vy);
        } else {
            noises_vy_.fill(0.0f);
        }

        const float beta = std::max(
            0.0f, std::min(0.99f, settings.noise_correlation));
        const float innovation_scale = std::sqrt(
            std::max(0.0f, 1.0f - beta * beta));
        if (beta > 0.0f && settings.time_steps > 1) {
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(settings.thread_count)
#endif
            for (long long sample = 0;
                 sample < static_cast<long long>(settings.batch_size); ++sample) {
                const size_t i = static_cast<size_t>(sample);
                for (size_t j = 1; j < settings.time_steps; ++j) {
                    noises_vx_(i, j) = beta * noises_vx_(i, j - 1) +
                        innovation_scale * noises_vx_(i, j);
                    noises_wz_(i, j) = beta * noises_wz_(i, j - 1) +
                        innovation_scale * noises_wz_(i, j);
                    if (is_holonomic_) {
                        noises_vy_(i, j) = beta * noises_vy_(i, j - 1) +
                            innovation_scale * noises_vy_(i, j);
                    }
                }
            }
        }

        applyKernelInterpolationUnlocked();
        applyAntitheticAndNominalConstraintsUnlocked();
    }

    void applyKernelInterpolationUnlocked()
    {
        const auto & settings = settings_;
        const size_t support_count = std::min<size_t>(
            settings.sampling_support_points, settings.time_steps);
        if (support_count < 2 || support_count >= settings.time_steps) {
            return;
        }

        const float sigma = std::max(0.10f, settings.sampling_kernel_sigma);
        const float denominator = static_cast<float>(settings.time_steps - 1);
        const std::array<size_t, 2> support_shape{
            static_cast<size_t>(settings.batch_size), support_count};
        xt::xtensor<float, 2> support_vx = xt::zeros<float>(
            support_shape);
        xt::xtensor<float, 2> support_wz = xt::zeros<float>(
            support_shape);
        xt::xtensor<float, 2> support_vy = xt::zeros<float>(
            support_shape);

        for (size_t k = 0; k < support_count; ++k) {
            const float position = static_cast<float>(k) * denominator /
                static_cast<float>(support_count - 1);
            const size_t index = std::min<size_t>(
                settings.time_steps - 1,
                static_cast<size_t>(std::lround(position)));
            for (size_t i = 0; i < settings.batch_size; ++i) {
                support_vx(i, k) = noises_vx_(i, index);
                support_wz(i, k) = noises_wz_(i, index);
                if (is_holonomic_) {
                    support_vy(i, k) = noises_vy_(i, index);
                }
            }
        }

        // KMPPI-style RBF interpolation: optimize a lower-dimensional set of
        // temporal support controls, then expand them over the full horizon.
        // Dividing by sqrt(sum(w^2)) preserves the requested pointwise noise
        // variance when the support variables are independent.
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(settings.thread_count)
#endif
        for (long long sample = 0;
             sample < static_cast<long long>(settings.batch_size); ++sample) {
            const size_t i = static_cast<size_t>(sample);
            for (size_t j = 0; j < settings.time_steps; ++j) {
                const float coordinate =
                    static_cast<float>(j) *
                    static_cast<float>(support_count - 1) / denominator;
                float weighted_vx = 0.0f;
                float weighted_wz = 0.0f;
                float weighted_vy = 0.0f;
                float squared_weight_sum = 0.0f;
                for (size_t k = 0; k < support_count; ++k) {
                    const float normalized_distance =
                        (coordinate - static_cast<float>(k)) / sigma;
                    const float weight = std::exp(
                        -0.5f * normalized_distance * normalized_distance);
                    squared_weight_sum += weight * weight;
                    weighted_vx += weight * support_vx(i, k);
                    weighted_wz += weight * support_wz(i, k);
                    if (is_holonomic_) {
                        weighted_vy += weight * support_vy(i, k);
                    }
                }
                const float scale = 1.0f /
                    std::sqrt(std::max(squared_weight_sum, 1e-8f));
                noises_vx_(i, j) = scale * weighted_vx;
                noises_wz_(i, j) = scale * weighted_wz;
                if (is_holonomic_) {
                    noises_vy_(i, j) = scale * weighted_vy;
                }
            }
        }
    }

    void applyAntitheticAndNominalConstraintsUnlocked()
    {
        const auto & settings = settings_;
        // Pair positive and negative perturbations to reduce Monte-Carlo variance
        // without increasing the batch size.
        if (settings.antithetic_sampling && settings.batch_size > settings.nominal_sample_count + 1) {
            const size_t first = std::min<size_t>(
                settings.nominal_sample_count, settings.batch_size);
            const size_t available = settings.batch_size - first;
            const size_t pair_count = available / 2;
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(settings.thread_count)
#endif
            for (long long pair = 0; pair < static_cast<long long>(pair_count); ++pair) {
                const size_t i = static_cast<size_t>(pair);
                const size_t mirror = first + pair_count + i;
                for (size_t j = 0; j < settings.time_steps; ++j) {
                    noises_vx_(mirror, j) = -noises_vx_(first + i, j);
                    noises_wz_(mirror, j) = -noises_wz_(first + i, j);
                    if (is_holonomic_) {
                        noises_vy_(mirror, j) = -noises_vy_(first + i, j);
                    }
                }
            }
        }

        // 保留零噪声名义轨迹，避免当前控制序列被采样集合完全排除。
        const size_t nominal_count = std::min<size_t>(
            settings.nominal_sample_count, settings.batch_size);
        for (size_t i = 0; i < nominal_count; ++i) {
            for (size_t j = 0; j < settings.time_steps; ++j) {
                noises_vx_(i, j) = 0.0f;
                noises_wz_(i, j) = 0.0f;
                noises_vy_(i, j) = 0.0f;
            }
        }
    }

    xt::xtensor<float, 2> noises_vx_;
    xt::xtensor<float, 2> noises_vy_;
    xt::xtensor<float, 2> noises_wz_;
    OptimizerSettings settings_;
    bool is_holonomic_ = false;
    std::mutex noise_mutex_;
};

}  // namespace mppi

#endif  // MPPI_TOOLS_NOISE_GENERATOR_HPP_
