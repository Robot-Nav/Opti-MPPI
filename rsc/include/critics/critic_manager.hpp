// ============================================================================
// 文件：critic_manager.hpp
// 功能：代价函数管理器，负责存储所有代价函数并批量执行评分
//       支持每个Critic的独立代价统计（均值/最小/最大），便于调参分析
// ============================================================================
#ifndef MPPI_CRITICS_CRITIC_MANAGER_HPP_
#define MPPI_CRITICS_CRITIC_MANAGER_HPP_

#include <vector>
#include <memory>
#include <string>
#include <chrono>
#include <limits>

#include <xtensor/xtensor.hpp>

#include "critics/critic_function.hpp"
#include "critics/critic_data.hpp"

namespace mppi
{

/**
 * @brief 单个Critic的代价统计
 */
struct CriticStatistics
{
    std::string name;
    float mean_cost = 0.0f;
    float min_cost = 0.0f;
    float max_cost = 0.0f;
    double elapsed_ms = 0.0f;
};

/**
 * @brief 代价函数管理器，负责存储所有代价函数并批量执行评分
 */
class CriticManager
{
public:
    CriticManager() = default;
    /** @brief 添加一个代价函数（接管所有权） */
    void addCritic(std::unique_ptr<CriticFunction> critic) { critics_.push_back(std::move(critic)); }
    /** @brief 初始化所有代价函数 */
    void initializeCritics() { for (auto & c : critics_) c->initialize(); }
    void setCollectStatistics(bool enabled)
    {
        collect_statistics_ = enabled;
        if (!enabled) statistics_.clear();
    }

    /**
     * @brief 对所有代价函数执行评分，累加代价到 data.costs
     *        同时统计每个Critic的代价贡献和耗时
     * @param data 评分所需数据
     */
    void evalTrajectoriesScores(CriticData & data) const
    {
        for (size_t i = 0; i < data.costs.shape(0); ++i) data.costs(i) = 0.0f;

        statistics_.clear();
        for (const auto & critic : critics_) {
            if (!critic->isEnabled()) continue;

            if (!collect_statistics_) {
                critic->score(data);
                continue;
            }

            // 保存评分前的costs快照
            auto costs_before = data.costs;

            // 计时
            auto start = std::chrono::steady_clock::now();
            critic->score(data);
            auto end = std::chrono::steady_clock::now();

            // 统计代价增量
            CriticStatistics stat;
            stat.name = critic->getName();
            stat.elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
            float sum = 0.0f, mn = std::numeric_limits<float>::max(), mx = -std::numeric_limits<float>::max();
            for (size_t i = 0; i < data.costs.shape(0); ++i) {
                float delta = data.costs(i) - costs_before(i);
                sum += delta;
                if (delta < mn) mn = delta;
                if (delta > mx) mx = delta;
            }
            stat.mean_cost = data.costs.shape(0) > 0 ? sum / data.costs.shape(0) : 0.0f;
            stat.min_cost = mn;
            stat.max_cost = mx;
            statistics_.push_back(stat);
        }
    }

    /** @return 最近一次评分的各Critic统计 */
    const std::vector<CriticStatistics>& getStatistics() const { return statistics_; }

    /**
     * @brief 根据名称获取代价函数指针（用于外部设置参数）
     */
    CriticFunction* getCritic(const std::string & name) const
    {
        for (const auto & c : critics_)
            if (c->getName() == name) return c.get();
        return nullptr;
    }

private:
    std::vector<std::unique_ptr<CriticFunction>> critics_;
    mutable std::vector<CriticStatistics> statistics_;
    bool collect_statistics_ = false;
};

} // namespace mppi

#endif // MPPI_CRITICS_CRITIC_MANAGER_HPP_
