// ============================================================================
// 文件：critic_function.hpp
// 功能：代价函数基类定义
// ============================================================================
#ifndef MPPI_CRITICS_CRITIC_FUNCTION_HPP_
#define MPPI_CRITICS_CRITIC_FUNCTION_HPP_

#include <string>
#include <memory>

namespace mppi
{

// 前向声明
class MotionModel;
struct CriticData;

/**
 * @brief 代价函数基类，所有具体代价函数需继承并实现 score 方法
 */
class CriticFunction
{
public:
    CriticFunction() = default;
    virtual ~CriticFunction() = default;

    /** @brief 初始化代价函数（如加载参数） */
    virtual void initialize() = 0;

    /**
     * @brief 对采样轨迹计算代价，并累加到 data.costs 中
     * @param data 包含状态、轨迹、路径等信息的结构体
     */
    virtual void score(CriticData & data) = 0;

    /** @brief 启用/禁用该代价函数 */
    virtual void setEnabled(bool enabled) { enabled_ = enabled; }
    /** @return 是否启用 */
    bool isEnabled() const { return enabled_; }
    /** @brief 设置代价函数名称 */
    void setName(const std::string & name) { name_ = name; }
    /** @return 代价函数名称 */
    const std::string & getName() const { return name_; }

protected:
    bool enabled_ = true;
    std::string name_;
};

} // namespace mppi

#endif // MPPI_CRITICS_CRITIC_FUNCTION_HPP_
