#include "pid_controller.h"
#include <algorithm>
#include <cmath>
#include <cstring>

/**
 * @brief 构造函数
 */
PIDController::PIDController(double kp, double ki, double kd, double sample_time_ms)
    : kp_(kp), ki_(ki), kd_(kd), sample_time_(sample_time_ms / 1000.0f)
{
    initialize();
}

/**
 * @brief 拷贝构造函数
 */
PIDController::PIDController(const PIDController& other)
    : kp_(other.kp_), ki_(other.ki_), kd_(other.kd_)
    , setpoint_(other.setpoint_), input_(other.input_), output_(other.output_)
    , error_(other.error_), last_error_(other.last_error_), integral_(other.integral_)
    , output_max_(other.output_max_), output_min_(other.output_min_)
    , integral_max_(other.integral_max_), integral_min_(other.integral_min_)
    , sample_time_(other.sample_time_), last_time_(other.last_time_)
    , derivative_enabled_(other.derivative_enabled_), integral_enabled_(other.integral_enabled_)
    , initialized_(other.initialized_), deadband_(other.deadband_)
    , map_output_min_(other.map_output_min_), map_output_max_(other.map_output_max_)
    , map_scale_min_(other.map_scale_min_), map_scale_max_(other.map_scale_max_)
    , output_mapping_enabled_(other.output_mapping_enabled_)
{
}

/**
 * @brief 赋值操作符
 */
PIDController& PIDController::operator=(const PIDController& other)
{
    if (this != &other) {
        kp_ = other.kp_;
        ki_ = other.ki_;
        kd_ = other.kd_;
        setpoint_ = other.setpoint_;
        input_ = other.input_;
        output_ = other.output_;
        error_ = other.error_;
        last_error_ = other.last_error_;
        integral_ = other.integral_;
        output_max_ = other.output_max_;
        output_min_ = other.output_min_;
        integral_max_ = other.integral_max_;
        integral_min_ = other.integral_min_;
        sample_time_ = other.sample_time_;
        last_time_ = other.last_time_;
        derivative_enabled_ = other.derivative_enabled_;
        integral_enabled_ = other.integral_enabled_;
        initialized_ = other.initialized_;
        deadband_ = other.deadband_;
        map_output_min_ = other.map_output_min_;
        map_output_max_ = other.map_output_max_;
        map_scale_min_ = other.map_scale_min_;
        map_scale_max_ = other.map_scale_max_;
        output_mapping_enabled_ = other.output_mapping_enabled_;
    }
    return *this;
}

/**
 * @brief 初始化成员变量
 */
void PIDController::initialize()
{
    setpoint_ = 0.0f;
    input_ = 0.0f;
    output_ = 0.0f;
    error_ = 0.0f;
    last_error_ = 0.0f;
    integral_ = 0.0f;
    
    // 默认输出范围：0-100%
    output_max_ = 100.0f;
    output_min_ = 0.0f;
    integral_max_ = 1000.0f;
    integral_min_ = -1000.0f;
    
    last_time_ = getCurrentTimeMs();
    derivative_enabled_ = true;
    integral_enabled_ = true;
    initialized_ = true;
    deadband_ = 0.0f;
    
    // 初始化映射参数
    map_output_min_ = 0.0f;
    map_output_max_ = 100.0f;
    map_scale_min_ = 0.0f;
    map_scale_max_ = 100.0f;
    output_mapping_enabled_ = false;
}

/**
 * @brief 设置PID参数
 */
void PIDController::setParameters(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

/**
 * @brief 设置输出限制（完全支持正负值）
 */
void PIDController::setOutputLimits(double min, double max)
{
    if (min > max) return;
    
    output_min_ = min;
    output_max_ = max;
    
    // 如果设置为对称输出，自动更新映射参数
    if (min == -max) {
        map_output_min_ = min;
        map_output_max_ = max;
    }
}

/**
 * @brief 设置积分限制
 */
void PIDController::setIntegralLimits(double min, double max)
{
    if (min > max) return;
    
    integral_min_ = min;
    integral_max_ = max;
}

/**
 * @brief 设置目标值
 */
void PIDController::setSetpoint(double setpoint)
{
    setpoint_ = setpoint;
}

/**
 * @brief 获取目标值
 */
double PIDController::getSetpoint() const
{
    return setpoint_;
}

/**
 * @brief 计算PID输出（支持正负输出）
 */
double PIDController::compute(double feedback)
{
    if (!initialized_) return 0.0f;
    
    // 保存当前反馈值
    input_ = feedback;
    
    // 计算误差
    error_ = setpoint_ - input_;
    
    // 死区处理
    if (std::abs(error_) < deadband_) {
        error_ = 0.0f;
    }
    
    // 积分项计算
    if (integral_enabled_) {
        integral_ += error_ * sample_time_;
        // 积分限幅
        integral_ = constrain(integral_, integral_min_, integral_max_);
    }
    
    // 微分项计算
    double derivative = 0.0f;
    if (derivative_enabled_ && sample_time_ > 0.0f) {
        derivative = (error_ - last_error_) / sample_time_;
    }
    
    // PID输出计算
    double p_term = kp_ * error_;
    double i_term = integral_enabled_ ? ki_ * integral_ : 0.0f;
    double d_term = derivative_enabled_ ? kd_ * derivative : 0.0f;
    
    output_ = p_term + i_term + d_term;
    
    // 输出限幅（支持正负值）
    output_ = constrain(output_, output_min_, output_max_);
    
    // 保存当前误差用于下次计算
    last_error_ = error_;
    
    return output_;
}

/**
 * @brief 带时间戳的PID计算
 */
double PIDController::compute(double feedback, uint64_t timestamp)
{
    if (!initialized_) return 0.0f;
    
    // 计算实际采样时间
    if (timestamp > last_time_) {
        sample_time_ = (timestamp - last_time_) / 1000.0f; // 转换为秒
    }
    last_time_ = timestamp;
    
    return compute(feedback);
}

/**
 * @brief 使用系统时间的PID计算
 */
double PIDController::computeWithSystemTime(double feedback)
{
    uint64_t current_time = getCurrentTimeMs();
    return compute(feedback, current_time);
}

/**
 * @brief 重置PID控制器
 */
void PIDController::reset()
{
    error_ = 0.0f;
    last_error_ = 0.0f;
    integral_ = 0.0f;
    output_ = 0.0f;
    input_ = 0.0f;
    last_time_ = getCurrentTimeMs();
}

/**
 * @brief 获取当前误差
 */
double PIDController::getError() const
{
    return error_;
}

/**
 * @brief 获取当前积分值
 */
double PIDController::getIntegral() const
{
    return integral_;
}

/**
 * @brief 获取当前输出值
 */
double PIDController::getOutput() const
{
    return output_;
}

/**
 * @brief 获取当前输入值
 */
double PIDController::getInput() const
{
    return input_;
}

/**
 * @brief 设置采样时间
 */
void PIDController::setSampleTime(double sample_time_ms)
{
    sample_time_ = sample_time_ms / 1000.0f;
}

/**
 * @brief 启用/禁用微分项
 */
void PIDController::enableDerivative(bool enable)
{
    derivative_enabled_ = enable;
}

/**
 * @brief 启用/禁用积分项
 */
void PIDController::enableIntegral(bool enable)
{
    integral_enabled_ = enable;
}

/**
 * @brief 设置死区
 */
void PIDController::setDeadband(double deadband)
{
    deadband_ = std::abs(deadband);
}

/**
 * @brief 设置输出映射（用于PWM映射）
 */
void PIDController::setOutputMapping(double output_min, double output_max, double scale_min, double scale_max)
{
    if (output_min >= output_max || scale_min >= scale_max) return;
    
    map_output_min_ = output_min;
    map_output_max_ = output_max;
    map_scale_min_ = scale_min;
    map_scale_max_ = scale_max;
    output_mapping_enabled_ = true;
}

/**
 * @brief 获取映射后的输出值
 */
double PIDController::computeMapped(double feedback)
{
    double raw_output = compute(feedback);
    
    if (output_mapping_enabled_) {
        return map(raw_output, map_output_min_, map_output_max_, map_scale_min_, map_scale_max_);
    }
    
    return raw_output;
}

/**
 * @brief 检查输出限制是否对称
 */
bool PIDController::isOutputSymmetric() const
{
    return (output_min_ == -output_max_);
}

/**
 * @brief 限制值在指定范围内
 */
double PIDController::constrain(double value, double min, double max) const
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/**
 * @brief 线性映射函数
 */
double PIDController::map(double value, double in_min, double in_max, double out_min, double out_max) const
{
    if (in_min == in_max) return out_min;
    
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief 获取当前系统时间戳(毫秒)
 */
uint64_t PIDController::getCurrentTimeMs() const
{
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}