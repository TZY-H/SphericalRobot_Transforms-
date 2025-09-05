#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <cstdint>
#include <chrono>

/**
 * @brief PID控制器类
 */
class PIDController {
public:
    /**
     * @brief 构造函数
     * @param kp 比例系数
     * @param ki 积分系数
     * @param kd 微分系数
     * @param sample_time_ms 采样时间(毫秒)
     */
    PIDController(double kp = 1.0f, double ki = 0.0f, double kd = 0.0f, double sample_time_ms = 10.0f);
    
    /**
     * @brief 析构函数
     */
    ~PIDController() = default;
    
    /**
     * @brief 拷贝构造函数
     */
    PIDController(const PIDController& other);
    
    /**
     * @brief 赋值操作符
     */
    PIDController& operator=(const PIDController& other);
    
    /**
     * @brief 设置PID参数
     * @param kp 比例系数
     * @param ki 积分系数
     * @param kd 微分系数
     */
    void setParameters(double kp, double ki, double kd);
    
    /**
     * @brief 设置输出限制
     * @param min 最小输出值
     * @param max 最大输出值
     */
    void setOutputLimits(double min, double max);
    
    /**
     * @brief 设置积分限制
     * @param min 最小积分值
     * @param max 最大积分值
     */
    void setIntegralLimits(double min, double max);
    
    /**
     * @brief 设置目标值
     * @param setpoint 目标值
     */
    void setSetpoint(double setpoint);
    
    /**
     * @brief 获取目标值
     * @return 当前目标值
     */
    double getSetpoint() const;
    
    /**
     * @brief 计算PID输出
     * @param feedback 当前反馈值
     * @return PID输出值
     */
    double compute(double feedback);
    
    /**
     * @brief 带时间戳的PID计算（自动计算时间差）
     * @param feedback 当前反馈值
     * @param timestamp 时间戳（毫秒）
     * @return PID输出值
     */
    double compute(double feedback, uint64_t timestamp);
    
    /**
     * @brief 使用系统时间的PID计算
     * @param feedback 当前反馈值
     * @return PID输出值
     */
    double computeWithSystemTime(double feedback);
    
    /**
     * @brief 重置PID控制器
     */
    void reset();
    
    /**
     * @brief 获取当前误差
     * @return 当前误差值
     */
    double getError() const;
    
    /**
     * @brief 获取当前积分值
     * @return 当前积分值
     */
    double getIntegral() const;
    
    /**
     * @brief 获取当前输出值
     * @return 当前输出值
     */
    double getOutput() const;
    
    /**
     * @brief 获取当前输入值（反馈值）
     * @return 当前输入值
     */
    double getInput() const;
    
    /**
     * @brief 设置采样时间
     * @param sample_time_ms 采样时间(毫秒)
     */
    void setSampleTime(double sample_time_ms);
    
    /**
     * @brief 启用/禁用微分项
     * @param enable 是否启用
     */
    void enableDerivative(bool enable);
    
    /**
     * @brief 启用/禁用积分项
     * @param enable 是否启用
     */
    void enableIntegral(bool enable);
    
    /**
     * @brief 设置死区
     * @param deadband 死区值
     */
    void setDeadband(double deadband);
    
    /**
     * @brief 设置输出映射（用于PWM映射）
     * @param output_min 实际输出最小值
     * @param output_max 实际输出最大值
     * @param scale_min 映射范围最小值
     * @param scale_max 映射范围最大值
     */
    void setOutputMapping(double output_min, double output_max, double scale_min, double scale_max);
    
    /**
     * @brief 获取映射后的输出值
     * @param feedback 当前反馈值
     * @return 映射后的输出值
     */
    double computeMapped(double feedback);
    
    /**
     * @brief 检查输出限制是否对称（用于判断是否为双向控制）
     * @return true: 对称输出 false: 非对称输出
     */
    bool isOutputSymmetric() const;

private:
    // PID参数
    double kp_;
    double ki_;
    double kd_;
    
    // 运行时变量
    double setpoint_;
    double input_;
    double output_;
    
    // PID计算中间变量
    double error_;
    double last_error_;
    double integral_;
    
    // 限制参数
    double output_max_;
    double output_min_;
    double integral_max_;
    double integral_min_;
    
    // 时间相关
    double sample_time_;  // 采样时间(秒)
    uint64_t last_time_; // 上次计算时间戳(毫秒)
    
    // 功能开关
    bool derivative_enabled_;
    bool integral_enabled_;
    bool initialized_;
    
    // 死区
    double deadband_;
    
    // 输出映射参数
    double map_output_min_;
    double map_output_max_;
    double map_scale_min_;
    double map_scale_max_;
    bool output_mapping_enabled_;
    
    /**
     * @brief 初始化成员变量
     */
    void initialize();
    
    /**
     * @brief 限制值在指定范围内
     * @param value 待限制的值
     * @param min 最小值
     * @param max 最大值
     * @return 限制后的值
     */
    double constrain(double value, double min, double max) const;
    
    /**
     * @brief 线性映射函数
     * @param value 输入值
     * @param in_min 输入范围最小值
     * @param in_max 输入范围最大值
     * @param out_min 输出范围最小值
     * @param out_max 输出范围最大值
     * @return 映射后的值
     */
    double map(double value, double in_min, double in_max, double out_min, double out_max) const;
    
    /**
     * @brief 获取当前系统时间戳(毫秒)
     * @return 当前时间戳
     */
    uint64_t getCurrentTimeMs() const;
};

class BidirectionalMotor {
private:
    double speed_;
    double inertia_;
    
public:
    BidirectionalMotor(double inertia = 0.1f) : speed_(0.0f), inertia_(inertia) {}
    
    void update(double pwm, double dt) {
        // 双向电机模型：PWM -50到+50对应-500到+500 RPM
        double target_speed = pwm * 10.0f; // -500到+500 RPM
        speed_ += (target_speed - speed_) * inertia_ * dt;
    }
    
    double getSpeed() const {
        return speed_;
    }
};

#endif // PID_CONTROLLER_H