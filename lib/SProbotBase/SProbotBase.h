#ifndef SERIAL_MOTOR_CONTROLLER_H
#define SERIAL_MOTOR_CONTROLLER_H

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
// #include <sys/types.h>
#include <termios.h>

#include "pid_controller.h"
// #include "SProbotBase.h"
#include "IM948_CMD.h"
// #include "UdpComm.h"
#include "SCServo.h"

class SerialMotorController
{
private:
    std::string port_name;
    int serial_fd;
    std::atomic<bool> running;
    std::thread communication_thread;
    std::mutex data_mutex;

    // 电机控制参数
    std::atomic<int16_t> PWM_WL{0};
    std::atomic<int16_t> PWM_WR{0};
    std::atomic<int16_t> PWM_PL{0};
    std::atomic<int16_t> PWM_PR{0};

    // 编码器数据
    std::atomic<uint16_t> Encoder_L{0};
    std::atomic<uint16_t> Encoder_R{0};

    // 上次编码器值（用于计算转速）
    uint16_t last_Encoder_L = 0;
    uint16_t last_Encoder_R = 0;

    // 转速数据
    std::atomic<double> speed_L{0.0};
    std::atomic<double> speed_R{0.0};

    // 转速计算参数
    static constexpr double PULSES_PER_REVOLUTION = 75.0 * 44.0 / 6.2831853; // 根据实际编码器设置
    static constexpr double UPDATE_INTERVAL = 0.01;              // 10ms更新间隔
    // 接收缓冲区
    std::string receive_buffer;
    char databufTX[128];

    uint16_t write_hex_to_buf(char *databuf, uint16_t count, uint16_t data);
    uint16_t read_hex_from_buf(char *databuf, uint16_t count, uint16_t *data);

public:
    SerialMotorController();
    ~SerialMotorController();
    std::chrono::microseconds loop_duration;

    // 初始化串口
    bool init(int baud_rate = 500000, const std::string &port = "/dev/ttyUSB0");

    // 启动通信线程
    bool start();

    // 停止通信线程
    void stop();

    // 设置PWM值
    void setPWM(int16_t wl, int16_t wr, int16_t pl, int16_t pr);

    // 获取编码器值
    uint16_t getEncoderL() const;
    uint16_t getEncoderR() const;

    // 获取转速值
    double getSpeedL() const;
    double getSpeedR() const;

private:
    // 根据波特率获取termios常量
    speed_t get_baud_rate(int baud_rate);

    // 通信主循环
    void communication_loop();

    // 发送PWM数据
    void send_pwm_data();

    // 接收编码器数据
    void receive_encoder_data();

    // 处理接收缓冲区
    void process_receive_buffer();

    // 计算转速
    void calculate_speed();
};

class SProbot
{
private:
    std::string sms_port_name;
    std::string imu_port_name;
    std::string smc_port_name;
    int sms_baud_rate;
    int imu_baud_rate;
    int smc_baud_rate;

    bool initkey = false;
    std::atomic<bool> running;
    std::thread ctrl_thread;
    void ctrl_loop();

    // 1: 二轮差速
    // 外环1 goal_vx 内环1 goal_pitch
    // 外环2 goal_wz 内环1 无

    // 2: 陆地球
    // 外环1 goal_vx   内环1 goal_pitch
    // 外环2 goal_roll 内环2 偏向锤角度

    // 2: 水面球
    // 外环1 goal_vx   内环1 goal_pitch
    // 外环2 goal_roll 内环2 偏向锤角度

    double goal_pitch = 0; // rad
    double goal_roll = 0;  // rad


    int32_t dogtime = 0;

public:
    double goal_vx = 0; // m/s
    double goal_wz = 0; // rad/s


    SMS_STS sm_st;
    SerialMotorController controller;
    IM948 imu948;
    SProbot();
    ~SProbot();
    bool init(const std::string &sms_port, int sms_baud,
               const std::string &imu_port, int imu_baud,
               const std::string &smc_port, int smc_baud);
    void start();
    void stop();

    uint8_t mod = 0; // 0:手动模式; 1:二轮差速; 2:陆地球; 3:水面球;
    double PWM_PL = 0;
    double PWM_PR = 0;
};

#endif
