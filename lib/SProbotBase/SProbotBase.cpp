#include <iostream>
#include <sstream>
#include <iomanip>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <sys/types.h>
#include "SProbotBase.h"

SerialMotorController::SerialMotorController()
// : serial_fd(-1), running(false)
{
}

SerialMotorController::~SerialMotorController()
{
    stop();
    if (serial_fd >= 0)
    {
        close(serial_fd);
    }
}

bool SerialMotorController::init(int baud_rate, const std::string &port)
{
    port_name = port;
    serial_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1)
    {
        std::cerr << "无法打开串口: " << port_name << std::endl;
        return false;
    }

    struct termios options;
    if (tcgetattr(serial_fd, &options) != 0)
    {
        std::cerr << "获取串口属性失败" << std::endl;
        close(serial_fd);
        return false;
    }

    // 设置波特率
    cfsetispeed(&options, get_baud_rate(baud_rate));
    cfsetospeed(&options, get_baud_rate(baud_rate));

    // 设置数据格式：8位数据位，无校验，1位停止位
    options.c_cflag &= ~PARENB; // 无校验
    options.c_cflag &= ~CSTOPB; // 1位停止位
    options.c_cflag &= ~CSIZE;  // 清除数据位设置
    options.c_cflag |= CS8;     // 8位数据位

    options.c_cflag |= (CLOCAL | CREAD); // 本地连接，接收使能

    // 设置非规范模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    // 设置超时
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;

    tcflush(serial_fd, TCIFLUSH);
    if (tcsetattr(serial_fd, TCSANOW, &options) != 0)
    {
        std::cerr << "设置串口属性失败" << std::endl;
        close(serial_fd);
        return false;
    }

    return true;
}

bool SerialMotorController::start()
{
    if (serial_fd < 0)
    {
        std::cerr << "串口未初始化" << std::endl;
        return false;
    }

    running = true;
    communication_thread = std::thread(&SerialMotorController::communication_loop, this);
    return true;
}

void SerialMotorController::stop()
{
    running = false;
    if (communication_thread.joinable())
    {
        communication_thread.join();
    }
}

void SerialMotorController::setPWM(int16_t wl, int16_t wr, int16_t pl, int16_t pr)
{
    PWM_WL = wl;
    PWM_WR = wr;
    PWM_PL = pl;
    PWM_PR = pr;
}

uint16_t SerialMotorController::getEncoderL() const
{
    return Encoder_L.load();
}

uint16_t SerialMotorController::getEncoderR() const
{
    return Encoder_R.load();
}

double SerialMotorController::getSpeedL() const
{
    return speed_L.load();
}

double SerialMotorController::getSpeedR() const
{
    return speed_R.load();
}

speed_t SerialMotorController::get_baud_rate(int baud_rate)
{
    switch (baud_rate)
    {
    case 500000:
        return B500000;
    case 115200:
        return B115200;
    case 57600:
        return B57600;
    case 38400:
        return B38400;
    case 19200:
        return B19200;
    case 9600:
        return B9600;
    default:
        return B500000;
    }
}
void SerialMotorController::communication_loop()
{
    auto period = std::chrono::milliseconds(10);
    auto next_time = std::chrono::high_resolution_clock::now();

    while (running)
    {
        next_time += period;
        // auto loop_start = std::chrono::high_resolution_clock::now();

        // 发送PWM数据
        send_pwm_data();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        // 接收编码器数据
        receive_encoder_data();

        // 计算转速
        calculate_speed();

        // // 记录总循环时间
        // auto loop_end = std::chrono::high_resolution_clock::now();
        // loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start);

        // 精确延时
        std::this_thread::sleep_until(next_time);
    }
}
// void SerialMotorController::communication_loop()
// {
//     while (running)
//     {
//         // 发送PWM数据
//         send_pwm_data();

//         // 接收编码器数据
//         receive_encoder_data();

//         // 计算转速
//         calculate_speed();

//         // 延时100ms
//         std::this_thread::sleep_for(std::chrono::milliseconds(20));
//     }
// }
uint16_t SerialMotorController::write_hex_to_buf(char *databuf, uint16_t count, uint16_t data)
{
    static const char hex_chars[] = "0123456789ABCDEF";

    // 直接按位提取并写入
    databuf[count++] = hex_chars[(data >> 12) & 0xF];
    databuf[count++] = hex_chars[(data >> 8) & 0xF];
    databuf[count++] = hex_chars[(data >> 4) & 0xF];
    databuf[count++] = hex_chars[data & 0xF];

    return count;
}

uint16_t SerialMotorController::read_hex_from_buf(char *databuf, uint16_t count, uint16_t *data)
{
    uint16_t result = 0;

    // 逐字符转换4个16进制字符
    for (int i = 0; i < 4; i++)
    {
        char c = databuf[count + i];
        uint8_t value = 0;

        if (c >= '0' && c <= '9')
            value = c - '0';
        else if (c >= 'A' && c <= 'F')
            value = c - 'A' + 10;
        else if (c >= 'a' && c <= 'f')
            value = c - 'a' + 10;
        else
            return 0xFFFF;

        result = (result << 4) | value;
    }

    *data = result;
    return count + 4;
}

void SerialMotorController::send_pwm_data()
{
    // // 构造发送数据
    // std::stringstream ss;
    // ss << "@R;@S";
    // ss << std::hex << std::uppercase << std::setfill('0') << std::setw(4) << (uint16_t)PWM_WL.load();
    // ss << std::hex << std::uppercase << std::setfill('0') << std::setw(4) << (uint16_t)PWM_WR.load();
    // ss << std::hex << std::uppercase << std::setfill('0') << std::setw(4) << (uint16_t)PWM_PL.load();
    // ss << std::hex << std::uppercase << std::setfill('0') << std::setw(4) << (uint16_t)PWM_PR.load();
    // ss << ";";

    // std::string data = ss.str();
    uint16_t count = 0;
    databufTX[count++] = '@';
    databufTX[count++] = 'R';
    databufTX[count++] = ';';
    databufTX[count++] = '@';
    databufTX[count++] = 'S';
    count = write_hex_to_buf(databufTX, count, (uint16_t)PWM_WL.load());
    count = write_hex_to_buf(databufTX, count, (uint16_t)PWM_WR.load());
    count = write_hex_to_buf(databufTX, count, (uint16_t)PWM_PL.load());
    count = write_hex_to_buf(databufTX, count, (uint16_t)PWM_PR.load());
    databufTX[count++] = ';';
    // 发送数据
    ssize_t bytes_written = write(serial_fd, databufTX, count);
    if (bytes_written < 0)
    {
        std::cerr << "发送数据失败" << std::endl;
    }

    // tcdrain(serial_fd); // 等待数据发送完成
}

void SerialMotorController::receive_encoder_data()
{
    char buffer[256];
    ssize_t bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);

    if (bytes_read > 0)
    {
        buffer[bytes_read] = '\0';
        receive_buffer += std::string(buffer);

        // 处理完整的数据包
        process_receive_buffer();
    }
}

void SerialMotorController::process_receive_buffer()
{
    size_t start_pos = receive_buffer.find('[');
    while (start_pos != std::string::npos)
    {
        size_t end_pos = receive_buffer.find(']', start_pos);
        if (end_pos != std::string::npos && end_pos - start_pos == 9)
        {
            // 提取编码器数据 [XXXXYYYY]
            std::string data = receive_buffer.substr(start_pos + 1, 8);

            if (data.length() == 8)
            {
                try
                {
                    // 解析左编码器 (前4位)
                    uint16_t left_encoder = std::stoi(data.substr(0, 4), nullptr, 16);
                    // 解析右编码器 (后4位)
                    uint16_t right_encoder = std::stoi(data.substr(4, 4), nullptr, 16);

                    // 更新编码器值
                    {
                        std::lock_guard<std::mutex> lock(data_mutex);
                        Encoder_L = left_encoder;
                        Encoder_R = right_encoder;
                    }
                }
                catch (const std::exception &e)
                {
                    std::cerr << "解析编码器数据失败: " << e.what() << std::endl;
                }
            }

            // 移除已处理的数据
            receive_buffer = receive_buffer.substr(end_pos + 1);
            start_pos = receive_buffer.find('[');
        }
        else
        {
            // 数据不完整，等待更多数据
            break;
        }
    }

    // 保持缓冲区大小合理
    if (receive_buffer.length() > 1024)
    {
        receive_buffer = receive_buffer.substr(receive_buffer.length() - 512);
    }
}

void SerialMotorController::calculate_speed()
{
    std::lock_guard<std::mutex> lock(data_mutex);

    // 计算左电机转速
    int32_t delta_L = (int32_t)Encoder_L.load() - (int32_t)last_Encoder_L;
    // 处理溢出情况
    if (delta_L > 32768)
        delta_L -= 65536;
    if (delta_L < -32768)
        delta_L += 65536;

    // 转速 = 脉冲数/更新时间/每转脉冲数 (单位: 转/秒)
    double speed_L_new = delta_L / UPDATE_INTERVAL / PULSES_PER_REVOLUTION;
    speed_L = 0.15 * speed_L_new + 0.85 * speed_L;
    last_Encoder_L = Encoder_L.load();

    // 计算右电机转速
    int32_t delta_R = (int32_t)Encoder_R.load() - (int32_t)last_Encoder_R;
    // 处理溢出情况
    if (delta_R > 32768)
        delta_R -= 65536;
    if (delta_R < -32768)
        delta_R += 65536;

    double speed_R_new = delta_R / UPDATE_INTERVAL / PULSES_PER_REVOLUTION;
    speed_R = 0.15 * speed_R_new + 0.85 * speed_R;
    last_Encoder_R = Encoder_R.load();
}

bool SProbot::init(const std::string &sms_port, int sms_baud,
                   const std::string &imu_port, int imu_baud,
                   const std::string &smc_port, int smc_baud)
{
    std::string sms_port_name = sms_port;
    std::string imu_port_name = imu_port;
    std::string smc_port_name = smc_port;
    int sms_baud_rate = sms_baud;
    int imu_baud_rate = imu_baud;
    int smc_baud_rate = smc_baud;

    if (!sm_st.begin(sms_baud, sms_port_name.c_str()))
    {
        std::cout << "Failed to init scscl motor!" << std::endl;
        return 0;
    }
    if (!imu948.init(imu_baud_rate, imu_port_name.c_str()))
    {
        std::cerr << "imu948 初始化失败" << std::endl;
        return 0;
    }
    if (!imu948.start())
    {
        std::cerr << "imu948 启动失败" << std::endl;
        return 0;
    }
    if (!controller.init(smc_baud_rate, smc_port_name.c_str()))
    {
        std::cerr << "SerialMotorController 初始化失败" << std::endl;
        return 0;
    }
    if (!controller.start())
    {
        std::cerr << "SerialMotorController 启动失败" << std::endl;
        return 0;
    }
    initkey = true;
    return true;
}
void SProbot::start()
{
    if (initkey)
    {
        running = true;
        ctrl_thread = std::thread(&SProbot::ctrl_loop, this);
    }
}
void SProbot::stop()
{
    running = false;
    if (ctrl_thread.joinable())
    {
        ctrl_thread.join();
    }
    controller.stop();
    imu948.stop();
    sm_st.end();
}

SProbot::SProbot()
{
}
SProbot::~SProbot()
{
    stop();
}

void SProbot::ctrl_loop()
{

    // PIDController pitch_pid(0.5, 1.0, 0, 10.0f);
    // pitch_pid.setOutputLimits(-1.0, 1.0);
    // pitch_pid.setIntegralLimits(-2.0, 2.0);
    PIDController vxVib_pid(0.20, 0, 0, 10.0f); // 震荡抑制
    vxVib_pid.enableIntegral(false);
    vxVib_pid.enableDerivative(false);
    vxVib_pid.setOutputLimits(-0.4, 0.4);
    vxVib_pid.setDeadband(0.3);
    // vxVib_pid.setIntegralLimits(-2.0, 2.0);

    PIDController SL_pid(3.50, 4.0, 0, 10.0f);
    SL_pid.setOutputLimits(-1.0, 1.0);
    SL_pid.setIntegralLimits(-1.5, 1.5);

    PIDController SR_pid(3.50, 4.0, 0, 10.0f);
    SR_pid.setOutputLimits(-1.0, 1.0);
    SR_pid.setIntegralLimits(-1.5, 1.5);

    auto period = std::chrono::milliseconds(10);
    auto next_time = std::chrono::high_resolution_clock::now();
    uint8_t mod_old = -1;
    std::cout << "running!!" << std::endl;
    int32_t count = 0;
    while (running)
    {
        next_time += period;
        if (mod_old != mod)
        {
            switch (mod)
            {
            case 0:
                break;
            case 1:
                vxVib_pid.reset();
                SL_pid.reset();
                SR_pid.reset();
                break;
            case 2:
                break;
            case 3:
                break;
            }
            mod_old = mod;
        }
        switch (mod)
        {
        case 0:
            break;
        case 1:
        {
            vxVib_pid.setSetpoint(0);
            double pitch_w = imu948.angular_vel_y;
            double vxVib_out = vxVib_pid.compute(pitch_w);

            double goal_vl = goal_vx - goal_wz * 0.154 / 2.0;
            double goal_vr = goal_vx + goal_wz * 0.154 / 2.0;
            SL_pid.setSetpoint(goal_vl);
            SR_pid.setSetpoint(goal_vr);

            double vl_now = controller.getSpeedL() * 0.1;
            double vr_now = controller.getSpeedR() * 0.1;
            double vl_out = SL_pid.compute(vl_now);
            double vr_out = SR_pid.compute(vr_now);

            double pwm_l = 0, pwm_r = 0;

            pwm_l += (vl_out + vxVib_out);
            pwm_r += (vr_out + vxVib_out);

            double temp = std::max(std::abs(pwm_l), std::abs(pwm_r));
            if (temp > 1)
            {
                pwm_l /= temp;
                pwm_r /= temp;
            }
            controller.setPWM(pwm_l * 3500, pwm_r * 3500, PWM_PL * 3500, PWM_PR * 3500);

            if (++count % 10 == 0)
            {
                double vx_now = (controller.getSpeedR() + controller.getSpeedL()) / 2;
                double wz_now = (controller.getSpeedR() - controller.getSpeedL()) * 0.1 / 0.154;
                std::cout
                    // << "goal_vl: " << goal_vl
                    // << ", goal_vr: " << goal_vr
                    << ", pwm_l: " << pwm_l
                    << ", pwm_r: " << pwm_r
                    << ", pitch_w: " << pitch_w
                    << ", vxVib_out: " << vxVib_out
                    << std::endl;
            }
        }
        break;
        case 2:
            break;
        case 3:
            break;
        }

        std::this_thread::sleep_until(next_time);
    }
    std::cout << "end!!" << std::endl;
}
