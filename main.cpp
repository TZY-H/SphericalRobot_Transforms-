#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <iomanip>
#include "pid_controller.h"
#include "SProbotBase.h"
#include "IM948_CMD.h"
#include "UdpComm.h"
#include "SCServo.h"

#include <nlohmann/json.hpp> // 使用nlohmann/json库
#include <sstream>

// 使用nlohmann json库的命名空间
using json = nlohmann::json;

uint8_t updata_key = 0;
uint8_t mod = 0;
double speed_X = 0;
double speed_R = 0;
double speed_Pl = 0;
double speed_Pr = 0;
double tilt = 0;

// JSON解析回调函数
void onMessageReceived(const std::string &message, const std::string &ip, int port)
{
    std::cout << "来源: " << ip << ":" << port << std::endl;

    try
    {
        uint8_t key = 0;
        // 解析JSON数据
        json jsonData = json::parse(message);
        std::cout << "解析成功: " << jsonData.dump(4) << std::endl;

        // 根据业务需求处理不同的JSON格式
        if (jsonData.contains("speed"))
        {
            json &data = jsonData["speed"];
            speed_X = data[0];
            speed_R = data[1];
            speed_Pl = data[2];
            speed_Pr = data[3];
            key = 1;
        }
        if (jsonData.contains("mod"))
        {
            int data = jsonData["mod"];
            mod = (uint8_t)data;
            key = 1;
        }
        if (jsonData.contains("tilt"))
        {
            tilt = jsonData["tilt"];
            key = 1;
        }

        updata_key = key;
    }
    catch (const json::parse_error &e)
    {
        std::cerr << "JSON解析错误: " << e.what() << std::endl;
        std::cerr << "错误数据: " << message << std::endl;
    }
}

int main()
{
    // 创建UDP通信对象，监听5353端口
    UdpComm udpComm(5353, onMessageReceived);

    // 启动监听
    if (!udpComm.start())
    {
        std::cerr << "启动UDP通信失败" << std::endl;
        return -1;
    }

    std::cout << "UDP服务器启动，监听端口 5353" << std::endl;

    SProbot robot;
    robot.init("/dev/ttyS5", 500000,
               "/dev/ttyS1", 115200,
               "/dev/ttyS2", 500000);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    robot.start();
    robot.mod = 2;
    // robot.goal_vx = 0;
    // robot.goal_wz = 0.5;
    // double scale = 3500;
    // for (int32_t i = 0; i < 400; i++)
    // {
    //     // std::cout << robot.imu948.angular_vel_z << std::endl;
    //     std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }
    while (1)
    {
        if (updata_key)
        {
            updata_key = 0;
            robot.goal_vx = speed_X;
            robot.goal_wz = speed_R;
            robot.PWM_PL = speed_Pl;
            robot.PWM_PR = speed_Pr;
            robot.goal_tilt = tilt;
            // int16_t PWM_WL = (int16_t)(speed_X * scale);
            // int16_t PWM_WR = (int16_t)(speed_X * scale);
            // if (mod) // mod=0球形态
            // {
            //     PWM_WL -= ((int16_t)(speed_R * scale));
            //     PWM_WR += ((int16_t)(speed_R * scale));
            //     double temp = std::max(abs(PWM_WL), abs(PWM_WR)) / scale;
            //     if (temp > 1)
            //     {
            //         PWM_WL /= temp;
            //         PWM_WR /= temp;
            //     }
            // }
            // int16_t PWM_PL = 0, PWM_PR = 0;
            // if (fabs(speed_P) > 0.1)
            // {
            //     PWM_PL = (int16_t)(speed_P * scale);
            //     PWM_PR = (int16_t)(speed_P * scale * 0.9);
            // }
            // controller.setPWM(PWM_WL, PWM_WR, PWM_PL, PWM_PR);
            // int16_t tiltAngle = tilt * (4095 * 60 / 360) + 2047;

            // sm_st.WritePos(1, tiltAngle, 100, 500);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    robot.stop();
    return 0;
}