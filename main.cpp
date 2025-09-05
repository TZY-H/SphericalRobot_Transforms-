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
double speed_P = 0;
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
            speed_P = data[2];
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
    SProbot robot;
    robot.init("/dev/ttyS5", 500000,
               "/dev/ttyS1", 115200,
               "/dev/ttyS2", 500000);

    double scale = 3500;
    while (1)
    {
        std::cout << robot.imu948.angular_vel_z << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    robot.stop();
    return 0;
}