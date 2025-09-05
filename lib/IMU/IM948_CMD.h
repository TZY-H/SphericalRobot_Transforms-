/******************************************************************************
                        设备与IM948模块之间的串口通信库
版本: V1.04
记录: 1、增加 加速计和陀螺仪量程可设置
      2、增加 磁场校准开始命令
      3、增加 设置陀螺仪自动校正标识命令
      4、增加 设置静止节能模式的触发时长
      5、增加 设置上传圈数和支持数据透传
*******************************************************************************/
#ifndef _im948_CMD_h
#define _im948_CMD_h

#include <stdio.h>
#include <math.h>
#include <thread>
#include <atomic>
#include <string>
#include <termios.h>

typedef signed char S8;
typedef unsigned char U8;
typedef signed short S16;
typedef unsigned short U16;
typedef signed int S32;
typedef unsigned int U32;
typedef float F32;
#define pow2(x) ((x) * (x)) // 求平方

// ===============================调试信息开关====================================
// #define __Debug  // 使用调试口输出调试信息,不使用调试信息屏蔽本句即可
#ifdef __Debug
#define Dbp(fmt, args...) printf(fmt, ##args) // 若需要使用调试信息, 用户对接Dbp函数名即可
extern void Dbp_U8_buf(const char *sBeginInfo, const char *sEndInfo, const char *sFormat, const U8 *Buf, U32 Len);
#else
#define Dbp(fmt, args...)
#define Dbp_U8_buf(sBeginInfo, sEndInfo, sFormat, Buf, Len)
#endif

class IM948
{
private:
    std::string port_name;
    int serial_fd;
    std::atomic<bool> running;
    std::thread communication_thread;

    speed_t get_baud_rate(int baud_rate);
    // 通信主循环
    void communication_loop();

    U8 targetDeviceAddress = 255; // 通信地址，设为0-254指定则设备地址，设为255则不指定设备(即广播), 当需要使用485总线形式通信时通过该参数选中要操作的设备，若仅仅是串口1对1通信设为广播地址255即可
    // =================================移植接口======================================
    /**
     * 用于捕获数据包, 用户只需把接收到的每字节数据传入该函数即可
     * @param byte 传入接收到的每字节数据
     * @return U8 1=接收到完整数据包, 0未获取到完整数据包
     */
    U8 Cmd_GetPkt(U8 byte);
    int Cmd_PackAndTx(U8 *pDat, U8 DLen);
    void Cmd_Write(U8 *pBuf, int Len);
    void Cmd_RxUnpack(U8 *buf, U8 DLen);
    // 当接收到有效数据包，会回调进入到 Cmd_RxUnpack(U8 *buf, U8 DLen) 函数里，用户在该函数里处理数据即可

public:
    double linear_acc = 0; // 加速度xyz 去掉了重力
    double linear_acc_x = 0;
    double linear_acc_y = 0;
    double linear_acc_z = 0;

    double linear_accG = 0; // 加速度xyz 包含了重力
    double linear_accG_x = 0;
    double linear_accG_y = 0;
    double linear_accG_z = 0;

    double angular_vel = 0; // 角速度xyz
    double angular_vel_x = 0;
    double angular_vel_y = 0;
    double angular_vel_z = 0;

    double Temperature = 0, Pressure = 0, Altitude = 0; // 温度 气压 高度

    double orientation_x = 0; // 四元素 wxyz
    double orientation_y = 0;
    double orientation_z = 0;
    double orientation_w = 0;

    double euler_x = 0; // 欧拉角xyz
    double euler_y = 0;
    double euler_z = 0;

    IM948();
    ~IM948();

    // 初始化串口
    bool init(int baud_rate = 500000,const std::string &port = "/dev/ttyUSB0");
    // 启动通信线程
    bool start();
    // 停止通信线程
    void stop();

    void im948_test(void); // 测试示例 功能是监听调试串口发来的操作指令，然后对模块进行操作，放在循环里即可

    // ================================模块的操作指令=================================
    void Cmd_02(void); // 睡眠传感器
    void Cmd_03(void); // 唤醒传感器
    void Cmd_18(void); // 关闭数据主动上报
    void Cmd_19(void); // 开启数据主动上报
    void Cmd_11(void); // 获取1次订阅的功能数据
    void Cmd_10(void); // 获取设备属性和状态
    /**
     * 设置设备参数
     * @param accStill    惯导-静止状态加速度阀值 单位dm/s?
     * @param stillToZero 惯导-静止归零速度(单位cm/s) 0:不归零 255:立即归零
     * @param moveToZero  惯导-动态归零速度(单位cm/s) 0:不归零
     * @param isCompassOn 1=需融合磁场 0=不融合磁场
     * @param barometerFilter 气压计的滤波等级[取值0-3],数值越大越平稳但实时性越差
     * @param reportHz 数据主动上报的传输帧率[取值0-250HZ], 0表示0.5HZ
     * @param gyroFilter    陀螺仪滤波系数[取值0-2],数值越大越平稳但实时性越差
     * @param accFilter     加速计滤波系数[取值0-4],数值越大越平稳但实时性越差
     * @param compassFilter 磁力计滤波系数[取值0-9],数值越大越平稳但实时性越差
     * @param Cmd_ReportTag 功能订阅标识
     */
    void Cmd_12(U8 accStill, U8 stillToZero, U8 moveToZero, U8 isCompassOn, U8 barometerFilter, U8 reportHz, U8 gyroFilter, U8 accFilter, U8 compassFilter, U16 Cmd_ReportTag);
    void Cmd_13(void); // 惯导三维空间位置清零
    void Cmd_16(void); // 计步数清零
    void Cmd_14(void); // 恢复出厂校准参数
    void Cmd_15(void); // 保存当前校准参数为出厂校准参数
    void Cmd_07(void); // 加速计简易校准 模块静止在水平面时，发送该指令并收到回复后等待5秒即可
    /**
     * 加速计高精度校准
     * @param flag 若模块未处于校准状态时：
     *                 值0 表示请求开始一次校准并采集1个数据
     *                 值255 表示询问设备是否正在校准
     *             若模块正在校准中:
     *                 值1 表示要采集下1个数据
     *                 值255 表示要采集最后1个数据并结束
     */
    void Cmd_17(U8 flag);

    void Cmd_32(void); // 开始磁力计校准
    void Cmd_04(void); // 结束磁力计校准
    void Cmd_05(void); // z轴角归零
    void Cmd_06(void); // xyz世界坐标系清零
    void Cmd_08(void); // 恢复默认的自身坐标系Z轴指向及恢复默认的世界坐标系
    /**
     * 设置PCB安装方向矩阵
     * @param accMatrix 加速计方向矩阵
     * @param comMatrix 磁力计方向矩阵
     */
    void Cmd_20(S8 *accMatrix, S8 *comMatrix);
    void Cmd_21(void); // 读取PCB安装方向矩阵
    /**
     * 设置蓝牙广播名称
     *
     * @param bleName 蓝牙名称(最多支持15个字符长度,不支持中文)
     */
    void Cmd_22(const char *bleName);
    void Cmd_23(void); // 读取蓝牙广播名称
    /**
     * 设置关机电压和充电参数
     * @param PowerDownVoltageFlag 关机电压选择 0=3.4V(锂电池用) 1=2.7V(其它干电池用)
     * @param charge_full_mV  充电截止电压 0:3962mv 1:4002mv 2:4044mv 3:4086mv 4:4130mv 5:4175mv 6:4222mv 7:4270mv 8:4308mv 9:4349mv 10:4391mv
     * @param charge_full_mA 充电截止电流 0:2ma 1:5ma 2:7ma 3:10ma 4:15ma 5:20ma 6:25ma 7:30ma
     * @param charge_mA      充电电流 0:20ma 1:30ma 2:40ma 3:50ma 4:60ma 5:70ma 6:80ma 7:90ma 8:100ma 9:110ma 10:120ma 11:140ma 12:160ma 13:180ma 14:200ma 15:220ma
     */
    void Cmd_24(U8 PowerDownVoltageFlag, U8 charge_full_mV, U8 charge_full_mA, U8 charge_mA);
    void Cmd_25(void); // 读取 关机电压和充电参数
    void Cmd_26(void); // 断开蓝牙连接
    /**
     * 设置用户的GPIO引脚
     *
     * @param M 0=浮空输入, 1=上拉输入, 2=下拉输入, 3=输出0, 4=输出1
     */
    void Cmd_27(U8 M);
    void Cmd_2A(void); // 设备重启
    void Cmd_2B(void); // 设备关机
    /**
     * 设置 空闲关机时长
     *
     * @param idleToPowerOffTime 当串口没有通信且蓝牙在广播中，连续计时达到这么多个10分钟则关机  0=不关机
     */
    void Cmd_2C(U8 idleToPowerOffTime);
    void Cmd_2D(void); // 读取 空闲关机时长
    /**
     * 设置 禁止蓝牙方式更改名称和充电参数 标识
     *
     * @param DisableBleSetNameAndCahrge 1=禁止通过蓝牙更改名称及充电参数 0=允许(默认) 可能客户的产品不想让别人用蓝牙随便改，设为1即可
     */
    void Cmd_2E(U8 DisableBleSetNameAndCahrge);
    void Cmd_2F(void); // 读取 禁止蓝牙方式更改名称和充电参数 标识
    /**
     * 设置 串口通信地址
     *
     * @param address 设备地址只能设置为0-254
     */
    void Cmd_30(U8 address);
    void Cmd_31(void); // 读取 串口通信地址
    /**
     * 设置 加速计和陀螺仪量程
     *
     * @param AccRange  目标加速度量程 0=2g 1=4g 2=8g 3=16g
     * @param GyroRange 目标陀螺仪量程 0=256 1=512 2=1024 3=2048
     */
    void Cmd_33(U8 AccRange, U8 GyroRange);
    void Cmd_34(void); // 读取 加速计和陀螺仪量程
    /**
     * 设置 陀螺仪自动校正标识
     *
     * @param GyroAutoFlag  1=陀螺仪自动校正灵敏度开  0=关
     */
    void Cmd_35(U8 GyroAutoFlag);
    void Cmd_36(void); // 读取 加速计和陀螺仪量程
    /**
     * 设置 静止节能模式的触发时长
     *
     * @param EcoTime_10s 该值大于0，则开启自动节能模式(即传感器睡眠后不主动上报，或静止EcoTime_10s个10秒自动进入运动监测模式且暂停主动上报)  0=不启用自动节能
     */
    void Cmd_37(U8 EcoTime_10s);
    void Cmd_38(void); // 读取 静止节能模式的触发时长
    /**
     * 设置 当前高度为指定值
     *
     * @param val 要设置的高度值 单位为mm
     */
    void Cmd_42(S32 val);
    /**
     * 设置 自动补偿高度标识
     *
     * @param OnOff 0=关闭 1=开启
     */
    void Cmd_43(U8 OnOff);
    // 读取 自动补偿高度标识
    void Cmd_44(void);
    /**
     * 设置 串口波特率
     *
     * @param BaudRate 目标波特率 0=9600 1=115200 2=230400 3=460800
     */
    void Cmd_47(U8 BaudRate);
    // 读取 串口波特率
    void Cmd_48(void);
    // 闪烁几下led指示灯
    void Cmd_49(void);
    /**
     * 数据透传
     *
     * @param TxBuf 要透传的数据
     * @param TxLen 字节数 必须小于CmdPacketMaxDatSizeTx
     */
    void Cmd_50(U8 *TxBuf, U8 TxLen);
    /**
     * 设置 是否上传总圈数并清零
     *
     * @param isReportCycle 0=传输欧拉角数据, 1=用总圈数代替欧拉角传输 并清零圈数
     */
    void Cmd_51(U8 isReportCycle);
};

#endif
