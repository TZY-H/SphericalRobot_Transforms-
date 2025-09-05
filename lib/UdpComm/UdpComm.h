#ifndef UDP_COMM_H
#define UDP_COMM_H

#include <string>
#include <thread>
#include <functional>
#include <atomic>
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <cstring>
#endif

class UdpComm {
public:
    // 回调函数类型定义
    using MessageCallback = std::function<void(const std::string& message, const std::string& ip, int port)>;
    
    /**
     * 构造函数
     * @param localPort 本地监听端口
     * @param callback 接收到消息时的回调函数
     */
    UdpComm(int localPort, MessageCallback callback = nullptr);
    
    /**
     * 析构函数
     */
    ~UdpComm();
    
    /**
     * 启动UDP监听线程
     * @return 是否启动成功
     */
    bool start();
    
    /**
     * 停止UDP监听线程
     */
    void stop();
    
    /**
     * 发送数据到指定地址
     * @param data 要发送的数据
     * @param ip 目标IP地址
     * @param port 目标端口
     * @return 发送是否成功
     */
    bool sendTo(const std::string& data, const std::string& ip, int port);
    
    /**
     * 设置消息回调函数
     * @param callback 回调函数
     */
    void setMessageCallback(MessageCallback callback);
    
    /**
     * 检查是否正在运行
     * @return 是否正在运行
     */
    bool isRunning() const;

private:
    /**
     * 监听线程函数
     */
    void listenThread();
    
    /**
     * 初始化socket
     * @return 是否初始化成功
     */
    bool initSocket();
    
    /**
     * 关闭socket
     */
    void closeSocket();
    
    // 成员变量
    int localPort_;                              // 本地监听端口
    MessageCallback messageCallback_;            // 消息回调函数
    std::unique_ptr<std::thread> listenThread_;  // 监听线程
    std::atomic<bool> running_;                  // 运行状态标志
    int socketFd_;                               // socket文件描述符
    struct sockaddr_in localAddr_;               // 本地地址结构
    
#ifdef _WIN32
    WSADATA wsaData_;
#endif
};

#endif // UDP_COMM_H