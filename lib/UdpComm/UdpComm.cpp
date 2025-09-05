#include "UdpComm.h"
#include <iostream>
#include <cstring>

UdpComm::UdpComm(int localPort, MessageCallback callback)
    : localPort_(localPort)
    , messageCallback_(callback)
    , listenThread_(nullptr)
    , running_(false)
    , socketFd_(-1) {
    
#ifdef _WIN32
    // Windows socket初始化
    WSAStartup(MAKEWORD(2, 2), &wsaData_);
#endif
    
    // 初始化本地地址结构
    memset(&localAddr_, 0, sizeof(localAddr_));
    localAddr_.sin_family = AF_INET;
    localAddr_.sin_addr.s_addr = INADDR_ANY;
    localAddr_.sin_port = htons(localPort_);
}

UdpComm::~UdpComm() {
    stop();
    
#ifdef _WIN32
    WSACleanup();
#endif
}

bool UdpComm::start() {
    if (running_) {
        return true; // 已经在运行
    }
    
    if (!initSocket()) {
        return false;
    }
    
    running_ = true;
    listenThread_ = std::make_unique<std::thread>(&UdpComm::listenThread, this);
    
    return true;
}

void UdpComm::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    if (listenThread_ && listenThread_->joinable()) {
        listenThread_->join();
    }
    
    closeSocket();
}

bool UdpComm::sendTo(const std::string& data, const std::string& ip, int port) {
    if (!running_ || socketFd_ < 0) {
        std::cerr << "UDP socket not initialized" << std::endl;
        return false;
    }
    
    struct sockaddr_in destAddr;
    memset(&destAddr, 0, sizeof(destAddr));
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(port);
    
#ifdef _WIN32
    destAddr.sin_addr.S_un.S_addr = inet_addr(ip.c_str());
#else
    if (inet_aton(ip.c_str(), &destAddr.sin_addr) == 0) {
        std::cerr << "Invalid IP address: " << ip << std::endl;
        return false;
    }
#endif
    
    int sentBytes = sendto(socketFd_, data.c_str(), data.length(), 0,
                          (struct sockaddr*)&destAddr, sizeof(destAddr));
    
    if (sentBytes < 0) {
        std::cerr << "Failed to send UDP data" << std::endl;
        return false;
    }
    
    return true;
}

void UdpComm::setMessageCallback(MessageCallback callback) {
    messageCallback_ = callback;
}

bool UdpComm::isRunning() const {
    return running_;
}

bool UdpComm::initSocket() {
    // 创建UDP socket
    socketFd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketFd_ < 0) {
        std::cerr << "Failed to create UDP socket" << std::endl;
        return false;
    }
    
    // 绑定到本地端口
    if (bind(socketFd_, (struct sockaddr*)&localAddr_, sizeof(localAddr_)) < 0) {
        std::cerr << "Failed to bind UDP socket to port " << localPort_ << std::endl;
        closeSocket();
        return false;
    }
    
    return true;
}

void UdpComm::closeSocket() {
    if (socketFd_ >= 0) {
#ifdef _WIN32
        closesocket(socketFd_);
#else
        close(socketFd_);
#endif
        socketFd_ = -1;
    }
}

void UdpComm::listenThread() {
    const int bufferSize = 1024;
    char buffer[bufferSize];
    
    while (running_) {
        struct sockaddr_in clientAddr;
        socklen_t clientAddrLen = sizeof(clientAddr);
        
        // 接收数据
        int recvBytes = recvfrom(socketFd_, buffer, bufferSize - 1, 0,
                                (struct sockaddr*)&clientAddr, &clientAddrLen);
        
        if (recvBytes > 0 && running_) {
            buffer[recvBytes] = '\0'; // 确保字符串结束
            
            // 获取客户端信息
            std::string clientIp = inet_ntoa(clientAddr.sin_addr);
            int clientPort = ntohs(clientAddr.sin_port);
            
            // 调用回调函数
            if (messageCallback_) {
                messageCallback_(std::string(buffer), clientIp, clientPort);
            }
        } else if (recvBytes < 0 && running_) {
            // 检查是否是中断错误
#ifdef _WIN32
            if (WSAGetLastError() != WSAEINTR) {
                std::cerr << "UDP receive error" << std::endl;
                break;
            }
#else
            if (errno != EINTR) {
                std::cerr << "UDP receive error" << std::endl;
                break;
            }
#endif
        }
    }
}