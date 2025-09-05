cmake_minimum_required(VERSION 3.10)

# 库项目名称
project(SProbotBase)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找线程库
find_package(Threads REQUIRED)

# 创建静态库
add_library(${PROJECT_NAME} STATIC
    SProbotBase.cpp
)

# 设置库的包含目录
target_include_directories(${PROJECT_NAME} 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}  # 使得其他项目可以找到头文件
)

# 链接线程库
target_link_libraries(${PROJECT_NAME} 
    PRIVATE 
        Threads::Threads
        pid_controller 
        IMU948
        # UdpComm
        SCServo
)

# 如果需要链接其他系统库（如串口相关）
# target_link_libraries(${PROJECT_NAME} PRIVATE rt)