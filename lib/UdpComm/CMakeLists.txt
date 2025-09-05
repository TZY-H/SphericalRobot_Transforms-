cmake_minimum_required(VERSION 3.10)

# 库项目名称
project(UdpComm)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找线程库
find_package(Threads REQUIRED)

# 创建静态库
add_library(${PROJECT_NAME} STATIC
    UdpComm.cpp
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
)
