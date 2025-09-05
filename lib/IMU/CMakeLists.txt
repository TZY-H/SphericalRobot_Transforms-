cmake_minimum_required(VERSION 3.10)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 查找线程库
find_package(Threads REQUIRED)

add_library(IMU948 STATIC IM948_CMD.cpp)
target_include_directories(IMU948 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# 链接线程库
target_link_libraries(IMU948 
    PRIVATE 
        Threads::Threads
)