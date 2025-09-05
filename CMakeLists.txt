cmake_minimum_required(VERSION 3.10)

project(SphericalRobot)

set(CMAKE_CXX_STANDARD 17)


# 添加需要的子库
add_subdirectory(lib/example)
add_subdirectory(lib/FTServo)
add_subdirectory(lib/SProbotBase)
add_subdirectory(lib/IMU)
add_subdirectory(lib/pid_controller)
add_subdirectory(lib/UdpComm)
include_directories(lib)

add_executable(SphericalRobot main.cpp)
target_link_libraries(SphericalRobot 
    PRIVATE 
        SProbotBase 
        pid_controller 
        IMU948
        UdpComm
        SCServo
)

# add_executable(temp temp/examples/SMS_STS/WritePos/WritePos.cpp)
# target_link_libraries(temp PRIVATE SCServo)
