add_library(lqr_controller STATIC lqr_controller.cpp)
target_include_directories(lqr_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
