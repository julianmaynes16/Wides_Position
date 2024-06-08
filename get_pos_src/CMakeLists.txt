cmake_minimum_required(VERSION 3.10)

project(Get_Pos)


find_library(PIGPIO_LIBRARIES NAMES pigpio PATHS /usr/lib /usr/local/lib)

add_executable(Get_Pos get_pos.cpp)


target_link_libraries(Get_Pos /usr/local/lib/librealsense2.so.2.48.0)
target_link_libraries(Get_Pos ${PIGPIO_LIBRARIES})
