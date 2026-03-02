cmake_minimum_required(VERSION 3.15)

#========================
# Project
#========================

set(CUR_LIB ros2_production)
project(${CUR_LIB})

set(CMAKE_CXX_STANDARD 17)  # 必须设置为 17 或更高
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # 强制要求标准

set(CMAKE_BUILD_TYPE Release)
add_definitions(-std=c++17)
add_compile_options(-W)
add_compile_options(-std=c++17)
add_definitions(-DDEBUG_LEVEL=0)

include(cmake/srcs.cmake)

set(${CUR_LIB} PARENT_SCOPE)



