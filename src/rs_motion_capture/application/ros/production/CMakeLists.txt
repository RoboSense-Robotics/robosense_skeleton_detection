# ROS1 production 库 - 订阅 Image/PointCloud2，向 Pipeline 提供数据
cmake_minimum_required(VERSION 3.15)

set(CUR_LIB ros_production)
project(${CUR_LIB})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-std=c++17)
add_compile_options(-W -std=c++17)
add_definitions(-DDEBUG_LEVEL=0)

# ROS1 依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  pcl_conversions
  pcl_ros
  std_msgs
)
find_package(spdlog REQUIRED)

# 源文件
set(PRODUCTION_SRCS
  src/ros_production.cpp
)

add_library(${CUR_LIB} SHARED ${PRODUCTION_SRCS})

target_include_directories(${CUR_LIB}
  PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${CUR_LIB}
  motion_capture
  spdlog::spdlog
  ${catkin_LIBRARIES}
)

set(${CUR_LIB} ${CUR_LIB} PARENT_SCOPE)
