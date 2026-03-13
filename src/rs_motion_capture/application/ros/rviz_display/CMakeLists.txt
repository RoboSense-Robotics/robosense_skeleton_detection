# ROS1 rviz_display 库 - 发布 MarkerArray/Image 到 RViz
cmake_minimum_required(VERSION 3.15)

set(CUR_LIB ros_rviz_display)
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
  visualization_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  pcl_conversions
  pcl_ros
)
find_package(spdlog REQUIRED)

# 源文件
set(RVIZ_DISPLAY_SRCS
  src/ros_rviz_display.cpp
)

add_library(${CUR_LIB} SHARED ${RVIZ_DISPLAY_SRCS})

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
