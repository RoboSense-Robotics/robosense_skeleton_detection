cmake_minimum_required(VERSION 3.15)
project(rs_motion_capture)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

add_definitions(-DDEBUG_LEVEL=0)
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
add_definitions(-DCMAKE_BUILD)
add_definitions(-DMODULE_NAME="RS_MOTION_CAPTURE")

# 检测构建环境：ROS1 (catkin) 或 ROS2 (ament)
find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(catkin_FOUND)
  set(ROS_VERSION 1)
elseif(ament_cmake_FOUND)
  set(ROS_VERSION 2)
else()
  message(FATAL_ERROR "Can not detect ROS version automatically")
endif()
message(STATUS "Building for ROS${ROS_VERSION}")

# 公共依赖与子目录（ROS 无关）
find_package(spdlog REQUIRED)
find_package(cv_bridge REQUIRED)
add_subdirectory(third_party/rally/rally_utils)
add_subdirectory(third_party/rally/rally_core)
add_subdirectory(third_party/rally/rally_common)
add_subdirectory(third_party/rs_inference)
add_subdirectory(motion_capture)

if(ROS_VERSION EQUAL 1)
  #========================
  # ROS1 构建 (catkin_make)
  #========================
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    visualization_msgs
    cv_bridge
    pcl_conversions
    pcl_ros
    std_msgs
    geometry_msgs
  )
  # catkin_package 必须在 add_executable 之前调用，以设置 CMAKE_RUNTIME_OUTPUT_DIRECTORY
  catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs visualization_msgs cv_bridge
                  pcl_conversions pcl_ros std_msgs geometry_msgs
  )
  add_subdirectory(application/ros)

  add_executable(motion_capture_node_ros1 node/motion_capture_node_ros1.cpp)
  add_dependencies(motion_capture_node_ros1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(motion_capture_node_ros1
    ros_production
    ros_rviz_display
    spdlog::spdlog
  )

  set(INSTALL_LIBS
    ros_production
    ros_rviz_display
    motion_capture
    rs_inference
    rally_utils
    rally_core
    rally_common
  )
  install(TARGETS ${INSTALL_LIBS} LIBRARY DESTINATION lib)
  install(TARGETS motion_capture_node_ros1 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
  install(
    PROGRAMS
    rs_motion_capture/calib_infrared_tracker.py
    rs_motion_capture/calib_infrared_tracker_v2.py
    rs_motion_capture/survive_cli.py
    rs_motion_capture/quest3_teleop_node.py
    rs_motion_capture/quest3_glove_calib.py
    rs_motion_capture/quest3_glove_teleop_node.py
    DESTINATION lib/${PROJECT_NAME}
  )
  install(
    DIRECTORY rs_motion_capture/quest3
    DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

else()
  #========================
  # ROS2 构建 (colcon build)
  #========================
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  add_subdirectory(application/ros2)

  add_executable(motion_capture_node node/motion_capture_node.cpp)
  target_link_libraries(motion_capture_node
    ros2_production
    rviz_display
    spdlog::spdlog
  )

  set(INSTALL_LIBS
    ros2_production
    rviz_display
    motion_capture
    rs_inference
    rally_utils
    rally_core
    rally_common
  )
  install(TARGETS ${INSTALL_LIBS} LIBRARY DESTINATION lib)
  install(TARGETS motion_capture_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
  install(
    PROGRAMS
    rs_motion_capture/calib_infrared_tracker.py
    rs_motion_capture/calib_infrared_tracker_v2.py
    rs_motion_capture/survive_cli.py
    rs_motion_capture/quest3_teleop_node.py
    rs_motion_capture/quest3_glove_calib.py
    rs_motion_capture/quest3_glove_teleop_node.py
    DESTINATION lib/${PROJECT_NAME}
  )
  install(
    DIRECTORY rs_motion_capture/quest3
    DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

  ament_package()
endif()
