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

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(spdlog REQUIRED)
find_package(cv_bridge REQUIRED)
add_subdirectory(third_party/rally/rally_utils)
add_subdirectory(third_party/rally/rally_core)
add_subdirectory(third_party/rally/rally_common)
add_subdirectory(third_party/rs_inference)
add_subdirectory(motion_capture)
add_subdirectory(application/ros2)

#========================
# ros2 node
#========================
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
install(
    TARGETS ${INSTALL_LIBS}
    LIBRARY DESTINATION lib
)

install(
    TARGETS  motion_capture_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

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
  DIRECTORY
  rs_motion_capture/quest3
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


