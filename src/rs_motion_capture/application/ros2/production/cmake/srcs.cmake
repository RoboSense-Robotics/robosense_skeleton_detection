#========================
# libs
#========================

set(CUR_SRCS "")
set(CUR_INCLUDES "include")

set(CUR_SUB_DIR "")
LIST(APPEND CUR_SUB_DIR include)
LIST(APPEND CUR_SUB_DIR src)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)

#set(robosense_msgs_DIR "/mnt/1T/msg_ws/install/robosense_msgs/share/robosense_msgs/cmake")
find_package(robosense_msgs REQUIRED)
find_package(spdlog REQUIRED)

foreach (dir ${CUR_SUB_DIR})
    file(GLOB_RECURSE tmp_srcs ${dir}/*.cpp ${dir}/*.h)
    list(APPEND CUR_SRCS ${tmp_srcs})
endforeach ()

add_library(${CUR_LIB} SHARED
        ${CUR_SRCS}
        )

target_include_directories(${CUR_LIB}
        PUBLIC
        ${CUR_INCLUDES}
        )
target_link_libraries(${CUR_LIB}
        PUBLIC
        motion_capture
        spdlog::spdlog
        ${cv_bridge_LIBRARIES}

        #        robosense_msgs__rosidl_typesupport_cpp
#        rclcpp
#        sensor_msgs
#        geometry_msgs
        )
ament_target_dependencies(${CUR_LIB} PUBLIC rclcpp sensor_msgs geometry_msgs pcl_conversions robosense_msgs cv_bridge)



