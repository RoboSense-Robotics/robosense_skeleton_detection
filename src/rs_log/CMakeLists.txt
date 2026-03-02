cmake_minimum_required(VERSION 3.5)
project(rs_log)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Windows supplies macros for min and max by default. We should only use min and max from stl
if(WIN32)
  add_definitions(-DNOMINMAX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(python_cmake_module REQUIRED)
# For python interface
find_package(pybind11 CONFIG REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
# find_package(rcl REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rcutils REQUIRED)
# find_package(rmw REQUIRED)

_ament_cmake_python_register_environment_hook()

# ZSTD 
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64)$")
  set(ZSTD_LIB "/usr/lib/aarch64-linux-gnu/libzstd.so")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|i686)$")
  set(ZSTD_LIB "/usr/lib/x86_64-linux-gnu/libzstd.so")
endif() 


# rs_log:  
set(PROJECT_SOURCES "")
list(APPEND PROJECT_SOURCES src/logger/async_logger.cc)
list(APPEND PROJECT_SOURCES src/logger/log_file_object.cc)
list(APPEND PROJECT_SOURCES src/logger/logger_util.cc)
list(APPEND PROJECT_SOURCES src/glog_wrapper/glog_wrapper.cpp)
list(APPEND PROJECT_SOURCES src/init.cc)
list(APPEND PROJECT_SOURCES src/state.cc)


add_library(${PROJECT_NAME} SHARED  ${PROJECT_SOURCES})

target_include_directories(${PROJECT_NAME} 
                           PUBLIC 
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)
# ----------------------------------------------------------------------
# ament_target_dependencies(${PROJECT_NAME}
#     rcl
#     rclcpp
#     rcutils
#     rmw
#   )
  
target_link_libraries(${PROJECT_NAME}
    ${ZSTD_LIB}
    glog 
    gflags
  )

# Build python interface
pybind11_add_module(${PROJECT_NAME}_py src/pybinding.cc)
set_target_properties(${PROJECT_NAME}_py PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_py PUBLIC include)
target_link_libraries(${PROJECT_NAME}_py PUBLIC ${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME}_py PRIVATE MODULE_NAME="RS_LOG_PYHTON")

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(NOT DEFINED PYTHON_INSTALL_DIR)
  set(PYTHON_INSTALL_DIR "local/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")
endif()

install(
  TARGETS ${PROJECT_NAME}_py
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}"
  RUNTIME DESTINATION bin
)

# Export old-style CMake variables
ament_export_include_directories("include/")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

ament_package()
