cmake_minimum_required(VERSION 3.10)
#========================
# Project
#========================

set(CUR_LIB rs_inference)

project(${CUR_LIB})

set(INFER_VERSION_MAJOR 0)
set(INFER_VERSION_MINOR 0)
set(INFER_VERSION_PATCH 0)
set(INFER_VERSION_STRING ${INFER_VERSION_MAJOR}.${INFER_VERSION_MINOR}.${INFER_VERSION_PATCH})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib" CACHE PATH "Library output dir.")

option(INFER_SHARED_LIB "Shared library support" ON)
option(INFER_TENSORRT "Build TensorRT inference engine" OFF)
option(INFER_INSTALL "install inference library and headers" ON)
option(INFER_TEST "Whether to build test" OFF)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

add_definitions(-std=c++14)
add_compile_options(-w)
add_compile_options(-std=c++14)
add_definitions(-DDEBUG_LEVEL=0)
add_definitions(-DCMAKE_DEV)
# add_definitions(-DCMAKE_CUDA_FLAGS="-Xcompiler -fPIC")
add_compile_options(-fPIC)

add_compile_options(-Wno-deprecated-declarations)

include(cmake/check_platform.cmake)

include(cmake/srcs.cmake)

configure_file(Doxyfile.in Doxyfile)

if (INFER_TEST)
    add_subdirectory(test)
endif()

# set(${CUR_LIB} PARENT_SCOPE)
