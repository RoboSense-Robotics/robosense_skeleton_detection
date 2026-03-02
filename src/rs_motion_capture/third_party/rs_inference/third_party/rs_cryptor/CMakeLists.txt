cmake_minimum_required(VERSION 3.5)
project(rs_cryptor)

add_compile_options(-fPIC)

#SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "Release")

## Compile as C++11
add_compile_options(-std=c++11)

#add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")


add_library(${PROJECT_NAME} STATIC
        include/rs_perception/rs_cryptor.h
        src/rs_cryptor.cpp
        )
target_include_directories(${PROJECT_NAME}
        PUBLIC
        include
        )

#set(RELEASE_LIBS
#        ${PROJECT_NAME}
#        )
#if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
#    set(RS_PERCEPTION_LIB_PATH ${PROJECT_SOURCE_DIR}/x86_64)
#endif()
#if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
#    set(RS_PERCEPTION_LIB_PATH ${PROJECT_SOURCE_DIR}/aarch64)
#endif()
#
#install(TARGETS ${RELEASE_LIBS}
#        LIBRARY DESTINATION ${RS_PERCEPTION_LIB_PATH}
#        ARCHIVE DESTINATION ${RS_PERCEPTION_LIB_PATH}
#        COMPONENT release
#        )

#use yaml
#find_package(yaml-cpp REQUIRED)

##bin
#add_executable(encrypt_bin bin/encrypt.cpp bin/common.h)
#target_link_libraries(encrypt_bin ${PROJECT_NAME} yaml-cpp)
#
##bin
#add_executable(decrypt_bin bin/decrypt.cpp bin/common.h)
#target_link_libraries(decrypt_bin ${PROJECT_NAME} yaml-cpp)


