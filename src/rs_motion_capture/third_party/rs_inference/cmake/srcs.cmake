
set(CUR_SRCS "")
set(CUR_INCLUDES "include")
set(CUR_SRCS_DIR "src/inference")
set(CUR_COMMON_DIR "src/common")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#========================
# rs_cryptor part
#========================
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/rs_cryptor)

set(INFER_INCLUDE_DIRS "")
set(INFER_LIBS "")
set(INFER_LIBS_DIR "")

#========================
# infer part
#========================
if(INFER_TENSORRT)
    find_package(CUDA REQUIRED)
    find_package(TensorRT REQUIRED)
    list(APPEND INFER_INCLUDE_DIRS ${TENSORRT_INCLUDE_DIR})
    list(APPEND INFER_INCLUDE_DIRS ${CUDA_INCLUDE_DIRS})
    list(APPEND INFER_LIBS ${TENSORRT_LIBS})
    if (CUDA_VERSION VERSION_GREATER_EQUAL 11.1)
        list(APPEND INFER_LIBS ${CUDA_LIBRARIES})
    endif()
endif()

#========================
# srcs
#========================
set(COMMON_SRCS
        log.cpp
        md5.cpp
        register.cpp
        )

## TensorRT infer sources
if(INFER_TENSORRT)
    set(TENSORRT_SRCS
        trt_bindings.cpp
        trt_infer.cpp
        trt_utils.cpp
        trt_profiler.cpp
        trt_infer_impl.cpp)
endif()


set(CUR_SRCS
        inference.cpp
        ${TENSORRT_SRCS}
        )

set(SRCS "")

foreach(_file IN LISTS CUR_SRCS)
    LIST(APPEND SRCS ${CUR_SRCS_DIR}/${_file})
endforeach()

foreach (_file IN LISTS COMMON_SRCS)
    LIST(APPEND SRCS ${CUR_COMMON_DIR}/${_file})
endforeach ()

#========================
# libs
#========================
if(INFER_SHARED_LIB)
    add_library(${CUR_LIB} SHARED ${SRCS})
else()
    add_library(${CUR_LIB} STATIC ${SRCS})
endif()

target_include_directories(${CUR_LIB}
        PUBLIC
        ${INFER_INCLUDE_DIRS}
        ${CUR_INCLUDES}
        )
target_link_libraries(${CUR_LIB}
        PUBLIC
        rs_cryptor
        ${INFER_LIBS}
        )
#========================
# plugins
#========================
# set(PLUGIN_SRCS
#     layernorm/layernorm_kernel.cu
#     layernorm/layernorm.cpp
#     ppscatter/pillarScatter.cpp
#     ppscatter/pillarScatterKernels.cu
# )
# set(PLUGIN_SOURCE_FILES "")
# foreach(_file IN LISTS PLUGIN_SRCS)
#     LIST(APPEND PLUGIN_SOURCE_FILES ${CUR_SRCS_DIR}/trt_plugin/${_file})
# endforeach()
# add_library(custom_plugin SHARED ${PLUGIN_SOURCE_FILES})
# target_include_directories(custom_plugin
#     PUBLIC
#     include
#     ${CUDA_INCLUDE_DIRS}
#     ${TENSORRT_INCLUDE_DIR}
# )

# target_link_libraries(custom_plugin PUBLIC
#     ${INFER_LIBS}
# )
#add_subdirectory(src/inference/trt_plugin)
# set(NMS_PLUGIN efficient_nms_plugin)
# add_subdirectory(efficientNMSPlugin)
# add_subdirectory(common)
# CUDA_ADD_LIBRARY(${NMS_PLUGIN} SHARED ${NMS_PLUGIN_SOURCES})
# message(STATUS cmake install lib ${CMAKE_INSTALL_LIBDIR})
#=============================
# install
#=============================

# install(TARGETS ${CUR_LIB}
#         LIBRARY DESTINATION ${HYPER_VISION_DEV_LIB_PATH}
#         ARCHIVE DESTINATION ${HYPER_VISION_DEV_LIB_PATH}
#         COMPONENT release
#         )
# if(INFER_TENSORRT)
#     if (CUDA_VERSION VERSION_GREATER_EQUAL 11.1)
#         install(FILES ${TENSORRT_LIBS}
#                 DESTINATION ${HYPER_VISION_DEV_LIB_PATH}
#                 COMPONENT release)
#     endif()
# endif()