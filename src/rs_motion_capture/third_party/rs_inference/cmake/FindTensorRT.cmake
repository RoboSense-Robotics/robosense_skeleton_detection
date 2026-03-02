
# include(${PROJECT_SOURCE_DIR}/cmake/cuda/cuda.cmake)

include(FindPackageHandleStandardArgs)
unset(TensorRT_FOUND)
# set(TENSORRT_RELEASE_PATH /usr/local/TensorRT-8.5.2.2)
set(DEFAULT_TRT_VER "V8")
set(TRT_VER ${DEFAULT_TRT_VER})

set(TRT_LIB_DIR "")
if (CMAKE_SYSTEM_NAME MATCHES "Linux")
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        set(TENSORRT_RELEASE_PATH "/usr/local/TensorRT-10.7.0.23/" CACHE PATH "TensorRT root dir")
        set(TENSORRT_INCLUDE_DIR ${TENSORRT_RELEASE_PATH}/include)
        list(APPEND TRT_LIB_DIR ${TENSORRT_RELEASE_PATH}/lib)
        message(STATUS "TENSORRT_RELEASE_PATH = ${TENSORRT_RELEASE_PATH}")
        message(STATUS "TRT_LIB_DIR = ${TRT_LIB_DIR}")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        set(TENSORRT_RELEASE_PATH /lib/aarch64-linux-gnu/)
        set(TENSORRT_INCLUDE_DIR /usr/include/aarch64-linux-gnu)
        list(APPEND TRT_LIB_DIR /lib/aarch64-linux-gnu/)
    endif()
else()
    message(FATAL_ERROR "Unsupported System!")
endif()

## libraries path

set(TENSORRT_INSTALL_LIBS "")
## macro definitions
## !!important
add_definitions(-DUSE_TENSORRT_${TRT_VER})


if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    find_library(NVPARSER_LIBRARY NAMES nvonnxparser libnvonnxparser.so libnvonnxparser.so.10 PATHS ${TRT_LIB_DIR} NO_DEFAULT_PATH)
    find_library(NVINFER_LIBRARY NAMES nvinfer libnvinfer.so libnvinfer.so.10 PATHS ${TRT_LIB_DIR} NO_DEFAULT_PATH)
    find_library(NVINFER_PLUGIN_LIBRARY NAMES nvinfer_plugin libnvinfer_plugin.so libnvinfer_plugin.so.10 PATHS ${TRT_LIB_DIR} NO_DEFAULT_PATH)
    find_library(NVINFER_BUILDER_RESOURCE NAMES nvinfer_builder_resource libnvinfer_builder_resource.so libnvinfer_builder_resource.so.10 libnvinfer_builder_resource.so.10.8.0 PATHS ${TRT_LIB_DIR} NO_DEFAULT_PATH)

    list(APPEND TENSORRT_LIBS ${NVPARSER_LIBRARY} ${NVINFER_LIBRARY} ${NVINFER_PLUGIN_LIBRARY} ${NVINFER_BUILDER_RESOURCE})

    message(STATUS "NVPARSER_LIBRARY = ${NVPARSER_LIBRARY}")
    message(STATUS "NVINFER_LIBRARY = ${NVINFER_LIBRARY}")
    message(STATUS "NVINFER_PLUGIN_LIBRARY = ${NVINFER_PLUGIN_LIBRARY}")
    message(STATUS "NVINFER_BUILDER_RESOURCE = ${NVINFER_BUILDER_RESOURCE}")
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    find_library(NVPARSER_LIBRARY NAMES nvonnxparser PATHS ${TRT_LIB_DIR} NO_DEFAULT_PATH)
    find_library(NVINFER_LIBRARY NAMES nvinfer PATHS ${TRT_LIB_DIR} NO_DEFAULT_PATH)
    find_library(NVPARSER_LIBRARY NAMES libnvonnxparser.so PATHS ${TRT_LIB_DIR} NO_DEFAULT_PATH)
    find_library(NVINFER_LIBRARY NAMES libnvinfer.so PATHS ${TRT_LIB_DIR} NO_DEFAULT_PATH)
    find_library(NVINFER_PLUGIN_LIBRARY NAMES libnvinfer_plugin.so PATHS ${TRT_LIB_DIR} NO_DEFAULT_PATH)
    find_library(NVINFER_BUILDER_RESOURCE NAMES libnvinfer_builder_resource.so.10.3.0 PATHS ${TRT_LIB_DIR} NO_DEFAULT_PATH)
    list(APPEND TENSORRT_LIBS ${NVPARSER_LIBRARY}
            ${NVINFER_LIBRARY}
            ${NVINFER_PLUGIN_LIBRARY}
            ${NVINFER_BUILDER_RESOURCE})
    message(${TENSORRT_LIBS})
else()
    message(FATAL_ERROR "Unknown system processor ${CMAKE_SYSTEM_PROCESSOR}.")
endif()

message(STATUS "TensorRT libraries found: ${TENSORRT_LIBS}")

list(APPEND TENSORRT_INCLUDE_DIR ${GPU_INCLUDE_DIRS})
find_package_handle_standard_args(TensorRT DEFAULT_MSG TENSORRT_INCLUDE_DIR TENSORRT_LIBS)

# hide locals from GUI
mark_as_advanced(TENSORRT_INCLUDE_DIR)
