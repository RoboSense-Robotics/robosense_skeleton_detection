set(CUR_SRCS "")
LIST(APPEND CUR_INCLUDES "include")

set(CUR_SUB_DIR "")
LIST(APPEND CUR_SUB_DIR include)
LIST(APPEND CUR_SUB_DIR src)

foreach (dir ${CUR_SUB_DIR})
    file(GLOB_RECURSE tmp_srcs ${dir}/*.cpp ${dir}/*.h ${dir}/*.cu)
    list(APPEND CUR_SRCS ${tmp_srcs})
endforeach ()

enable_language(CUDA)

find_package(Ceres REQUIRED)
find_package(rs_log REQUIRED)

add_library(${CUR_LIB} SHARED
        ${CUR_SRCS}
        )
target_include_directories(${CUR_LIB}
        PUBLIC
        ${CUR_INCLUDES}
        )
target_link_libraries(${CUR_LIB}
        PUBLIC
        rally_utils
        rally_common
        rally_core
        rs_inference
        ${CERES_LIBRARIES}
        )

ament_target_dependencies(${CUR_LIB} PUBLIC rs_log)

target_compile_options(${CUR_LIB} PRIVATE 
            "$<$<COMPILE_LANGUAGE:CUDA>:-gencode=arch=compute_80,code=sm_80>"
            "$<$<COMPILE_LANGUAGE:CUDA>:-gencode=arch=compute_86,code=sm_86>"
            "$<$<COMPILE_LANGUAGE:CUDA>:-gencode=arch=compute_87,code=sm_87>" # orin
            "$<$<COMPILE_LANGUAGE:CUDA>:-gencode=arch=compute_89,code=sm_89>"
)
