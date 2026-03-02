#========================
# libs
#========================

set(CUR_SRCS "")
LIST(APPEND CUR_INCLUDES "include")

set(CUR_SUB_DIR "")
LIST(APPEND CUR_SUB_DIR include)
LIST(APPEND CUR_SUB_DIR src)

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
        ${EIGEN3_INCLUDE_DIR}
        )
target_link_libraries(${CUR_LIB}
        PUBLIC
        rally_utils
        )

#=============================
# install
#=============================

if (HYPER_VISION_DEV_LIB_PATH)
    install(TARGETS ${CUR_LIB}
            LIBRARY DESTINATION ${HYPER_VISION_DEV_LIB_PATH}
            ARCHIVE DESTINATION ${HYPER_VISION_DEV_LIB_PATH}
            COMPONENT release
            )

    install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/rally/core
            DESTINATION ${HYPER_VISION_DEV_INCLUDE_PATH}/rally
            COMPONENT release
            )
endif ()
