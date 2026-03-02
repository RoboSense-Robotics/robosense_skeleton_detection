//
// Created by sti on 2025/6/6.
//

#ifndef POSE_DETECTION_MACROS_H
#define POSE_DETECTION_MACROS_H

#define OPENCV_VERSION_AT_LEAST(major, minor, subminor) \
    (CV_MAJOR_VERSION > (major) || \
    (CV_MAJOR_VERSION == (major) && CV_MINOR_VERSION > (minor)) || \
    (CV_MAJOR_VERSION == (major) && CV_MINOR_VERSION == (minor) && CV_SUBMINOR_VERSION >= (subminor)))


#define CUDA_CHECK(call) \
do { \
    cudaError_t err = (call); \
    if (err != cudaSuccess) { \
        fprintf(stderr, "[CUDA ERROR] %s:%d | %s\n", __FILE__, __LINE__, cudaGetErrorString(err)); \
        exit(EXIT_FAILURE); \
    } \
} while(0)

#endif //POSE_DETECTION_MACROS_H
