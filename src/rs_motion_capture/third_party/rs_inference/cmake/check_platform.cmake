# Platform lists

# system - linux
# system - qnx
# x86_64 - with cuda
# x86_64 - without cuda
# aarch64 - with cuda
# aarch64 - without cuda
# aarch64 + linux - mdc610
# aarch64 + linux - tda4vm
# aarch64 + linux - horizonX3
# aarch64 + qnx qam9000 + qam8540

find_package(CUDA QUIET)
if(CUDA_FOUND)
    set(INFER_TENSORRT ON)
    return()
endif()

if(INFER_NOWARNING)
    add_compile_options(-Wno-all)
endif()