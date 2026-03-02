**注意**
<!-- 本库中自带 tensorrt 8.5.2 版本，如果上层应用依赖于此版本的库文件，则需要将其路径加入到LD_LIBRARY_PATH 中 -->
请检查正确版本的 TensorRT 是否在 LD_LIBRARY_PATH 中, 如果不在，
``` shell
export LD_LIBRARY_PATH=YOUR_PATH_TO_TNSORRT/lib:$LD_LIBRARY_PATH
```

例如:

``` shell
export LD_LIBRARY_PATH=/usr/local/tensorrt/TensorRT8.5.2.2/lib:$LD_LIBRARY_PATH
```

**Buffer 说明:**

**DiscreteMirroredBuffer**： 最常见的CPU内存和GPU显存使用方式，HostBuffer 和 Device Buffer 分别指向不同的设备内存地址，**默认使用此Buffer类**。

**GPUDeviceBuffer**： HostBuffer 和 DeviceBuffer 指向 GPU 显存的地址（目前只支持网络输入申请，输出为DiscreteMirroredBuffer），开启方式：初始化推理引擎时，InferOptions 的 use_gpu_input 方法设为 true。

**UnifiedMirroredBuffer**： 使用 Nvidia 的 Unified Memory 特性实现的统一内存和显存地址，在不同平台有不同表现，比如在x86平台，分别指向内存和显存地址，只是在使用阶段隐藏了实现。开启方式 ：初始化推理引擎时，InferOptions 的 use_managed 方法设为true。

**UnifiedAddressBuffer**：针对 Orin 平台的 Buffer 构建方式，CPU 申请的内存地址交给 GPU 使用。**推荐 Orin平台使用此Buffer**，开启方式：初始化推理引擎时，InferOptions的 use_Unified_Address 方法设为 true。