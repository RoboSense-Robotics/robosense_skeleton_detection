# ROBOSENSE SKELETON DETECTION

[ 中文版本 ](README_CN.md)

A ROS2 project for human skeleton detection based on point cloud and image data from ROBOSENSE AC2 sensor, accelerated with TensorRT. This project contains a ROS2 node that can extract human skeleton information from AC2 data in real-time and publish relevant ROS2 messages.

## System Requirements

- Ubuntu (20.04/22.04/24.04)
- ROS2 (Foxy/Humble/Jazzy)
- CUDA 12.x
- TensorRT 10.x
- C++17

## Dependencies

Before building this project, please ensure that the necessary development libraries are installed and the driver environment is correctly configured.

### 1. System Library Installation

This project requires the `spdlog` logging library. Please run the following command to install it:

```bash
sudo apt install libspdlog-dev
```

### 2. Driver Node Configuration

Requires the [robosense_ac_driver](https://github.com/RoboSense-Robotics/robosense_ac_driver) node to publish AC2 sensor data. If not compiled yet, please download and compile the driver source code first.

```bash
git clone git@github.com:RoboSense-Robotics/robosense_ac_driver.git
cd robosense_ac_driver
colcon build
source install/setup.bash
```

## Installation

### 1. Clone the Repository

```bash
git clone git@github.com:RoboSense-Robotics/robosense_skeleton_detection.git
cd robosense_skeleton_detection
```

### 2. Model Conversion

The project uses TensorRT for inference acceleration, which requires converting ONNX models to TRT models.

Before downloading models, please create the required directories:

```bash
mkdir -p src/rs_motion_capture/model/onnx
mkdir -p src/rs_motion_capture/model/x86_64
```

#### 2.1 Download ONNX Models

This repository does not include model files. Please download them first:
- [dynamic_rtmdet_s_coco_640x640_20231209.onnx](https://cdn.robosense.cn/models/skeleton_detection/dynamic_rtmdet_s_coco_640x640_20231209.onnx)
- [dynamic_end2end2_key26.onnx](https://cdn.robosense.cn/models/skeleton_detection/dynamic_end2end2_key26.onnx)

After downloading, place the files in `src/rs_motion_capture/model/onnx/`

The directory structure should look like:
```
src/rs_motion_capture/model/onnx/
├── dynamic_rtmdet_s_coco_640x640_20231209.onnx  # Object detection model (Stage 1)
└── dynamic_end2end2_key26.onnx                  # Pose estimation model (Stage 2)
```

#### 2.2 Convert Models with TensorRT

**Download TensorRT Package (Optional)**

If you haven't installed TensorRT yet, you can try using the package provided by this repository, which includes the trtexec tool for model conversion and related library files. The TensorRT provided by this repository is a portable version that can be used directly after extraction.

👉 Download Link  
[TensorRT-10.8.0.43.Linux.x86_64-gnu.cuda-12.8.tar.gz](https://cdn.robosense.cn/third_party/TensorRT-10.8.0.43.Linux.x86_64-gnu.cuda-12.8.tar.gz)

After downloading, extract the package to any directory:

```bash
tar -xvf TensorRT-10.8.0.43.Linux.x86_64-gnu.cuda-12.8.tar.gz
```

**Use trtexec Command Line Tool**

```bash
# Ensure TensorRT is properly installed and environment variables are set
export TENSORRT_DIR=<Your TensorRT Root Directory>
export PATH=$TENSORRT_DIR/bin:$PATH
export LD_LIBRARY_PATH=$TENSORRT_DIR/lib:$LD_LIBRARY_PATH

# Convert Stage1 model (Object Detection)
trtexec --onnx=src/rs_motion_capture/model/onnx/dynamic_rtmdet_s_coco_640x640_20231209.onnx --saveEngine=src/rs_motion_capture/model/x86_64/stage1.trt --fp16 --workspace=4096

# Convert Stage2 model (Pose Estimation)
trtexec --onnx=src/rs_motion_capture/model/onnx/dynamic_end2end2_key26.onnx --saveEngine=src/rs_motion_capture/model/x86_64/stage2.trt --fp16 --workspace=4096
```

#### 2.3 Model Mapping

| ONNX Model | TRT Model | Purpose |
|-----------|---------|---------|
| `dynamic_rtmdet_s_coco_640x640_20231209.onnx` | `stage1.trt` | Object Detection (Human Detection) |
| `dynamic_end2end2_key26.onnx` | `stage2.trt` | Pose Estimation (Skeleton Keypoint Detection) |

**Important Notes**
- TRT models need to be generated according to your GPU architecture; models generated for different architectures are not interchangeable
- Models for x86_64 platform are stored in `src/rs_motion_capture/model/x86_64/` directory
- Models for ARM platform (e.g., Jetson) are stored in `src/rs_motion_capture/model/aarch/` directory
- It's recommended to use the `--fp16` option for better performance
- If conversion fails, please check the compatibility between TensorRT version and ONNX models

### 3. Build

```bash
colcon build --cmake-args -DTENSORRT_RELEASE_PATH=<Your TensorRT Root Directory>
```

**Note**
- `<Your TensorRT Root Directory>` is the installation path of your local TensorRT, for example `/media/sti/DD2/TensorRT-10.8.0.43`
- Please ensure you have completed the model conversion steps before building

### 4. Configuration File Modification

The configuration file is located at `src/rs_motion_capture/config/config.yaml`. Please modify the following parameters according to your actual situation:

- `sensor.calib_path`: AC2 calibration file path

## Running

> ⚠️ **Important Notes**
> 1. **Driver Dependency**: Before running, ensure that the `robosense_ac_driver` node is already active and publishing AC2 sensor data.
> 2. **Environment Dependency (TensorRT)**: This node requires **TensorRT** for inference. You must ensure the TensorRT root directory and environment variables are properly configured before execution; otherwise, the program will fail to load the required dynamic libraries.
> 3. **Communication Check**: Ensure that the current terminal and the driver node are using the same `ROS_DOMAIN_ID` and are on the same network.

### 1. Configure Environment Variables

Before launching the node, execute the following commands (replace `<Your TensorRT Root Directory>` with your actual installation path):

```bash
export TENSORRT_DIR=<Your TensorRT Root Directory>
export PATH=$TENSORRT_DIR/bin:$PATH
export LD_LIBRARY_PATH=$TENSORRT_DIR/lib:$LD_LIBRARY_PATH
```

### 2. Launch the Node

```bash
source install/setup.bash
ros2 launch rs_motion_capture motion_capture_node_launch.py collector:=zed calib_mode:=false
```

## ROS2 Topics

### Subscribed Topics

- `/rs_camera/left/color/image_raw` - Left camera RGB image
- `/rs_camera/right/color/image_raw` - Right camera RGB image
- `/rs_lidar/points` - LiDAR point cloud

### Published Topics

- `/pose_detection/pose_markers` - 3D skeleton pose
- `/left/pd` - 2D skeleton pose (left camera image)
- `/right/pd` - 2D skeleton pose (right camera image)

## Visualization

When running this project via ros2 launch, RViz2 will automatically start for visualization. You can view in RViz2:
- Human skeleton keypoints
- 3D pose estimation results
- Point cloud data
- Camera images

![](images/27.jpg)

## Troubleshooting

### 1. TensorRT Not Found During Compilation

Ensure that the `TENSORRT_RELEASE_PATH` parameter is correctly set and TensorRT is properly installed.

```bash
# Check TensorRT installation
ls <Your TensorRT Root Directory>/lib
# You should see library files like libnvinfer.so
```

### 2. TRT Model Files Not Found at Runtime

Please ensure that you have completed the model conversion steps and that the TRT model files are in the correct directory:
- x86_64: `src/rs_motion_capture/model/x86_64/stage1.trt` and `stage2.trt`
- ARM: `src/rs_motion_capture/model/aarch/stage1.trt` and `stage2.trt`

### 3. Slow Model Inference Speed

- Ensure you used the `--fp16` option for model conversion
- Check if GPU drivers are correctly installed
- Appropriately increase the `--workspace` size (recommended 4096MB or higher)
