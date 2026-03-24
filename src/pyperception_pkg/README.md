# Pyperception_pkg
Perception pipeline for Jetson Nano. Fusing Monocular Metric Depth Estimation with Multi Object Tracking to detect targets and localize in 3D space. The system is optimized with TensorRT 10.3.

---

## Features:
- Real time depth estimation: with Depth Anything v2 small 
- Multi object tracking through hsv based centroid tracking holding IDS 
- Target fusion 
- Target smoothing Alpha-beta filtering to reduce jitter 
- Change minimal area to be detected 

---

## Requirements 
- OS: Jetpack 6.2
- TensorRT 10.3
- ONNX Runtime GPU 
- CV bridge 
- Cuda 12.6 
- ROS2 Humble 

---
## Topics

| Name             | Description                          | Default              |
| ---------------- | ------------------------------------ | -------------------- |
/camera/image
/camera/camera_info
/perception/depthmap
/perception/image 
/perception/tracked_pixel
/perception/tracked_pose 

## Parameters
Alpha
Min-area
max_dissapeared
depth_max dist
depth_min_dist 


## Nodes 
### Depth estimation
Lower FPS 
/perception/depthmap


### Multi Object Tracking
Keep ID go for target first in order and highest confidence 
/perception/image
fusion with camera info
Multipje objects 


---

## Launch nodes 

ros2 launch ppyperception_pkg perception.launch.py 

---

## Tensor RT optimizations
The first time the node runs it takes 5 minutes to set up. 
It stores cache . clear cache if parameters are changed 
Need onnx model in models folder 

## Benchmarks
Tested with Depth Anything V2 (ViT-Small) @ 308x308 resolution:

| Topic | Average Rate | Latency (min/max) |
| :--- | :--- | :--- |
| `/camera/image` | 21.9 Hz | 38ms / 280ms |
| `/depth/visual` | 23.6 Hz | 37ms / 86ms |



## Verify installations: 
Tensor RT:
dpkg -l | grep nvinfer

 python3 -c "import tensorrt as trt; print(trt.__version__)"
10.3.0

https://docs.ultralytics.com/guides/nvidia-jetson/#nvidia-jetson-orin-nano-super-developer-kit



For ONNX - GPU 
https://pypi.jetson-ai-lab.io/jp6/cu126
install wheel for gpu support 

Check onnxruntime-gpu 


https://developer.nvidia.com/embedded/jetpack-sdk-62