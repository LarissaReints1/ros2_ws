# camera_pkg

## Overview

This package provides a ROS2 node for capturing camera images using OpenCV and GStreamer. It is optimized for NVIDIA Jetson platforms and supports publishing `sensor_msgs/Image` and `sensor_msgs/CameraInfo` topics. The package also integrates with ROS2 camera calibration tools.

---

## Features

- USB camera support via GStreamer pipeline
- Configurable resolution and frame rate
- Publishes image and camera_info topics
- Compatible with standard ROS2 calibration tools

---

## Requirements 
- OS: Jetpack 6.2
- ROS2 Humble 

---

## Topics

- `/camera/image` (sensor_msgs/Image)
- `/camera/camera_info` (sensor_msgs/CameraInfo)

---

## Parameters

| Name             | Description                          | Default              |
| ---------------- | ------------------------------------ | -------------------- |
| device           | Camera device index (e.g. "0")       | "0"                  |
| width            | Output image width in pixels         | 640                  |
| height           | Output image height in pixels        | 480                  |
| fps              | Target frame rate                    | 25                   |
| camera_info_file | Calibration file in config directory | camera_info_640.yaml |
| camera_name      | Camera identifier                    | rx0_camera           |
| frame_id         | TF frame ID                          | camera_link          |
| flip_vertical    | Flip image vertically                | false                |
| flip_horizontal  | Flip image horizontally              | false                |
| camera_stream    | Use network stream instead of device | false                |

---

## Debug / Test

List available video devices:

```bash
v4l2-ctl --list-devices
```

List supported formates and resolutions
````bash
v4l2-ctl -d /dev/video0 --list-formats-ext
````

Test camera capture
````bash
gst-launch-1.0 v4l2src device=/dev/video0 ! image/jpeg ! jpegdec ! autovideosink
````

## Launch files:

### Calibration

- Launch the calibration GUI, fill in the parameters in the launch file
  ```bash
  ros2 launch camera_pkg calibration.launch.py
  ```


### Camera Node

Run the node:

```bash
ros2 launch camera_pkg camera.launch.py \
```

