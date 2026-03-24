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

## Topics

- `/camera/image` (sensor_msgs/Image)
- `/camera/camera_info` (sensor_msgs/CameraInfo)

---

## Parameters

| Name             | Description                          | Default              |
| ---------------- | ------------------------------------ | -------------------- |
| stream_url       | Camera device index (e.g. "0")       | "0"                  |
| width            | Output image width in pixels         | 640                  |
| height           | Output image height in pixels        | 480                  |
| fps              | Target frame rate                    | 30                   |
| camera_info_file | Calibration file in config directory | camera_info_640.yaml |
| camera_name      | Camera identifier                    | rx0_camera           |
| frame_id         | TF frame ID                          | camera_link          |
| flip_vertical    | Flip image vertically                | false                |
| flip_horizontal  | Flip image horizontally              | false                |
| camera_stream    | Use network stream instead of device | false                |

---

## Debug / Test

- List available video devices:

```bash
v4l2-ctl --list-devices
```

## Launch files:

### Calibration

- Launch the calibration GUI, fill in the parameters in the launch file
  ```bash
  ros2 launch camera_pkg calibration.launch.py
  ```

````

## Run

Run the node:

```bash
ros2 launch camera_pkg camera_jetson.launch.py \
```

Inspect the FPS topic:
ros2 topic echo /camera/image
````
