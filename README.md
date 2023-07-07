# ROS2 CameraNode

Copyright (c) 2022-2023 [Antmicro](https://www.antmicro.com)

`CaneraNode` is a ROS2 node that allows to stream frames from V4L2 cameras.
It provides parameters to manipulate camera properties and exposes a service to query available properties.

This project offers two pre-implemented nodes:
* `CameraNode`- Manipulates camera properties and streams frames to a parametrized topic.
* `FrameFetcherNode` - Displays frames using `OpenCV` from a parametrized topic.

## CameraNode parameters

`CameraNode` offers set of parameters to configure the camera:
* `camera_path` : Path to a video device.
* `camera_frame_dim` : Target dimensions [width, height] of the frame to be streamed by a camera.
* `camera_refresh_rate` : Frequency at which this node will try to grab new frames from camera.
* `camera_info_rate` : Frequency at which this node will log information about the frame rate.
* `camera_internal_{...}` : Internal camera parameters, queried from driver by the node at the start.

## Dependencies

* [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
* `OpenCV`
* [grabthecam](https://github.com/antmicro/grabthecam) - A C++ library for controlling V4L2 cameras and capturing frames.

## Building the CameraNode

`CameraNode` uses `colcon` build system to build the project.
Follow the steps below to build the project:
```
source <path_to_ros2_env>
colcon build
source install/local_setup.bash
```

Once the setup script is sourced to the environment, the sample node demonstration can be launched:
```
ros2 launch camera_node camera_node_demo.py
```
