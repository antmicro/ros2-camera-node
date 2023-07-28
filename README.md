# ROS 2 CameraNode

Copyright (c) 2022-2023 [Antmicro](https://www.antmicro.com)

`CameraNode` is a ROS 2 node for streaming frames from V4L2 cameras.
It provides parameters to manipulate camera properties and exposes a service to query and modify available properties.

## CameraNode parameters

`CameraNode` offers the following set of camera configuration parameters:

* `camera_path` - Path to a video device.
* `camera_frame_dim` - Target dimensions [width, height] of the frame to be streamed by the camera.
* `camera_refresh_rate` - Frequency at which this node will try to grab new frames from camera.
* `camera_info_rate` - Frequency at which this node will log information about the frame rate.
* `camera_driver_{...}` - Camera parameters queried from driver by the node at the start.

## Dependencies

* [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) environment - follow [installation instructions](https://docs.ros.org/en/humble/Installation.html) for your system
* [OpenCV](https://github.com/opencv/opencv) - can be installed with package manager
* [grabthecam](https://github.com/antmicro/grabthecam) - A C++ library for controlling V4L2 cameras and capturing frames.
  When not found in the filesystem, it will be downloaded and built automatically.

## Building the CameraNode

`CameraNode` uses the `colcon` build system to build the project.

Follow the steps below to build the project:

* Clone the repository and go to the newly created directory:
  ```bash
  git clone https://github.com/antmicro/ros2-camera-node.git
  cd ros2-camera-node/
  ```
* Launch the configuration script (e.g. `setup.sh`) from ROS 2 to prepare the environment for building ROS 2 packages.
  Assuming the ROS 2 was installed in the `/opt/ros` directory, run:
  ```
  source /opt/ros/setup.sh
  ```
* Build the project:
  ```bash
  colcon build
  ```

Then, the built packages can be found in the `install/` directory (placed in the current directory).

The `CameraNode` is built as [composable node](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html), registered as `camera_node::CameraNode` and available under `libcamera_node_component.so` in the `install/camera_node/lib` directory.

## Running the CameraNode

To run the `camera_node::CameraNode`, a ROS 2 component container is required.
The easiest way to start the composable nodes and their components is to use ROS2 launch files.
For more details, check [Using ROS 2 launch to launch composable nodes](https://docs.ros.org/en/humble/How-To-Guides/Launching-composable-nodes.html).

A sample launch file for this node can be found under: [launch/camera_node_demo.py](launch/camera_node_demo.py).

To run this sample launch file, first update the ROS 2 environment with the built `setup.sh` script from the `install/` directory:

```bash
source install/setup.sh
```

And run:

```bash
ros2 launch camera_node camera_node_demo.py
```

This will start:

* `camera_node::CameraNode` component node in the `camera_node_container`.
* A helper `camera_node::FrameFetcherNode` component node in the `frame_fetcher_container` - it is a simple node displaying frames from `CameraNode` using OpenCV's `imshow` method.

To see available arguments for the launch script, run:

```bash
ros2 launch camera_node camera_node_demo.py --show-arguments
```

To, e.g., change the path to the camera device, use the `camera_path:=<new-path>` argument, i.e.:

```bash
ros2 launch camera_node camera_node_demo.py camera_path:=/dev/video1
```

## Configuring a camera using CameraNode

This implementation leverages the V4L2 API in the [grabthecam](https://github.com/antmicro/grabthecam) library and [ROS 2 parameters](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html) to introduce all camera settings to the [ROS 2 Service](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html) ecosystem.

All parameters of any node can be listed, read and modified using ROS 2 services, such as `describe_parameters`, `get_parameter_types`, `get_parameters`, `set_parameters`, `list_parameters`.
They are created for every created node.

To check parameter-related services, run the `CameraNode` as previously:

```bash
ros2 launch camera_node camera_node_demo.py
```

and then (in a separate terminal, with the environment loaded from `install/setup.sh`) run:

```bash
ros2 service list
```

This, among others, should list the following services:

* `/camera_node/describe_parameters` - provides descriptions for parameter names provided in the request
* `/camera_node/get_parameter_types` - provides types for parameter names provided in the request
* `/camera_node/get_parameters` - provides values for parameters listed in the request
* `/camera_node/list_parameters` - lists available parameters
* `/camera_node/set_parameters` - sets values for parameters listed in the request
* `/camera_node/set_parameters_atomically` - sets values for parameters listed in the request, atomically.


The above services can be accessed through:

* ROS 2 Python and C++ API -they can be addressed as regular services, with the help of [rcl_interfaces library](https://github.com/ros2/rcl_interfaces/tree/humble/rcl_interfaces).
* via ROS 2 CLI tools, such as `ros2 param`, or `ros2 service`

The `ros2 param` subcommand allows to e.g. `get`, `set`, `list` parameters of a given node.

For `CameraNode`, the command:

```bash
ros2 param list --param-type
```

Will display, e.g.:

```
/camera_node:
  camera_driver_auto_exposure (type: integer)
  camera_driver_backlight_compensation (type: integer)
  camera_driver_brightness (type: integer)
  camera_driver_contrast (type: integer)
  camera_driver_exposure_dynamic_framerate (type: integer)
  camera_driver_exposure_time_absolute (type: integer)
  camera_driver_focus_absolute (type: integer)
  camera_driver_focus_automatic_continuous (type: integer)
  camera_driver_gain (type: integer)
  camera_driver_pan_absolute (type: integer)
  camera_driver_power_line_frequency (type: integer)
  camera_driver_saturation (type: integer)
  camera_driver_sharpness (type: integer)
  camera_driver_tilt_absolute (type: integer)
  camera_driver_white_balance_automatic (type: integer)
  camera_driver_white_balance_temperature (type: integer)
  camera_driver_zoom_absolute (type: integer)
  camera_frame_dim (type: integer array)
  camera_info_rate (type: double)
  camera_path (type: string)
  camera_refresh_rate (type: double)
  qos_overrides./parameter_events.publisher.depth (type: integer)
  qos_overrides./parameter_events.publisher.durability (type: string)
  qos_overrides./parameter_events.publisher.history (type: string)
  qos_overrides./parameter_events.publisher.reliability (type: string)
  use_sim_time (type: boolean)
```

The parameters starting with `camera_driver_` are formed from driver-provided settings specific to the camera.
