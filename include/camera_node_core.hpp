/*
 * Copyright (c) 2022-2024 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "camera_node/srv/camera_get_properties.hpp"
#include "camera_node/srv/camera_get_property_details.hpp"
#include "grabthecam/cameracapture.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

namespace camera_node
{

/**
 * ROS 2 Composite node that can interacts with video4linux2 compatible cameras.
 * This node is capable of querying and setting available controls with the usage of services.
 * This node can also stream data from a camera to a topic.
 *
 * * Available topics include:
 * - camera_frame : Stream of frames from a camera in CameraFrame message format.
 *
 * * Available services include:
 * - camera_get_properties : Query available controls for underlying camera.
 * - camera_get_property_details : Query details for given property by index.
 *
 * * This node also exposes following internal parameters:
 * - camera_path : Path to a video device. Defaults to "/dev/video0".
 * - camera_frame_dim : Target dimensions [width, height] of the frame to be streamed by a camera.
 * - camera_refresh_rate : Frequency at which this node will try to grab new frames from camera.
 * - camera_info_rate : Frequency at which this node will log information about the framerate.
 * - camera_driver_{...} : Camera parameters from driver, queried from driver by the node at the start.
 */
class CameraNode : public rclcpp::Node
{
    /// @brief Camera instance.
    std::unique_ptr<grabthecam::CameraCapture> camera;
    /// @brief Mapping of transformed parameter names to v4l2 parameter indices.
    std::map<std::string, int32_t> parameters_name_to_index;
    /// @brief Mapping of cv::Mat types to sensor_msgs image encodings.
    std::map<int, std::string> cvmat_type_to_str;
    /// @brief Message counter for camera_frame topic. Used in measuring topic message frequency.
    uint32_t camera_frame_counter;
    /// @brief Helper time_point for measuring camera_frame topic message frequency.
    std::chrono::time_point<std::chrono::steady_clock> camera_frame_time_point;
    /// @brief Handle for setting Node's internal parameters.
    OnSetParametersCallbackHandle::SharedPtr parameters_set_handle;
    /// @brief Endpoint for camera_get_properties service.
    rclcpp::Service<camera_node::srv::CameraGetProperties>::SharedPtr camera_get_properties_srv;
    /// @brief Endpoint for camera_get_property_details service.
    rclcpp::Service<camera_node::srv::CameraGetPropertyDetails>::SharedPtr camera_get_property_details_srv;
    /// @brief Publisher for camera_frame topic. This topic uses sensor_msgs/msg/Image as its template.
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_frame_pub;
    /// @brief Loopback subscriber for camera_frame topic.
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_frame_loopback;
    /// @brief Clock that imposes frequency at which the Node will publish new frames.
    rclcpp::TimerBase::SharedPtr camera_frame_clock;
    /// @brief Clock that imposes frequency at which the Node will log framerate..
    rclcpp::TimerBase::SharedPtr camera_frame_info_clock;

    /**
     * @brief Callback for setting Node's internal parameters.
     *
     * @param parameters Vector of parameters that underwent change.
     * @return Whether the change was successful and status.
     */
    rcl_interfaces::msg::SetParametersResult parameters_set_callback(const std::vector<rclcpp::Parameter> &parameters);

    /**
     * @brief Callback for camera_get_properties service.
     *
     * @param request Request body for this call.
     * @param response Response body for this call.
     */
    void camera_get_properties_callback(
        const camera_node::srv::CameraGetProperties::Request::SharedPtr request,
        camera_node::srv::CameraGetProperties::Response::SharedPtr response);

    /**
     * @brief Callback for camera_get_property_details service.
     *
     * @param request Request body for this call.
     * @param response Response body for this call.
     */
    void camera_get_property_details_callback(
        const camera_node::srv::CameraGetPropertyDetails::Request::SharedPtr request,
        camera_node::srv::CameraGetPropertyDetails::Response::SharedPtr response);

    /**
     * @brief A callback method that is to be executed every time this Node tries to post to a CameraFrame topic.
     *
     */
    void camera_frame_callback();

    /**
     * @brief Callback for camera_frame subscriber callback.
     *
     * @param message Image from a camera.
     */
    void camera_frame_loopback_callback(const sensor_msgs::msg::Image &message);

    /**
     * @brief Callback for printing camera framerate.
     *
     */
    void camera_frame_info_callback();

    /**
     * @brief A method that queries driver for available camera controls and assigns them to Node's parameters.
     *
     */
    void prepare_driver_parameters();

    /**
     * @brief Generates mapping for cv::Mat types.
     *
     */
    void generate_mat_mappings();

public:
    /**
     * @brief Constructs a new CameraNode object.
     *
     * @param Options Internal parameters used by every Node's derivative.
     */
    CameraNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Destroy the Camera Node object.
     *
     */
    ~CameraNode();
};
} // namespace camera_node
