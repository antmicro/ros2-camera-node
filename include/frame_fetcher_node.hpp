#pragma once

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>

namespace camera_node
{

/**
 * ROS 2 Composite node.
 * This node is capable of displaying images received from camera_frame topic using OpenCV.
 *
 * * Available topics include:
 * - camera_frame : Stream of frames from a camera in CameraFrame message format.
 */
class FrameFetcherNode : public rclcpp::Node
{
    /**
     * @brief A callback method that is to be executed every time this Node receive
     * something that is posted to a CameraFrame topic.
     */
    void frame_fetcher_callback(const sensor_msgs::msg::Image &message) const;

    /// @brief Subscriber for camera_frame topic.
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frame_fetcher_sub;

public:
    /**
     * @brief Constructor for FrameFetcherNode.
     */
    FrameFetcherNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Destructor for FrameFetcherNode.
     *
     * @param Options Internal parameters used by every Node's derivative.
     */
    ~FrameFetcherNode();
};
} // namespace camera_node
