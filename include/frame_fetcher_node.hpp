#pragma once

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

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
    void frame_fetcher_callback(const sensor_msgs::msg::Image &message);

    /**
     * @brief Extracts encoding from a string.
     *
     * @param encoding String containing encoding.
     * @return int Encoding in integer format.
     */
    int extract_encoding(const std::string &encoding);

    /// @brief Subscriber for camera_frame topic.
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frame_fetcher_sub;

    /// @brief Map of encoding strings to encoding integers.
    std::map<std::string, int> encoding_map = {
        {"8U", CV_8U},
        {"8S", CV_8S},
        {"16U", CV_16U},
        {"16S", CV_16S},
        {"32S", CV_32S},
        {"32F", CV_32F},
        {"64F", CV_64F}};

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
