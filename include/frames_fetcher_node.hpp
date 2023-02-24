#pragma once

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>

namespace camera_node
{
class FrameFetcherNode : public rclcpp::Node
{
    void frame_fetcher_callback(const sensor_msgs::msg::Image &message) const;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frame_fetcher_sub;

  public:
    /**
     * @brief Constructor for FrameFetcherNode.
     *
     */
    FrameFetcherNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    /**
     * @brief Destructor for FrameFetcherNode.
     *
     */
    ~FrameFetcherNode();
};
} // namespace camera_node
