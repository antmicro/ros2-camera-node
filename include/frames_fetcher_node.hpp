#pragma once

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.h>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

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
    FrameFetcherNode();
    /**
     * @brief Destructor for FrameFetcherNode.
     *
     */
    ~FrameFetcherNode();
};

} // namespace camera_node
