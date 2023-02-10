#pragma once

#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <functional>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.h>

using std::placeholders::_1;

namespace camera_node
{

class FrameFetcherNode : public rclcpp::Node
{
  public:
    /**
     * @brief Constructor for frameFetcherNode.
     *
     */
    FrameFetcherNode();

  private:
    void frame_fetcher_callback(const sensor_msgs::msg::Image &message) const;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frame_fetcher_sub;
}

} // namespace camera_node
