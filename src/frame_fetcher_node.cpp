#include "frame_fetcher_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <functional>
#include <opencv2/opencv.hpp>

namespace camera_node
{
using namespace std::placeholders;
void FrameFetcherNode::frame_fetcher_callback(const sensor_msgs::msg::Image &received_image) const
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(received_image);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv::imshow(this->get_name(), cv_ptr->image);
    cv::waitKey(1);
}

FrameFetcherNode::FrameFetcherNode(const rclcpp::NodeOptions &options) : Node("frame_fetcher", options)
{
    frame_fetcher_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "camera_frame",
        1,
        std::bind(&FrameFetcherNode::frame_fetcher_callback, this, _1));
}

FrameFetcherNode::~FrameFetcherNode() {}
} // namespace camera_node

RCLCPP_COMPONENTS_REGISTER_NODE(camera_node::FrameFetcherNode)
