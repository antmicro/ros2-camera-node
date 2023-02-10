#include "frames_fetcher_node.hpp"

namespace camera_node
{
FrameFetcherNode::FrameFetcherNode() : Node()
{
    frame_fetcher_sub = this->create_subscription<std_msgs::msg::String>(
        "camera_frame",
        1,
        std::bind(&FrameFetcherNode::frame_fetcher_callback, this, _1));
}

void FrameFetcherNode::frame_fetcher_callback(const sensor_msgs::msg::Image &received_image) const
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception &e) 
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::imshow(this.getName(), cv_ptr->image);
}

} // namespace camera_node

RCLCPP_COMPONENTS_REGISTER_NODE(camera_node::FrameFetcherNode)