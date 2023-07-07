#include "frame_fetcher_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <functional>

namespace camera_node
{
using namespace std::placeholders;
void FrameFetcherNode::frame_fetcher_callback(const sensor_msgs::msg::Image &received_image)
{
    cv::Mat image = cv::Mat(
        received_image.height,
        received_image.width,
        extract_encoding(received_image.encoding),
        const_cast<uint8_t *>(received_image.data.data()),
        received_image.step);
    cv::imshow(this->get_name(), image);
    cv::waitKey(1);
}

int FrameFetcherNode::extract_encoding(const std::string &encoding)
{
    std::string prefix = encoding.substr(0, encoding.find('C'));
    int channels = std::stoi(encoding.substr(encoding.find('C') + 1));
    return CV_MAKETYPE(encoding_map[prefix], channels);
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
