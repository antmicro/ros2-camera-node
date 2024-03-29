/*
 * Copyright (c) 2022-2024 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "camera_node_core.hpp"
#include <functional>
#include <regex>

namespace camera_node
{
using namespace std::placeholders;
rcl_interfaces::msg::SetParametersResult
CameraNode::parameters_set_callback(const std::vector<rclcpp::Parameter> &parameters)
{
    RCLCPP_DEBUG(get_logger(), "Reconfiguring camera");
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &parameter : parameters)
    {
        if (parameters_name_to_index.contains(parameter.get_name()))
        {
            try
            {
                camera->set<int64_t>(parameters_name_to_index[parameter.get_name()], parameter.as_int());
            }
            catch (const grabthecam::CameraException &e)
            {
                std::ostringstream oss;
                oss << "Failed to set parameter [" << parameter.get_name() << "] with value [" << parameter.as_int()
                    << "]";
                RCLCPP_ERROR(get_logger(), "%s", oss.str().c_str());
                RCLCPP_ERROR(get_logger(), "Exception: [%s]", e.what());
                result.successful = false;
                result.reason = oss.str();
                return result;
            }
        }
        else if (parameter.get_name() == "camera_frame_dim")
        {
            camera->setFormat(
                static_cast<uint32_t>(parameter.as_integer_array()[0]),
                static_cast<uint32_t>(parameter.as_integer_array()[1]));
        }
        else if (parameter.get_name() == "camera_refresh_rate")
        {
            camera_frame_clock->cancel();
            camera_frame_clock = create_wall_timer(
                std::chrono::duration<double>(1 / parameter.as_double()),
                std::bind(&CameraNode::camera_frame_callback, this));
        }
        else if (parameter.get_name() == "camera_info_rate")
        {
            camera_frame_info_clock->cancel();
            camera_frame_info_clock = create_wall_timer(
                std::chrono::duration<double>(1 / parameter.as_double()),
                std::bind(&CameraNode::camera_frame_info_callback, this));

            camera_frame_counter = 0;
            camera_frame_time_point = std::chrono::steady_clock::now();
        }
        RCLCPP_DEBUG(get_logger(), "Setting [%s]", parameter.get_name().c_str());
    }
    return result;
}

void CameraNode::camera_get_properties_callback(
    const camera_node::srv::CameraGetProperties::Request::SharedPtr request,
    camera_node::srv::CameraGetProperties::Response::SharedPtr response)
{
    for (auto property = camera_node::msg::CameraProperty(); const auto &queried_property : camera->queryProperties())
    {
        property.index = queried_property.id;
        property.name = queried_property.name;
        property.type = static_cast<uint8_t>(queried_property.type);
        property.default_value = queried_property.defaultValue;

        response->properties.push_back(property);
    }
}

void CameraNode::camera_get_property_details_callback(
    const camera_node::srv::CameraGetPropertyDetails::Request::SharedPtr request,
    camera_node::srv::CameraGetPropertyDetails::Response::SharedPtr response)
{
    auto details = camera->queryPropertyDetails(request->index);

    response->details.property.index = details.property.id;
    response->details.property.name = details.property.name;
    response->details.property.type = static_cast<uint8_t>(details.property.type);
    response->details.property.default_value = details.property.defaultValue;

    for (auto menu_entry = camera_node::msg::CameraPropertyMenuEntry(); const auto &[index, name] : details.menuEntries)
    {
        menu_entry.index = index;
        menu_entry.name = name;
        response->details.menu_entries.push_back(menu_entry);
    }
}

void CameraNode::camera_frame_callback()
{
    auto frame = camera->capture();
    auto message = sensor_msgs::msg::Image();
    message.header = std_msgs::msg::Header();
    message.encoding = cvmat_type_to_str[frame.type()];
    message.height = frame.rows;
    message.width = frame.cols;
    message.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    message.data.assign(frame.datastart, frame.dataend);

    camera_frame_pub->publish(message);
}

void CameraNode::camera_frame_loopback_callback(const sensor_msgs::msg::Image &message) { camera_frame_counter++; }

void CameraNode::camera_frame_info_callback()
{
    auto frequency =
        static_cast<double>(camera_frame_counter) /
        std::chrono::duration<double, std::ratio<1L>>(std::chrono::steady_clock::now() - camera_frame_time_point)
            .count();

    RCLCPP_INFO(get_logger(), "Camera framerate: [%f]", frequency);
    camera_frame_counter = 0;
    camera_frame_time_point = std::chrono::steady_clock::now();
}

void CameraNode::prepare_driver_parameters()
{
    const std::regex regex("[^A-z0-9\\s]");
    for (auto &&queried_property : camera->queryProperties())
    {
        rcl_interfaces::msg::ParameterDescriptor param_descriptor;
        rcl_interfaces::msg::IntegerRange int_range;
        std::stringstream result;

        std::regex_replace(
            std::ostream_iterator<char>(result),
            queried_property.name.begin(),
            queried_property.name.end(),
            regex,
            "");
        std::string name = result.str();

        std::transform(name.begin(), name.end(), name.begin(), [](auto &c) { return std::tolower(c); });
        std::replace(name.begin(), name.end(), ' ', '_');

        parameters_name_to_index["camera_driver_" + name] = queried_property.id;

        // Get current value instead of default
        camera->get<int64_t>(queried_property.id, queried_property.defaultValue, true);

        // Apply bounds to parameter
        int_range.step = queried_property.step;
        int_range.from_value = queried_property.minimum;
        int_range.to_value = queried_property.maximum;
        param_descriptor.integer_range.push_back(int_range);

        declare_parameter<int32_t>("camera_driver_" + name, queried_property.defaultValue, param_descriptor);
    }
}

void CameraNode::generate_mat_mappings()
{

    for (auto &[cvtype, prefix] :
         {std::tuple<int, std::string>{CV_8U, "8U"},
          std::tuple<int, std::string>{CV_8S, "8S"},
          std::tuple<int, std::string>{CV_16U, "16U"},
          std::tuple<int, std::string>{CV_16S, "16S"},
          std::tuple<int, std::string>{CV_32S, "32S"},
          std::tuple<int, std::string>{CV_32F, "32F"},
          std::tuple<int, std::string>{CV_64F, "64F"}})
    {
        for (int channel : {1, 2, 3, 4})
        {
            cvmat_type_to_str[CV_MAKETYPE(cvtype, (channel))] = prefix + "C" + std::to_string(channel);
        }
    }
}

CameraNode::CameraNode(const rclcpp::NodeOptions &options) : Node("camera_node", options)
{
    // Initialize device
    rcl_interfaces::msg::ParameterDescriptor param_descriptor;
    param_descriptor.read_only = true;
    declare_parameter<std::string>("camera_path", "/dev/video0", param_descriptor);
    camera = std::make_unique<grabthecam::CameraCapture>(get_parameter("camera_path").as_string());

    // Retrieve properties
    prepare_driver_parameters();
    generate_mat_mappings();

    // Set device constraints
    std::pair<int, int> size = camera->getFormat();
    declare_parameter<std::vector<int64_t>>("camera_frame_dim", std::vector<int64_t>{size.first, size.second});
    if (get_parameter("camera_frame_dim").as_integer_array() != std::vector<int64_t>{size.first, size.second})
    {
        camera->setFormat(
            static_cast<uint32_t>(get_parameter("camera_frame_dim").as_integer_array()[0]),
            static_cast<uint32_t>(get_parameter("camera_frame_dim").as_integer_array()[1]));
    }

    declare_parameter<double>("camera_refresh_rate", 30.0);
    declare_parameter<double>("camera_info_rate", 0.1);
    camera_frame_counter = 0;
    camera_frame_time_point = std::chrono::steady_clock::now();

    parameters_set_handle = add_on_set_parameters_callback(std::bind(&CameraNode::parameters_set_callback, this, _1));

    camera_get_properties_srv = create_service<camera_node::srv::CameraGetProperties>(
        "camera_get_properties",
        std::bind(&CameraNode::camera_get_properties_callback, this, _1, _2));

    camera_get_property_details_srv = create_service<camera_node::srv::CameraGetPropertyDetails>(
        "camera_get_property_details",
        std::bind(&CameraNode::camera_get_property_details_callback, this, _1, _2));

    camera_frame_pub = create_publisher<sensor_msgs::msg::Image>("camera_frame", 1);

    camera_frame_clock = create_wall_timer(
        std::chrono::duration<double>(1 / get_parameter("camera_refresh_rate").as_double()),
        std::bind(&CameraNode::camera_frame_callback, this));

    camera_frame_loopback = create_subscription<sensor_msgs::msg::Image>(
        "camera_frame",
        10,
        std::bind(&CameraNode::camera_frame_loopback_callback, this, _1));

    camera_frame_info_clock = create_wall_timer(
        std::chrono::duration<double>(1 / get_parameter("camera_info_rate").as_double()),
        std::bind(&CameraNode::camera_frame_info_callback, this));
}

CameraNode::~CameraNode() {}
} // namespace camera_node

RCLCPP_COMPONENTS_REGISTER_NODE(camera_node::CameraNode)
