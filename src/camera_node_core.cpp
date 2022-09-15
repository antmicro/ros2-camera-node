#include "camera_node_core.hpp"
#include <regex>

namespace camera_node
{
rcl_interfaces::msg::SetParametersResult CameraNode::parameters_set_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reconfiguring camera");
    for (const auto &parameter : parameters)
    {
        if (parameters_name_to_index.contains(parameter.get_name()))
        {
            camera->set<int64_t>(parameters_name_to_index[parameter.get_name()], parameter.as_int());
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
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting [%s]", parameter.get_name().c_str());
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    return result;
}

void CameraNode::camera_get_properties_callback(
    const camera_node::srv::CameraGetProperties::Request::SharedPtr request,
    camera_node::srv::CameraGetProperties::Response::SharedPtr response)
{
    for (auto property = camera_node::msg::CameraProperty();
         const auto &[index, name, type, default_value] : camera->queryProperties())
    {
        property.index = index;
        property.name = name;
        property.type = static_cast<uint8_t>(type);
        property.default_value = default_value;

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
    cv_bridge::CvImage(std_msgs::msg::Header(), cvmat_type_to_str[frame.type()], frame).toImageMsg(message);

    camera_frame_pub->publish(message);
}

void CameraNode::camera_frame_loopback_callback(const sensor_msgs::msg::Image &message)
{
    camera_frame_counter++;
}

void CameraNode::camera_frame_info_callback()
{
    auto frequency =
        static_cast<double>(camera_frame_counter) /
        std::chrono::duration<double, std::ratio<1L>>(std::chrono::steady_clock::now() - camera_frame_time_point).count();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Camera framerate: [%f]", frequency);
    camera_frame_counter = 0;
    camera_frame_time_point = std::chrono::steady_clock::now();
}

void CameraNode::prepare_internal_parameters()
{
    for (auto &&[index, name, type, default_value] : camera->queryProperties())
    {
        const std::regex regex("[^A-z0-9\\s]");
        std::stringstream result;

        std::regex_replace(std::ostream_iterator<char>(result), name.begin(), name.end(), regex, "");
        name = result.str();

        std::transform(name.begin(), name.end(), name.begin(), [](auto &c) { return std::tolower(c); });
        std::replace(name.begin(), name.end(), ' ', '_');

        parameters_name_to_index[name] = index;
        declare_parameter<int32_t>("camera_internal_" + name, default_value);
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

CameraNode::CameraNode(const rclcpp::NodeOptions &Options) : Node("camera_node", Options)
{
    rcl_interfaces::msg::ParameterDescriptor param_descriptor;
    param_descriptor.read_only = true;
    declare_parameter<std::string>("camera_path", "/dev/video0", param_descriptor);
    declare_parameter<std::vector<int64_t>>("camera_frame_dim", {0, 0});
    declare_parameter<double>("camera_refresh_rate", 30.0);
    declare_parameter<double>("camera_info_rate", 0.1);

    camera = std::make_unique<grabthecam::CameraCapture>(get_parameter("camera_path").as_string());
    if (get_parameter("camera_frame_dim").as_integer_array() != std::vector<int64_t>{0, 0})
    {
        camera->setFormat(
            static_cast<uint32_t>(get_parameter("camera_frame_dim").as_integer_array()[0]),
            static_cast<uint32_t>(get_parameter("camera_frame_dim").as_integer_array()[1]));
    }

    prepare_internal_parameters();
    generate_mat_mappings();
    camera_frame_counter = 0;
    camera_frame_time_point = std::chrono::steady_clock::now();

    parameters_set_handle =
        add_on_set_parameters_callback(std::bind(&CameraNode::parameters_set_callback, this, std::placeholders::_1));

    camera_get_properties_srv = create_service<camera_node::srv::CameraGetProperties>(
        "camera_get_properties",
        std::bind(&CameraNode::camera_get_properties_callback, this, std::placeholders::_1, std::placeholders::_2));

    camera_get_property_details_srv = create_service<camera_node::srv::CameraGetPropertyDetails>(
        "camera_get_property_details",
        std::bind(&CameraNode::camera_get_property_details_callback, this, std::placeholders::_1, std::placeholders::_2));

    camera_frame_pub = create_publisher<sensor_msgs::msg::Image>("camera_frame", 1);

    camera_frame_clock = create_wall_timer(
        std::chrono::duration<double>(1 / get_parameter("camera_refresh_rate").as_double()),
        std::bind(&CameraNode::camera_frame_callback, this));

    camera_frame_loopback = create_subscription<sensor_msgs::msg::Image>(
        "camera_frame",
        10,
        std::bind(&CameraNode::camera_frame_loopback_callback, this, std::placeholders::_1));

    camera_frame_info_clock = create_wall_timer(
        std::chrono::duration<double>(1 / get_parameter("camera_info_rate").as_double()),
        std::bind(&CameraNode::camera_frame_info_callback, this));
}

CameraNode::~CameraNode() {}
} // namespace camera_node