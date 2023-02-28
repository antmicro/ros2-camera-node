cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 20)

project(camera_node)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
    include
    third-party/grabthecam/include
    ${rclcpp_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(frame_fetcher_node src/frame_fetcher_node.cpp)
add_library(camera_node_core src/camera_node_core.cpp)

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/CameraProperty.msg
    msg/CameraPropertyDetails.msg
    msg/CameraPropertyMenuEntry.msg
    srv/CameraGetProperties.srv
    srv/CameraGetPropertyDetails.srv
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(camera_node_core
    ${cpp_typesupport_target}
    grabthecam
)

target_link_libraries(frame_fetcher_node
    ${cpp_typesupport_target}
    ${OpenCV_LIBS}
)

install(
    TARGETS
        camera_node_core
        frame_fetcher_node
    INCLUDES DESTINATION include
    LIBRARY DESTINATION lib
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_target_dependencies(camera_node_core
    cv_bridge
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
)

ament_target_dependencies(frame_fetcher_node
    cv_bridge
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
)

rclcpp_components_register_node(camera_node_core PLUGIN "camera_node::CameraNode" EXECUTABLE camera_node_exe)
rclcpp_components_register_node(frame_fetcher_node PLUGIN "camera_node::FrameFetcherNode" EXECUTABLE frame_fetcher_node_exe)

ament_export_dependencies(rosidl_default_runtime)

add_custom_target(format
    COMMAND bash -c "find ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/include -iname \"*.cpp\" -o -iname \"*.hpp\" | xargs clang-tidy -format-style=file -p ${CMAKE_BINARY_DIR} -fix"
    COMMAND bash -c "find ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/include -iname \"*.cpp\" -o -iname \"*.hpp\" | xargs clang-format --style=file -i"
    VERBATIM
)

ament_package()
