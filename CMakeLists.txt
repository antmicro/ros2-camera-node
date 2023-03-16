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
add_subdirectory(third-party/grabthecam)

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
    DIRECTORY launch 
    DESTINATION share/${PROJECT_NAME}
)

install(
    TARGETS
        camera_node_core
        frame_fetcher_node
    INCLUDES DESTINATION include
    LIBRARY DESTINATION lib
)

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

if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_clang_tidy REQUIRED)
    ament_clang_tidy(${CMAKE_BINARY_DIR})
    ament_clang_format(CONFIG_FILE ".clang-format")
endif()


ament_package()
