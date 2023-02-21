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

add_subdirectory(${PROJECT_SOURCE_DIR}/third-party/grabthecam)

set(CAMERA_NODE_SRC
    src/camera_node_core.cpp
)

set(FRAME_FETCHER_NODE_SRC
    src/frames_fetcher_node.cpp
)

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/CameraProperty.msg
    msg/CameraPropertyDetails.msg
    msg/CameraPropertyMenuEntry.msg
    srv/CameraGetProperties.srv
    srv/CameraGetPropertyDetails.srv
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_node(camera_node_lib PLUGIN "camera_node::CameraNode" EXECUTABLE camera_node_exe)
rclcpp_components_register_node(camera_node_lib PLUGIN "camera_node::FrameFetcherNode" EXECUTABLE frame_fetcher_node_exe)

add_library(camera_node_lib ${CAMERA_NODE_SRC})
set_property(TARGET camera_node_lib PROPERTY OUTPUT_NAME camera_node)

add_library(frame_fetcher_node_lib ${FRAME_FETCHER_NODE_SRC})
set_property(TARGET frame_fetcher_node_lib PROPERTY OUTPUT_NAME frame_fetcher_node)

target_include_directories(camera_node_lib PRIVATE
    ${PROJECT_SOURCE_DIR}/third-party/grabthecam/include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(frame_fetcher_node_lib PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(camera_node_lib
    ${cpp_typesupport_target}
    grabthecam
)

target_link_libraries(frame_fetcher_node_lib
    ${cpp_typesupport_target}
)

install(TARGETS camera_node_lib DESTINATION lib)
install(TARGETS frame_fetcher_node_lib DESTINATION lib)
install(TARGETS grabthecam LIBRARY DESTINATION lib)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_target_dependencies(camera_node_lib
    cv_bridge
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
)

ament_target_dependencies(frame_fetcher_node_lib
    OpenCV
    cv_bridge
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
