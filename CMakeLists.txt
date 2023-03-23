cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 20)

project(camera_node)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(grabthecam QUIET)

if(NOT grabthecam_FOUND)
    message("grabthecam not found")
    include(ExternalProject)
    ExternalProject_Add(grabthecam_external
        GIT_REPOSITORY https://github.com/antmicro/grabthecam
        GIT_TAG main
    )
else()
    message("using system grabthecam")
endif()

include_directories(
    include
    ${grabthecam_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/CameraProperty.msg
    msg/CameraPropertyDetails.msg
    msg/CameraPropertyMenuEntry.msg
    srv/CameraGetProperties.srv
    srv/CameraGetPropertyDetails.srv
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(${PROJECT_NAME}_component SHARED
    src/camera_node_core.cpp
)

target_link_libraries(${PROJECT_NAME}_component
    ${cpp_typesupport_target}
    grabthecam
)

ament_target_dependencies(${PROJECT_NAME}_component
    cv_bridge
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
)

add_library(frame_fetcher_node SHARED
    src/frame_fetcher_node.cpp
)

target_link_libraries(frame_fetcher_node
    ${cpp_typesupport_target}
    ${OpenCV_LIBS}
)

ament_target_dependencies(frame_fetcher_node
    cv_bridge
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    TARGETS
        ${PROJECT_NAME}_component
        frame_fetcher_node
    INCLUDES DESTINATION include
    LIBRARY DESTINATION lib
)

rclcpp_components_register_node(${PROJECT_NAME}_component PLUGIN "camera_node::CameraNode" EXECUTABLE camera_node_exe)
rclcpp_components_register_node(frame_fetcher_node PLUGIN "camera_node::FrameFetcherNode" EXECUTABLE frame_fetcher_node_exe)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_clang_tidy REQUIRED)

    ament_clang_format(CONFIG_FILE ".clang-format"
        "include"
        "src"
    )

    ament_clang_tidy(CONFIG_FILE ".clang-format"
        ${CMAKE_BINARY_DIR}/compile_commands.json
        "include"
        "src"
    )
endif()

ament_package()
