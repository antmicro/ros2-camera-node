import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """
    Generates launch description for scenario where
    the frames derived by camera node are fetched and displayed
    by frame_fetcher_node.

    Returns:
    --------
    Launch description object

    """
    camera_node_container = ComposableNodeContainer(
        name='camera_node_container',
        namespace='camera_node',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='camera_node',
                    plugin='camera_node::CameraNode',
                    name='camera_node',
                    parameters=[{
                        'camera_path': '/dev/video0'
                    }]),
        ],
        output='both',
    )

    frame_fetcher_container = ComposableNodeContainer(
        name='frame_fetcher_container',
        namespace='camera_node',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='camera_node',
                    plugin='camera_node::FrameFetcherNode',
                    name='frame_fetcher_node',
                ),
        ],
        output='screen'
    )

    return launch.LaunchDescription([
        camera_node_container,
        frame_fetcher_container
    ])
