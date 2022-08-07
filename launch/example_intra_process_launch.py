import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config= os.path.join(
        get_package_share_directory('er_ros2_example_package'),
        'config',
        'example_node.yaml'
        )
        
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='example_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='er_ros2_example_package',
                    plugin='example_namespace::ExampleNode',
                    name='example_node',
                    remappings=[("/example_node/subscribe", "/example_node/publish")],
                    parameters=[config],
                    extra_arguments=[{'use_intra_process_comms' : True}]),
                ComposableNode(
                    package='er_ros2_example_package',
                    plugin='example_namespace::ExampleNode',
                    name='example_node1',
                    remappings=[("/example_node1/subscribe", "/example_node/publish")],
                    parameters=[config],
                    extra_arguments=[{'use_intra_process_comms' : True}]),
            ],
            output='both',
    )

    return launch.LaunchDescription([container])