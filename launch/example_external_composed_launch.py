from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Remove Container if you have an already existing component_container running
    container = Node(
        name='ComponentManager',
        package='rclcpp_components',
        executable='component_container',
        output='both',
    )

    config= os.path.join(
        get_package_share_directory('er_ros2_example_package'),
        'config',
        'example_node.yaml'
        )

    load_composable_nodes = LoadComposableNodes(
        target_container='ComponentManager',
        composable_node_descriptions=[
            ComposableNode(
                    package='er_ros2_example_package',
                    plugin='example_namespace::ExampleNode',
                    name='example_node',
                    remappings=[],
                    parameters=[config]),
        ],
    )

    return LaunchDescription([container, load_composable_nodes])