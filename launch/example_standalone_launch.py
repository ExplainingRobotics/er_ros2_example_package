from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    config= os.path.join(
        get_package_share_directory('er_ros2_example_package'),
        'config',
        'example_node.yaml'
        )

    node = Node(
            package='er_ros2_example_package',
            namespace='',
            executable='example_standalone',
            name='example_node',
            output="screen",
            emulate_tty=True,
            prefix=['xterm -e gdb -ex run --args'],
            remappings=[("/example_node/subscribe", "/example_node/publish")],
            parameters=[
                config
            ]
        )
    ld.add_action(node)

    return ld 