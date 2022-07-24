from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='er_ros2_example_package',
            namespace='',
            executable='example_standalone',
            name='test',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"example_parameter": "world"}
            ]
        )
    ])