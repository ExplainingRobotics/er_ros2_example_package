#!/usr/bin/env python3

from launch import LaunchDescription
import os
import sys
from launch_testing.legacy import LaunchTestService
from launch.actions import ExecuteProcess
from launch import LaunchService
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node = Node(
            package='er_ros2_example_package',
            namespace='',
            executable='example_standalone',
            name='example_node_test',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"example_parameter": "world",
                "heartbeat_period": 200}
            ]
        )
    ld.add_action(node)

    return ld 


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()
    testExecutable = os.getenv('TEST_EXECUTABLE')
    print(testExecutable)
    test1_action = ExecuteProcess(
        cmd=[testExecutable],
        name="example_test",
        output='screen',
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)

if __name__ == '__main__':
    sys.exit(main())
