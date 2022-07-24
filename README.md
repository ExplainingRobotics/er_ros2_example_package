# Explaining Robotics ROS2 Example Package

## Purpose
The Explaining Robotics ROS2 Example Package wants to be a template which can be copied to quickly create new packages, as it includes all relevant interfaces with an example already.
Therefore no lookup in the ROS2 Tutorials is neccessary on how to write a CPP Subscriber, Publisher, Service or Action.

The Example also focuses on Class based C++ implementation and directly export the node as component so that it can be used in a single process with shared memory.

## Launch
`` 
ros2 launch er_ros2_example_package example_standalone_launch.py
``

