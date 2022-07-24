# Explaining Robotics ROS2 Example Package

## Purpose
The Explaining Robotics ROS2 Example Package wants to be a template which can be copied to quickly create new packages, as it includes all relevant interfaces with an example already.
Therefore no lookup in the ROS2 Tutorials is neccessary on how to write a CPP Subscriber, Publisher, Service or Action.

The Example also focuses on Class based C++ implementation and directly export the node as component so that it can be used in a single process with shared memory.

## Template
For quick iteration you can also directly create a Repository based on this Template with the `Use this Template` Button on the top next to the clone Button.

### Creating a new Package / Node out of this Template
With Search all and replace all the use of the template is quite easy.
Only search for 
* ExampleNode (Class Name)
* example_namespace (namespace name)
* example_node (Shared Lib Name)
* example_standalone (Executable Name)
* EXAMPLE_NODE_DLL (DLL Name)
* er_ros2_example_package (Package Name)

and replace them with your new names. Then rename the folder and files

* example_node.cpp
* example_standalone.cpp
* include/er_ros2_example_package
* example_node.hpp

## Launch
`` 
ros2 launch er_ros2_example_package example_standalone_launch.py
``
