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
* example_test (Test Name)

and replace them with your new names. Then rename the folder and files

* example_node.cpp
* example_standalone.cpp
* include/er_ros2_example_package
* example_node.hpp
* example_node.yaml
* example_composed_launch.py
* example_external_composed_launch.py
* example_standalone_launch.py
* example_test.cpp
* example_test.launch.py (make sure it is chmod +x for running tests)

## Launch
`` 
ros2 launch er_ros2_example_package example_standalone_launch.py
``

## Test
go to `/build/er_ros2_example_package` and execute
`` 
ctest -V -R example_test
``
or
`` 
colcon test --packages-select er_ros2_example_package
``

## Debugging

To use GDB add 
`` 
add_compile_options(-g)
``
to your `CMakeLists.txt` and add
`` 
prefix=['xterm -e gdb -ex run --args'],
``
to your launch file. After a crash type `backtrace` to get the backtrace