#include "er_ros2_example_package/example_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<example_namespace::ExampleNode>(options));
  rclcpp::shutdown();
  return 0;
}
