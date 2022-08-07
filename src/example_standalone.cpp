#include "er_ros2_example_package/example_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  rclcpp::spin(std::make_shared<example_namespace::ExampleNode>(options,false)->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
