cmake_minimum_required(VERSION 3.8)
project(er_ros2_example_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)
set(node_plugins "")
###########
#  BUILD  #
###########

add_library(example_node SHARED
            src/example_node.cpp)
target_compile_definitions(example_node
  PRIVATE "EXAMPLE_NODE_DLL")
ament_target_dependencies(example_node rclcpp rclcpp_components rclcpp_action rcpputils std_msgs std_srvs example_interfaces Eigen3)
rclcpp_components_register_nodes(example_node "example_namespace::ExampleNode")
set(node_plugins "${node_plugins}example_namespace::ExampleNode;$<TARGET_FILE:example_node>\n")

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(example_standalone src/example_standalone.cpp)
target_link_libraries(example_standalone example_node)
ament_target_dependencies(example_standalone
  rclcpp)

#############
#  INSTALL  #
#############
install(TARGETS
example_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  example_standalone
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
  )

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
