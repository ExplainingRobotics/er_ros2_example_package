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
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)
set(node_plugins "")
###########
#  BUILD  #
###########

add_library(example SHARED
            src/example.cpp
            src/example_lifecycle_management.cpp)
target_compile_definitions(example
  PRIVATE "EXAMPLE_NODE_DLL")
ament_target_dependencies(
  example
  rclcpp
  rclcpp_components
  rclcpp_action
  rcpputils
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  std_srvs
  example_interfaces
  Eigen3
  tf2
  tf2_ros
  geometry_msgs
)

rclcpp_components_register_nodes(example "example_namespace::ExampleNode")
set(node_plugins "${node_plugins}example_namespace::ExampleNode;$<TARGET_FILE:example>\n")

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(example_node src/example_node.cpp)
target_link_libraries(example_node example)
ament_target_dependencies(example_node rclcpp)

#############
#  INSTALL  #
#############
install(TARGETS
example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  example_node
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

#############
#  TESTING  #
#############

if(BUILD_TESTING)
  # LINT TESTING
  find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # UNIT TESTING
  find_package(ament_cmake_gtest REQUIRED)
  # find_package(ament_cmake_cppcheck REQUIRED)
  # find_package(ament_cmake_cpplint REQUIRED)
  # find_package(ament_cmake_lint_cmake REQUIRED)
  # find_package(ament_cmake_uncrustify REQUIRED)
  # find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest_executable(example_test test/example_test.cpp)
  target_link_libraries(example_test example)

  ament_add_test(example_test
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 180
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/example_test.launch.py"
    ENV
      TEST_EXECUTABLE=$<TARGET_FILE:example_test>
  )
  install(
    TARGETS
    example_test
    DESTINATION lib/${PROJECT_NAME}
  )

  # Install test launch config files.
  install(DIRECTORY
    test
    DESTINATION share/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
  )
endif()

ament_package()
