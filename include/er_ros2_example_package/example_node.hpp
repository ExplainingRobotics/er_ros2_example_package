#pragma once
// Standard CPP
#include <memory>
#include <functional>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "er_ros2_example_package/visibility.h"
#include "rcpputils/asserts.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

//Messages
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "example_interfaces/action/fibonacci.hpp"

//External Libs
#include <eigen3/Eigen/Dense>

namespace example_namespace
{

class ExampleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    COMPOSITION_PUBLIC ExampleNode(rclcpp::NodeOptions options);
    COMPOSITION_PUBLIC ExampleNode(rclcpp::NodeOptions options, bool activate_lifecycle);

protected:
  /**
   * @brief Sets up required params and services. Loads map and its parameters from the file
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Start publishing the map using the latched topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Stops publishing the latched topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets the member variables
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;


private:
  /**
   * @brief Timer Callback to publish Data
   */
  void publishTimer();
  /**
   * @brief Registers all publishers
   */
  void registerPublisher();
  /**
   * @brief Registers all subscribers
   */
  void registerSubscriber();
  /**
   * @brief Registers all services
   */
  void registerServices();
  /**
   * @brief Registers all actions
   */
  void registerActions();
  /**
   * @brief Handles Parameter Server and changes
   */
  void registerParameters();
  /**
   * @brief Callback for the Subscriber
   * @param msg incomming message
   */
  void callbackString(std_msgs::msg::String::ConstSharedPtr msg);
  /**
   * @brief Callback to handle the Parameter Change event
   * @param event incomming message
   */
  void onParamEvent(rcl_interfaces::msg::ParameterEvent::UniquePtr event);
  /**
   * @brief Callback to handle the SetBool Service
   * @param request_header ros2 header
   * @param request input request message
   * @param response output response message
   */
  void callbackService(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  /**
   * @brief Callback to handle Cancling of the Action
   * @param goal_handle The goal handle to cancle
   * @return ACCEPT or REJECT
   */
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle);
  /**
   * @brief Callback to for executing the Goal
   * @param goal_handle The goal handle to cancle
   */
  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle);
  /**
   * @brief Callback to handle activating the Goal of the Action
   * @param goal The goal
   * @return ACCEPT or REJECT
   */
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const example_interfaces::action::Fibonacci::Goal> goal);
  /**
   * @brief Callback after accepting before executing the goal
   * @param goal_handle The goal handle to cancle
   */
  void handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle);
  
  bool activate_lifecycle_;
  
  long count_;
  bool positive_ = true;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub;
  rclcpp::AsyncParametersClient::SharedPtr param_client_;
  std::string example_parameter_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_string_;
  rclcpp::TimerBase::SharedPtr timer_publish_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_string_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_set_bool_;
  rclcpp_action::Server<example_interfaces::action::Fibonacci>::SharedPtr action_server_;
  Eigen::Vector2d eigen_vector_;
};

} // namespace
