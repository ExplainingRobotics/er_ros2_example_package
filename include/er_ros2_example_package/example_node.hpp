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
    void publishTimer();
    void registerPublisher();
    void registerSubscriber();
    void registerServices();
    void registerActions();
    void registerParameters();
    void callbackString(std_msgs::msg::String::ConstSharedPtr msg);
    void onParamEvent(rcl_interfaces::msg::ParameterEvent::UniquePtr event);
    void callbackService(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle);
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const example_interfaces::action::Fibonacci::Goal> goal);
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
