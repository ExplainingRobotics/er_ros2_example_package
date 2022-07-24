#pragma once
// Standard CPP
#include <memory>
#include <functional>
#include <vector>
#include <string>
#include <thread>


// ROS
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/asserts.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "er_ros2_example_package/visibility.h"

//Messages
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "example_interfaces/action/fibonacci.hpp"

//External Libs
#include <eigen3/Eigen/Dense>

namespace example_namespace
{

class ExampleNode : public rclcpp::Node
{
public:
    COMPOSITION_PUBLIC ExampleNode(rclcpp::NodeOptions options);

private:
    void publishTimer();
    void registerPublisher();
    void registerSubscriber();
    void registerServices();
    void registerActions();
    void registerParameters();
    void callbackString(const std_msgs::msg::String & msg);
    void onParamEvent(rcl_interfaces::msg::ParameterEvent::UniquePtr event);
    void callbackService(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle);
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const example_interfaces::action::Fibonacci::Goal> goal);
    void handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle);
    long count_;
    bool positive_ = true;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub;
    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    std::string example_parameter_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_example_handle_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_string_;
    rclcpp::TimerBase::SharedPtr timer_publish_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_string_;
    
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_set_bool_;
    rclcpp_action::Server<example_interfaces::action::Fibonacci>::SharedPtr action_server_;
    Eigen::Vector2d eigen_vector_;
};

} // namespace
