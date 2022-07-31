#include "er_ros2_example_package/example_node.hpp"
#include <gtest/gtest.h>

#include <chrono>
#include <thread>
#include <boost/algorithm/string.hpp>

bool got_string_ = false;
int count = -1000000;
void callbackString(const std_msgs::msg::String::SharedPtr msg)
{
  got_string_ = true;
  std::vector<std::string> strs;
  boost::split(strs, msg->data, boost::is_any_of("! "));
  count = stoi(strs[strs.size()-1]);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("test_logger"), "Read Number from Topic " << strs[strs.size()-1]);
}

// ROS Node unit tests
TEST(ExampleTest,interface_publisher)
{
    auto node_ = rclcpp::Node::make_shared("Publisher_Testcase");
    auto sub_string_ = node_->create_subscription<std_msgs::msg::String>(
    "/example_node_test/publish",10, callbackString);
    rclcpp::Rate loopRate(10);
    for (size_t i = 0; i < 30; ++i) {
        rclcpp::spin_some(node_);
        loopRate.sleep();
    }
    ASSERT_EQ(got_string_,true) << "We should have received a String";
}

TEST(ExampleTest,interface_service_negative)
{
    auto node_ = rclcpp::Node::make_shared("Service_Testcase");
    auto client = node_->create_client<std_srvs::srv::SetBool>("/example_node_test/count_positive");
    auto sub_string_ = node_->create_subscription<std_msgs::msg::String>(
    "/example_node_test/publish",10, callbackString);
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "client interrupted while waiting for service to appear.");
        ASSERT_EQ(false,true) << "The Service should have been advertised";
        }
        RCLCPP_INFO(node_->get_logger(), "waiting for service to appear...");
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "service call failed :(");
        client->remove_pending_request(result_future);
        ASSERT_EQ(false,true) << "The Service shouldn't have failed";
    }
    rclcpp::Rate loopRate(10);
    for (size_t i = 0; i < 20; ++i) {
        rclcpp::spin_some(node_);
        loopRate.sleep();
    }
    int before = count;
    for (size_t i = 0; i < 30; ++i) {
        rclcpp::spin_some(node_);
        loopRate.sleep();
    }
    int after = count;
    ASSERT_GT(before,after) << " We should have counted negative";
}

TEST(ExampleTest,interface_service_positive)
{
    auto node_ = rclcpp::Node::make_shared("Service_Testcase");
    auto client = node_->create_client<std_srvs::srv::SetBool>("/example_node_test/count_positive");
    auto sub_string_ = node_->create_subscription<std_msgs::msg::String>(
    "/example_node_test/publish",10, callbackString);
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "client interrupted while waiting for service to appear.");
        ASSERT_EQ(false,true) << "The Service should have been advertised";
        }
        RCLCPP_INFO(node_->get_logger(), "waiting for service to appear...");
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "service call failed :(");
        client->remove_pending_request(result_future);
        ASSERT_EQ(false,true) << "The Service shouldn't have failed";
    }
    rclcpp::Rate loopRate(10);
    for (size_t i = 0; i < 20; ++i) {
        rclcpp::spin_some(node_);
        loopRate.sleep();
    }
    int before = count;
    for (size_t i = 0; i < 30; ++i) {
        rclcpp::spin_some(node_);
        loopRate.sleep();
    }
    int after = count;
    ASSERT_GT(after,before) << "We should have counted positive";
}

TEST(ExampleTest,interface_action_server_send_goal)
{
    auto node_ = rclcpp::Node::make_shared("Action_Testcase");
    auto action_client = rclcpp_action::create_client<example_interfaces::action::Fibonacci>(node_, "/example_node_test/fibonacci");
    auto sub_string_ = node_->create_subscription<std_msgs::msg::String>("/example_node_test/publish",10, callbackString);
    while (!action_client->wait_for_action_server(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "client interrupted while waiting for service to appear.");
        ASSERT_EQ(false,true) << "The Action should have been advertised";
        }
        RCLCPP_INFO(node_->get_logger(), "waiting for action to appear...");
    }
    auto goal_msg = example_interfaces::action::Fibonacci::Goal();
    goal_msg.order = 5;
    RCLCPP_INFO(node_->get_logger(), "Sending goal");

    // Ask server to achieve some goal and wait until it's accepted
    auto goal_handle_future = action_client->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "send goal call failed");
        ASSERT_EQ(false,true) << "We should be able to send a goal";
    }
    rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>::SharedPtr goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        ASSERT_EQ(false,true) << "Goal shouldn't be rejected";
    }
    // Wait for the server to be done with the goal
    auto result_future = action_client->async_get_result(goal_handle);
    RCLCPP_INFO(node_->get_logger(), "Waiting for result");
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "get result call failed :(");
        ASSERT_EQ(false,true) << "Didn't get the Result";
    }

    rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>::WrappedResult wrapped_result = result_future.get();
    ASSERT_EQ(wrapped_result.code,rclcpp_action::ResultCode::SUCCEEDED) << "Goal should have been succeeded";
    ASSERT_EQ(wrapped_result.result->sequence[wrapped_result.result->sequence.size()-1],5) << "Fith Fibonacci Number should have been 5";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::Rate(1).sleep();

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}