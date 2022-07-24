#include <chrono>

#include "er_ros2_example_package/example_node.hpp"

using namespace std::chrono_literals;
namespace example_namespace
{
    ExampleNode::ExampleNode(rclcpp::NodeOptions options)
    : Node("example_node", options)
    {
        // Register Interfaces
        registerParameters();
        registerSubscriber();
        registerPublisher();
        registerServices();
        registerActions();
        // Check that Eigen (external Library) works
        RCLCPP_INFO_STREAM(this->get_logger(),"Eigen Vecotr Size: " << eigen_vector_.size());
        // Create Timer to publish
        timer_publish_ = create_wall_timer(500ms, std::bind(&ExampleNode::publishTimer, this));
    }

    void ExampleNode::registerParameters(){
        RCLCPP_INFO_STREAM(this->get_logger(),"Reading Parameters");
        //Declaring Parameters for this node
        this->declare_parameter<std::string>("example_parameter");

        //For Handling Parameter changes
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this,this->get_name());
        param_sub = param_client_->on_parameter_event(std::bind(&ExampleNode::onParamEvent,this,std::placeholders::_1));

        //Read Parameter
        rcpputils::assert_true(this->get_parameter("example_parameter", example_parameter_),"Failed to get param 'example_parameter'");
        RCLCPP_INFO_STREAM(this->get_logger(),example_parameter_);
    }
    void ExampleNode::registerPublisher(){
        RCLCPP_INFO_STREAM(this->get_logger(),"Register Publisher");
        pub_string_ = create_publisher<std_msgs::msg::String>("~/publish", 10);
    }
    void ExampleNode::registerSubscriber(){
        RCLCPP_INFO_STREAM(this->get_logger(),"Register Subscriber");
        sub_string_ = create_subscription<std_msgs::msg::String>("~/subscribe", 10, std::bind(&ExampleNode::callbackString, this, std::placeholders::_1));
    }
    void ExampleNode::registerServices(){
        RCLCPP_INFO_STREAM(this->get_logger(),"Register Services");
        service_set_bool_ = create_service<std_srvs::srv::SetBool>("~/count_positive", std::bind(&ExampleNode::callbackService, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
    }
    void ExampleNode::registerActions(){
        RCLCPP_INFO_STREAM(this->get_logger(),"Register Actions");
        action_server_ = rclcpp_action::create_server<example_interfaces::action::Fibonacci>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "~/fibonacci",
            std::bind(&ExampleNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ExampleNode::handleCancel, this, std::placeholders::_1),
            std::bind(&ExampleNode::handleAccepted, this, std::placeholders::_1));
    }

    void ExampleNode::onParamEvent(rcl_interfaces::msg::ParameterEvent::UniquePtr event){
        // ignore qos overrides
        event->new_parameters.erase(
            std::remove_if(
                event->new_parameters.begin(),
                event->new_parameters.end(),
                [](const auto & item) {
                    const char * param_override_prefix = "qos_overrides.";
                    return std::strncmp(
                    item.name.c_str(), param_override_prefix, sizeof(param_override_prefix) - 1) == 0u;
                }),
            event->new_parameters.end());
        if (!event->new_parameters.size() && !event->changed_parameters.size() && !event->deleted_parameters.size())
        {
            return;
        }
        if(event->node == std::string("/") + this->get_name()){
            for (auto & changed_parameter : event->changed_parameters) {
                RCLCPP_INFO_STREAM(this->get_logger(),"Changed Parameter : " << changed_parameter.name << "of Value Type: "<< std::to_string(changed_parameter.value.type) << ", to:  " << changed_parameter.value.string_value);
                if(changed_parameter.name == "example_parameter"){
                    example_parameter_ = changed_parameter.value.string_value;
                }
            }
        }
    }
    
    void ExampleNode::publishTimer()
    {
        auto message = std_msgs::msg::String();
        if(positive_){
            message.data = "Hello, " + example_parameter_  + "! " + std::to_string(count_++);
        }else{
            message.data = "Hello, " + example_parameter_  + "! " + std::to_string(count_--);
        }        
        RCLCPP_INFO(this->get_logger(), "Publisher: '%s'", message.data.c_str());
        pub_string_->publish(message);
    }
    void ExampleNode::callbackString(const std_msgs::msg::String & msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    void ExampleNode::callbackService(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response){
        (void)request_header;
        positive_ = request->data;
        response->success = true;
        response->message = "Reversed counter";
    }
    rclcpp_action::GoalResponse ExampleNode::handleGoal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const example_interfaces::action::Fibonacci::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void)uuid;
        // Let's reject sequences that are over 9000
        if (goal->order > 9000) {
        return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ExampleNode::handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void ExampleNode::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<example_interfaces::action::Fibonacci::Feedback>();
        auto & sequence = feedback->sequence;
        sequence.push_back(0);
        sequence.push_back(1);
        auto result = std::make_shared<example_interfaces::action::Fibonacci::Result>();

        for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
            result->sequence = sequence;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal Canceled");
            return;
        }
        // Update sequence
        sequence.push_back(sequence[i] + sequence[i - 1]);
        // Publish feedback
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish Feedback");

        loop_rate.sleep();
        }

        // Check if goal is done
        if (rclcpp::ok()) {
        result->sequence = sequence;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
        }
    }

    void ExampleNode::handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&ExampleNode::execute, this, _1), goal_handle}.detach();
    }
} // namespace 

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(example_namespace::ExampleNode)