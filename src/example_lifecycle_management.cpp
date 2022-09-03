#include "er_ros2_example_package/example_node.hpp"
using namespace std::chrono_literals;
namespace example_namespace
{
/// Transition callback for state configuring
/**
 * on_configure callback is being called when the lifecycle node
 * enters the "configuring" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "inactive" state or stays
 * in "unconfigured".
 * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
 * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ExampleNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "on_configure() is called.");
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
    timer_heartbeat_ = create_wall_timer(heartbeat_period_,std::bind(&ExampleNode::publishHeartbeat, this));
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state activating
/**
 * on_activate callback is being called when the lifecycle node
 * enters the "activating" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "active" state or stays
 * in "inactive".
 * TRANSITION_CALLBACK_SUCCESS transitions to "active"
 * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ExampleNode::on_activate(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(this->get_logger(), "on_activate() is called.");
    // The parent class method automatically transition on managed entities
    // (currently, LifecyclePublisher).
    // pub_->on_activate() could also be called manually here.
    // Overriding this method is optional, a lot of times the default is enough.
    LifecycleNode::on_activate(state);

    // Let's sleep for 2 seconds.
    // We emulate we are doing important
    // work in the activating phase.
    std::this_thread::sleep_for(2s);

    // We return a success and hence invoke the transition to the next
    // step: "active".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "inactive" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state deactivating
/**
 * on_deactivate callback is being called when the lifecycle node
 * enters the "deactivating" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "inactive" state or stays
 * in "active".
 * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
 * TRANSITION_CALLBACK_FAILURE transitions to "active"
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ExampleNode::on_deactivate(const rclcpp_lifecycle::State &state)
{
    // The parent class method automatically transition on managed entities
    // (currently, LifecyclePublisher).
    // pub_->on_deactivate() could also be called manually here.
    // Overriding this method is optional, a lot of times the default is enough.
    LifecycleNode::on_deactivate(state);

    RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");

    // We return a success and hence invoke the transition to the next
    // step: "inactive".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "active" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state cleaningup
/**
 * on_cleanup callback is being called when the lifecycle node
 * enters the "cleaningup" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "unconfigured" state or stays
 * in "inactive".
 * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
 * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ExampleNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    // In our cleanup phase, we release the shared pointers to the
    // timer and publisher. These entities are no longer available
    // and our node is "clean".
    timer_publish_.reset();
    pub_string_.reset();
    sub_string_.reset();
    param_subscriber_.reset();
    param_sub.reset();
    param_client_.reset();
    service_set_bool_.reset();
    action_server_.reset();
    RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");

    // We return a success and hence invoke the transition to the next
    // step: "unconfigured".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the "inactive" state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/// Transition callback for state shutting down
/**
 * on_shutdown callback is being called when the lifecycle node
 * enters the "shuttingdown" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "finalized" state or stays
 * in its current state.
 * TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
 * TRANSITION_CALLBACK_FAILURE transitions to current state
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ExampleNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    // In our shutdown phase, we release the shared pointers to the
    // timer and publisher. These entities are no longer available
    // and our node is "clean".
    timer_publish_.reset();
    pub_string_.reset();
    sub_string_.reset();
    param_subscriber_.reset();
    param_sub.reset();
    param_client_.reset();
    service_set_bool_.reset();
    action_server_.reset();
    RCLCPP_INFO(this->get_logger(), "on shutdown is called from state %s. ", state.label().c_str());

    // We return a success and hence invoke the transition to the next
    // step: "finalized".
    // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
    // would stay in the current state.
    // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
    // this callback, the state machine transitions to state "errorprocessing".
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
} // namespace 
