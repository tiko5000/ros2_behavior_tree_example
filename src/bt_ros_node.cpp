#include <chrono>

#include "ros2_behavior_tree_example/bt_ros_node.hpp"
#include "ros2_behavior_tree_example/plugins/pong_received_bt_node.hpp"
#include "ros2_behavior_tree_example/plugins/pong_received_executor_bt_node.hpp"
#include "ros2_behavior_tree_example/plugins/ping_bt_node.hpp"
#include "ros2_behavior_tree_example/plugins/log_status_bt_node.hpp"

#include "behaviortree_cpp/blackboard.h"
#include "rclcpp/publisher.hpp"

#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.h"

using namespace std::chrono_literals;

namespace polymath
{
namespace bt_ros_example
{
    using LifecycleNodeInterface = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

    BtRosNode::BtRosNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("bt_ros_node", "", options)
    {
        // Get all necesssary params for ros2 to utilize
        param_listener_ = std::make_shared<bt_params::ParamListener>(this->get_node_parameters_interface());
        params_ = param_listener_->get_params();

        // Example for changing a parameter:
        RCLCPP_INFO(this->get_logger(), "Initial parameter: num_republish = %i", params_.num_republish);

        std::vector<rclcpp::Parameter> new_params;
        new_params.emplace_back("num_republish", 9);  // Manually setting new value

        // Apply the update
        auto result = param_listener_->update(new_params);
        if (result.successful)
        {
            params_ = param_listener_->get_params();
            RCLCPP_INFO(this->get_logger(), "Manually updated num_republish to: %i", params_.num_republish);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Parameter update failed: %s", result.reason.c_str());
        }

        // Register Nodes into the Factory to generate a tree later
        factory_.registerNodeType<PongReceivedNode>("PongReceivedNode");
        factory_.registerNodeType<PongReceivedExecutorNode>("PongReceivedExecutorNode");
        factory_.registerNodeType<PingNode>("PingNode");
        factory_.registerNodeType<LogStatusNode>("LogStatusNode");
    }

    BtRosNode::~BtRosNode()
    {
        on_deactivate(get_current_state());
        on_cleanup(get_current_state());
    }

    LifecycleNodeInterface::CallbackReturn
    BtRosNode::on_configure(const rclcpp_lifecycle::State &)
    {
        // Set up the blackboard for the behavior tree
        blackboard_ = BT::Blackboard::create();
        blackboard_->set<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node", this->shared_from_this());
        blackboard_->set<int32_t>("num_publish", this->get_parameter("num_republish").as_int());
        blackboard_->set<bool>("ping_start", this->get_parameter("ping_starter").as_bool());
        blackboard_->set<int32_t>("ping_id", 0);
        blackboard_->set<int32_t>("pong_id", 0);

        RCLCPP_INFO(get_logger(), "Loading file %s", get_parameter("behaviortree_file").as_string().c_str());

        tree_ = factory_.createTreeFromFile(this->get_parameter("behaviortree_file").as_string(), blackboard_);
        
        // Running a timer to run this at a stable rate
        // This enables us to run the executor with just a spin at the upper level
        std::chrono::milliseconds rate(int32_t(1000.0 / this->get_parameter("rate_hz").as_double()));
        timer_ = this->create_wall_timer(rate, 
                                        std::bind(&BtRosNode::timer_callback, this));

        // start with the timer cancelled
        RCLCPP_INFO(this->get_logger(), "Stopping Timer from running");
        timer_->cancel();

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn
    BtRosNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
    {
        RCLCPP_INFO(this->get_logger(), "Starting the Timer and running Ticks");
        timer_->reset();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn
    BtRosNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
    {
        RCLCPP_INFO(this->get_logger(), "Stopping Timer");
        timer_->cancel();

        // We can wait until Cancel as well by doing timer_->is_cancelled()

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn
    BtRosNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning Up");
        blackboard_.reset();
        timer_.reset();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    BtRosNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
    {
        on_deactivate(get_current_state());
        on_cleanup(get_current_state());

        RCLCPP_INFO(this->get_logger(), "Shutting Down");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void
    BtRosNode::timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Ticking the tree");
        tree_.tickOnce();
        return;
    }
}
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(polymath::bt_ros_example::BtRosNode)
