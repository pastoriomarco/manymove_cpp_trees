#include "manymove_cpp_trees/hmi_service_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace manymove_cpp_trees
{

    HMIServiceNode::HMIServiceNode(const std::string &node_name, BT::Blackboard::Ptr blackboard)
        : Node(node_name), blackboard_(blackboard)
    {
        // Create the three services.
        start_execution_srv_ = this->create_service<std_srvs::srv::Empty>(
            "start_execution",
            std::bind(&HMIServiceNode::handle_start_execution, this,
                      std::placeholders::_1, std::placeholders::_2));

        stop_execution_srv_ = this->create_service<std_srvs::srv::Empty>(
            "stop_execution",
            std::bind(&HMIServiceNode::handle_stop_execution, this,
                      std::placeholders::_1, std::placeholders::_2));

        reset_program_srv_ = this->create_service<std_srvs::srv::Empty>(
            "reset_program",
            std::bind(&HMIServiceNode::handle_reset_program, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "HMI Service Node started.");
    }

    void HMIServiceNode::handle_start_execution(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
    {
        // Set stop_execution to false and execution_resumed to true.
        blackboard_->set("execution_resumed", true);
        blackboard_->set("stop_execution", false);
        RCLCPP_INFO(this->get_logger(), "start_execution service called: stop_execution set to false, execution_resumed set to true.");
    }

    void HMIServiceNode::handle_stop_execution(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
    {
        // Set stop_execution to true.
        blackboard_->set("stop_execution", true);
        RCLCPP_INFO(this->get_logger(), "stop_execution service called: stop_execution set to true.");
    }

    void HMIServiceNode::handle_reset_program(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
    {
        // Set stop_execution and abort_mission to true, and execution_resumed to false.
        blackboard_->set("stop_execution", true);
        blackboard_->set("abort_mission", true);
        blackboard_->set("execution_resumed", false);
        RCLCPP_INFO(this->get_logger(), "reset_program service called: stop_execution and abort_mission set to true, execution_resumed set to false.");
    }

} // namespace manymove_cpp_trees
