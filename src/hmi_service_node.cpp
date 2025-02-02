#include "manymove_cpp_trees/hmi_service_node.hpp"
#include <chrono>
#include <sstream>

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

        // Create a publisher for blackboard status using a standard string message.
        publisher_ = this->create_publisher<std_msgs::msg::String>("blackboard_status", 10);

        // Create a timer that calls publishBlackboardStatus() every 250ms.
        status_timer_ = this->create_wall_timer(
            250ms, std::bind(&HMIServiceNode::publishBlackboardStatus, this));
    }

    void HMIServiceNode::handle_start_execution(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
    {
        // Set stop_execution to false and execution_resumed to true.
        blackboard_->set("stop_execution", false);
        blackboard_->set("execution_resumed", true);
        RCLCPP_INFO(this->get_logger(), "start_execution: stop_execution=false, execution_resumed=true.");
    }

    void HMIServiceNode::handle_stop_execution(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
    {
        // Set stop_execution to true.
        blackboard_->set("stop_execution", true);
        RCLCPP_INFO(this->get_logger(), "stop_execution: stop_execution=true.");
    }

    void HMIServiceNode::handle_reset_program(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
    {
        // Set stop_execution and abort_mission to true and execution_resumed to false.
        blackboard_->set("stop_execution", true);
        blackboard_->set("abort_mission", true);
        blackboard_->set("execution_resumed", false);
        RCLCPP_INFO(this->get_logger(), "reset_program: stop_execution=true, abort_mission=true, execution_resumed=false.");
    }

    void HMIServiceNode::publishBlackboardStatus()
    {
        // Retrieve the three keys from the blackboard.
        bool execution_resumed = false;
        bool stop_execution = false;
        bool abort_mission = false;
        blackboard_->get("execution_resumed", execution_resumed);
        blackboard_->get("stop_execution", stop_execution);
        blackboard_->get("abort_mission", abort_mission);

        // Create a JSON string with the status.
        std_msgs::msg::String msg;
        std::ostringstream ss;
        ss << "{\"execution_resumed\": " << (execution_resumed ? "true" : "false")
           << ", \"stop_execution\": " << (stop_execution ? "true" : "false")
           << ", \"abort_mission\": " << (abort_mission ? "true" : "false") << "}";
        msg.data = ss.str();

        publisher_->publish(msg);
    }

} // namespace manymove_cpp_trees
