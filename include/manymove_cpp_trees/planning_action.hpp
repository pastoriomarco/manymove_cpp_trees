// src/manymove_cpp_trees/include/manymove_cpp_trees/planning_action.hpp

#ifndef MANYMOVE_CPP_TREES_PLANNING_ACTION_HPP
#define MANYMOVE_CPP_TREES_PLANNING_ACTION_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <manymove_planner/action/plan_manipulator.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <string>
#include "manymove_cpp_trees/serialization_helper.hpp"
#include "manymove_cpp_trees/move.hpp"

namespace manymove_cpp_trees
{

class PlanningAction : public BT::AsyncActionNode
{
public:
    using PlanManipulator = manymove_planner::action::PlanManipulator;
    using GoalHandlePlanManipulator = rclcpp_action::ClientGoalHandle<PlanManipulator>;

    PlanningAction(const std::string& name, const BT::NodeConfiguration& config);

    // Define the ports (inputs and outputs)
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("goal"),
            BT::InputPort<std::string>("move_id"), // move_id input port
            BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("trajectory"),
            BT::OutputPort<std::string>("planned_move_id"),
            BT::OutputPort<bool>("planning_validity")
        };
    }

    // Override the tick function
    BT::NodeStatus tick() override;

    // Override the halt function
    void halt() override;

private:
    // Callbacks for action client
    void goal_response_callback(std::shared_ptr<GoalHandlePlanManipulator> goal_handle);
    void feedback_callback(
        GoalHandlePlanManipulator::SharedPtr goal_handle,
        const std::shared_ptr<const PlanManipulator::Feedback> feedback);
    void result_callback(const GoalHandlePlanManipulator::WrappedResult& result);

    // ROS 2 node and action client
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<PlanManipulator>::SharedPtr action_client_;

    // State variables
    bool goal_sent_;
    bool result_received_;
    PlanManipulator::Result result_;
    std::chrono::steady_clock::time_point start_time_; 
};

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_PLANNING_ACTION_HPP
