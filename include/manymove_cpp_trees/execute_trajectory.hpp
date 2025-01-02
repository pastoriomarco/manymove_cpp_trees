// include/manymove_cpp_trees/execute_trajectory.hpp

#ifndef MANYMOVE_CPP_TREES_EXECUTE_TRAJECTORY_HPP
#define MANYMOVE_CPP_TREES_EXECUTE_TRAJECTORY_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <manymove_planner/action/execute_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <string>

namespace manymove_cpp_trees
{

    class ExecuteTrajectory : public BT::AsyncActionNode
    {
    public:
        using ExecuteTrajectoryAction = manymove_planner::action::ExecuteTrajectory;
        using GoalHandleExecuteTrajectory = rclcpp_action::ClientGoalHandle<ExecuteTrajectoryAction>;

        ExecuteTrajectory(const std::string &name, const BT::NodeConfiguration &config);

        // Define the ports (inputs and outputs)
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory", "Planned trajectory"),
                BT::InputPort<std::string>("planned_move_id", "Echoes move_id for validation"),
                BT::InputPort<bool>("planning_validity", "Indicates if planning was successful"),
                // BT::OutputPort<bool>("validity", "Indicates if execution was successful")
            };
        }

        // Override the tick function
        BT::NodeStatus tick() override;

        // Override the halt function
        void halt() override;

    private:
        // Callbacks for action client
        void goal_response_callback(std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle);
        void feedback_callback(
            GoalHandleExecuteTrajectory::SharedPtr goal_handle,
            const std::shared_ptr<const ExecuteTrajectoryAction::Feedback> feedback);
        void result_callback(const GoalHandleExecuteTrajectory::WrappedResult &result);

        // ROS 2 node and action client
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<ExecuteTrajectoryAction>::SharedPtr action_client_;

        // State variables
        bool goal_sent_;
        bool result_received_;
        ExecuteTrajectoryAction::Result result_;
        std::chrono::steady_clock::time_point start_time_;
        moveit_msgs::msg::RobotTrajectory trajectory_;
    };

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_EXECUTE_TRAJECTORY_HPP
