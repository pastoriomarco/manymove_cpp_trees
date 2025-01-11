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
    /**
     * @class ExecuteTrajectory
     * @brief A BehaviorTree node for sending a trajectory execution request to execute_manipulator_traj action server.
     *
     * This node sends the provided RobotTrajectory to an action server that handles
     * execution on the real or simulated robot. It uses feedback and result callbacks
     * to determine success or failure, and it can be halted (canceling the active goal).
     */
    class ExecuteTrajectory : public BT::AsyncActionNode
    {
    public:
        using ExecuteTrajectoryAction = manymove_planner::action::ExecuteTrajectory;
        using GoalHandleExecuteTrajectory = rclcpp_action::ClientGoalHandle<ExecuteTrajectoryAction>;

        /**
         * @brief Constructor for ExecuteTrajectory node.
         * @param name The name of this BT node.
         * @param config The configuration for BehaviorTree.CPP.
         */
        ExecuteTrajectory(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Provide the list of BT ports required/produced by this node.
         * @return A list of ports, describing inputs for the trajectory, planned_move_id, etc.
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory", "Planned trajectory"),
                BT::InputPort<std::string>("planned_move_id", "Echoes move_id for validation"),
                BT::InputPort<bool>("planning_validity", "Indicates if planning was successful")};
        }

        /**
         * @brief The main function called repeatedly by the BT framework.
         * @return RUNNING, SUCCESS, or FAILURE depending on the current action status.
         */
        BT::NodeStatus tick() override;

        /**
         * @brief Called to halt this node, triggering a cancel of the active goal if any.
         */
        void halt() override;

    private:
        /**
         * @brief Callback when the action server responds to the goal request.
         * @param goal_handle The handle to the goal, or nullptr if rejected.
         */
        void goal_response_callback(std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle);

        /**
         * @brief Callback for action feedback, providing partial progress updates.
         * @param goal_handle The handle to the goal.
         * @param feedback The feedback message, including progress percentage.
         */
        void feedback_callback(
            GoalHandleExecuteTrajectory::SharedPtr goal_handle,
            const std::shared_ptr<const ExecuteTrajectoryAction::Feedback> feedback);

        /**
         * @brief Callback for the final result of the trajectory execution action.
         * @param result The wrapped result containing success/failure info.
         */
        void result_callback(const GoalHandleExecuteTrajectory::WrappedResult &result);

        // ----------------
        // ROS2 Variables
        // ----------------
        rclcpp::Node::SharedPtr node_;                                            ///< Shared pointer to the ROS2 node.
        rclcpp_action::Client<ExecuteTrajectoryAction>::SharedPtr action_client_; ///< Action client for execution.

        // ----------------
        // State Variables
        // ----------------
        bool goal_sent_;                                   ///< Indicates if a goal has been sent.
        bool result_received_;                             ///< Indicates if the final result has been received.
        ExecuteTrajectoryAction::Result result_;           ///< Stores the result from the action server.
        std::chrono::steady_clock::time_point start_time_; ///< Start time for measuring the execution duration.
        moveit_msgs::msg::RobotTrajectory trajectory_;     ///< Local copy of the trajectory to be executed.
    };

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_EXECUTE_TRAJECTORY_HPP
