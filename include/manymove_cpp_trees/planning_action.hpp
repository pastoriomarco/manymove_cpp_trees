// include/manymove_cpp_trees/planning_action.hpp

#ifndef MANYMOVE_CPP_TREES_PLANNING_ACTION_HPP
#define MANYMOVE_CPP_TREES_PLANNING_ACTION_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <manymove_planner/action/plan_manipulator.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <string>
#include "manymove_cpp_trees/move.hpp"

namespace manymove_cpp_trees
{
    /**
     * @class PlanningAction
     * @brief A BehaviorTree node that sends a planning request to the plan_manipulator action server.
     *
     * This node uses the BehaviorTree.CPP (BT) framework. In its tick() function, it sends
     * a planning goal (derived from a Move object) to a plan_manipulator action server. When
     * the result is received, it sets the output ports accordingly.
     */
    class PlanningAction : public BT::AsyncActionNode
    {
    public:
        using PlanManipulator = manymove_planner::action::PlanManipulator;
        using GoalHandlePlanManipulator = rclcpp_action::ClientGoalHandle<PlanManipulator>;

        /**
         * @brief Constructor for the PlanningAction node.
         * @param name The name of this BT node.
         * @param config The configuration (ports, etc.) used by BehaviorTree.CPP.
         */
        PlanningAction(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Provide BT ports (inputs/outputs) for PlanningAction.
         * @return A list of ports with names and types.
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("move_id", "Unique identifier for the move"),
                BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("trajectory", "Planned trajectory"),
                BT::OutputPort<std::string>("planned_move_id", "Echoes move_id for validation"),
                BT::OutputPort<bool>("planning_validity", "Indicates if planning was successful")};
        }

        /**
         * @brief The main function that is called repeatedly by the BT framework.
         * @return The current status of the node (RUNNING, SUCCESS, or FAILURE).
         */
        BT::NodeStatus tick() override;

        /**
         * @brief Called when the node is halted (e.g., due to a higher-level BT node).
         */
        void halt() override;

    private:
        /**
         * @brief Callback for receiving the response to the goal request.
         * @param goal_handle The handle to the goal.
         */
        void goal_response_callback(std::shared_ptr<GoalHandlePlanManipulator> goal_handle);

        /**
         * @brief Callback for receiving feedback from the plan_manipulator action.
         * @param goal_handle The handle to the goal.
         * @param feedback The feedback message.
         */
        void feedback_callback(
            GoalHandlePlanManipulator::SharedPtr goal_handle,
            const std::shared_ptr<const PlanManipulator::Feedback> feedback);

        /**
         * @brief Callback for receiving the final result of the planning action.
         * @param result The wrapped result containing the success/failure and trajectory.
         */
        void result_callback(const GoalHandlePlanManipulator::WrappedResult &result);

        // ---------------
        // ROS2 Variables
        // ---------------
        rclcpp::Node::SharedPtr node_;                                    ///< Shared pointer to the ROS2 node.
        rclcpp_action::Client<PlanManipulator>::SharedPtr action_client_; ///< Action client for planning.

        // ---------------
        // State Variables
        // ---------------
        bool goal_sent_;       ///< Indicates whether the goal has been sent.
        bool result_received_; ///< Indicates whether the final result has been received.

        PlanManipulator::Result result_;                   ///< Stores the result from the action server.
        std::chrono::steady_clock::time_point start_time_; ///< Start time for measuring action duration.
    };

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_PLANNING_ACTION_HPP
