#ifndef MANYMOVE_CPP_TREES_ACTION_NODES_HPP
#define MANYMOVE_CPP_TREES_ACTION_NODES_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <manymove_planner/action/plan_manipulator.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include "manymove_cpp_trees/move.hpp"
#include <manymove_planner/action/execute_trajectory.hpp>
#include <string>
#include <vector>

namespace manymove_cpp_trees
{

    /**
     * @class PlanningAction
     * @brief A stateful BT node that sends a planning request to the plan_manipulator action server.
     *
     * We use StatefulActionNode to illustrate best practices:
     *   - onStart() => send goal
     *   - onRunning() => check result
     *   - onHalted() => cancel if needed
     */
    class PlanningAction : public BT::StatefulActionNode
    {
    public:
        using PlanManipulator = manymove_planner::action::PlanManipulator;
        using GoalHandlePlanManipulator = rclcpp_action::ClientGoalHandle<PlanManipulator>;

        /**
         * @brief Constructor for the PlanningAction node.
         * @param name The name of this BT node.
         * @param config The BT NodeConfiguration (ports, blackboard, etc.).
         */
        PlanningAction(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Define the required/optional ports for this node.
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("move_id", "Unique identifier for the move"),
                BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("trajectory", "Planned trajectory"),
                BT::OutputPort<std::string>("planned_move_id", "Echoes move_id for validation"),
                BT::OutputPort<bool>("planning_validity", "Indicates if planning was successful")};
        }

    protected:
        /**
         * @brief Called once when transitioning from IDLE to RUNNING.
         */
        BT::NodeStatus onStart() override;

        /**
         * @brief Called every tick while in RUNNING state.
         */
        BT::NodeStatus onRunning() override;

        /**
         * @brief Called if this node is halted by force.
         */
        void onHalted() override;

    private:
        // Callbacks for action client
        void goalResponseCallback(std::shared_ptr<GoalHandlePlanManipulator> goal_handle);
        void resultCallback(const GoalHandlePlanManipulator::WrappedResult &result);

        // ROS2 members
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<PlanManipulator>::SharedPtr action_client_;

        // Internal state
        bool goal_sent_;
        bool result_received_;

        PlanManipulator::Result action_result_;
        std::string move_id_; ///< Unique move identifier
    };

    /**
     * @class ExecuteTrajectory
     * @brief A stateful BT node that requests trajectory execution from an action server,
     *        implementing a polling approach if needed.
     *
     * Using StatefulActionNode:
     *   - onStart() => check if data is ready (or poll).
     *   - onRunning() => send goal if data is ready, then wait for result.
     *   - onHalted() => cancel if needed.
     */
    class ExecuteTrajectory : public BT::StatefulActionNode
    {
    public:
        using ExecuteTrajectoryAction = manymove_planner::action::ExecuteTrajectory;
        using GoalHandleExecuteTrajectory = rclcpp_action::ClientGoalHandle<ExecuteTrajectoryAction>;

        ExecuteTrajectory(const std::string &name, const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory", "Planned trajectory"),
                BT::InputPort<std::string>("planned_move_id", "Echoes move_id for validation"),
                BT::InputPort<bool>("planning_validity", "Indicates if planning was successful")
            };
        }

    protected:
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        // Poll data from blackboard
        bool dataReady();
        void sendGoal();

        // Callbacks
        void goalResponseCallback(std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle);
        void resultCallback(const GoalHandleExecuteTrajectory::WrappedResult &result);

        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<ExecuteTrajectoryAction>::SharedPtr action_client_;

        bool goal_sent_;
        bool result_received_;

        // Polled data
        bool planning_valid_;
        std::string move_id_;
        moveit_msgs::msg::RobotTrajectory traj_;

        ExecuteTrajectoryAction::Result action_result_;

        // Timestamps for polling / timeout
        std::chrono::steady_clock::time_point wait_start_time_;
        bool is_data_ready_;

        // Pointer to the blackboard
        BT::Blackboard::Ptr blackboard_;
    };

    /**
     * @class ResetTrajectories
     * @brief A synchronous BT node that resets trajectories and their validity in the blackboard.
     *
     * It takes a comma-separated list of move_ids and for each, sets 'trajectory_{id}' to empty
     * and 'validity_{id}' to false in the blackboard.
     */
    class ResetTrajectories : public BT::SyncActionNode
    {
    public:
        /**
         * @brief Constructor for the ResetTrajectories node.
         * @param name The name of the BT node.
         * @param config The BT NodeConfiguration (ports, blackboard, etc.).
         */
        ResetTrajectories(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Define the required/optional ports for this node.
         */
        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<std::string>("move_ids", "Comma-separated list of move IDs to reset")};
        }

        /**
         * @brief Tick function that performs the reset actions.
         */
        BT::NodeStatus tick() override;

    private:
        // ROS2 node
        rclcpp::Node::SharedPtr node_;
    };

} // namespace manymove_cpp_trees

#endif
