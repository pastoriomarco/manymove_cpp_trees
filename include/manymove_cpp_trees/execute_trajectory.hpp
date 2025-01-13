#ifndef MANYMOVE_CPP_TREES_EXECUTE_TRAJECTORY_HPP
#define MANYMOVE_CPP_TREES_EXECUTE_TRAJECTORY_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <manymove_planner/action/execute_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <string>
#include <vector> // Added for joint positions

namespace manymove_cpp_trees
{

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
                // Removed any reference to sequence_id
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

} // namespace manymove_cpp_trees

#endif
