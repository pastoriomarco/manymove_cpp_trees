#ifndef MANYMOVE_CPP_TREES_PLANNING_ACTION_HPP
#define MANYMOVE_CPP_TREES_PLANNING_ACTION_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <manymove_planner/action/plan_manipulator.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include "manymove_cpp_trees/move.hpp"

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

} // namespace manymove_cpp_trees

#endif
