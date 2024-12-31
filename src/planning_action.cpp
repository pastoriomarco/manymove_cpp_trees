// src/manymove_cpp_trees/src/planning_action.cpp

#include "manymove_cpp_trees/planning_action.hpp"

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <memory>

namespace manymove_cpp_trees
{

    PlanningAction::PlanningAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::AsyncActionNode(name, config),
          goal_sent_(false),
          result_received_(false)
    {
        // Obtain the ROS node from the blackboard
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        if (!node_)
        {
            throw BT::RuntimeError("PlanningAction: node not found on blackboard");
        }

        // Initialize the action client
        action_client_ = rclcpp_action::create_client<PlanManipulator>(node_, "plan_manipulator");

        // Wait for the action server to be available
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            throw BT::RuntimeError("PlanningAction: Action server 'plan_manipulator' not available after waiting");
        }
    }

    BT::NodeStatus PlanningAction::tick()
    {
        if (!goal_sent_)
        {
            RCLCPP_INFO(node_->get_logger(), "PlanningAction: Preparing to send goal...");

            // Retrieve the serialized goal from the input port
            std::string goal_str;
            if (!getInput<std::string>("goal", goal_str))
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanningAction: Missing required input [goal]");
                return BT::NodeStatus::FAILURE;
            }

            // Deserialize the goal
            manymove_planner::msg::MoveManipulatorGoal move_goal;
            try
            {
                move_goal = deserializeMoveManipulatorGoal(goal_str);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanningAction: Deserialization error: %s", e.what());
                return BT::NodeStatus::FAILURE;
            }

            // Prepare the action goal
            PlanManipulator::Goal goal_msg;
            goal_msg.goal = move_goal;

            // Send the goal asynchronously
            auto send_goal_options = rclcpp_action::Client<PlanManipulator>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&PlanningAction::goal_response_callback, this, std::placeholders::_1);
            send_goal_options.result_callback = std::bind(&PlanningAction::result_callback, this, std::placeholders::_1);

            action_client_->async_send_goal(goal_msg, send_goal_options);
            goal_sent_ = true;

            RCLCPP_INFO(node_->get_logger(), "PlanningAction: Goal sent successfully.");
            return BT::NodeStatus::RUNNING;
        }

        // Tick does not need to poll for result anymore. Callback handles status updates.
        return BT::NodeStatus::RUNNING;
    }

    void PlanningAction::halt()
    {
        if (goal_sent_ && !result_received_)
        {
            RCLCPP_INFO(node_->get_logger(), "PlanningAction: Halting, cancelling goal.");
            action_client_->async_cancel_all_goals();
        }

        // Reset state
        goal_sent_ = false;
        result_received_ = false;
    }

    void PlanningAction::goal_response_callback(std::shared_ptr<GoalHandlePlanManipulator> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "PlanningAction: Goal was rejected by server.");
            // Simulate result as failure
            PlanManipulator::Result failed_result;
            failed_result.success = false;
            result_ = failed_result;
            result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "PlanningAction: Goal accepted by server.");
        }
    }

    void PlanningAction::feedback_callback(
        GoalHandlePlanManipulator::SharedPtr /*goal_handle*/,
        const std::shared_ptr<const PlanManipulator::Feedback> feedback)
    {
        RCLCPP_INFO(node_->get_logger(), "PlanningAction: Progress: %.2f%%", feedback->progress * 100.0f);
    }

    void PlanningAction::result_callback(const GoalHandlePlanManipulator::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node_->get_logger(), "PlanningAction: Planning succeeded. Result received.");
            this->result_ = *(result.result);
            result_received_ = true;

            // Retrieve the move_id input
            std::string move_id;
            if (!getInput("move_id", move_id))
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanningAction: Missing required input [move_id]");
                setStatus(BT::NodeStatus::FAILURE);
                return;
            }

            // Store trajectory in output port
            if (!setOutput("trajectory", result_.trajectory))
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanningAction: Failed to set 'trajectory' output.");
                setStatus(BT::NodeStatus::FAILURE);
                return;
            }

            // Store move_id in the blackboard for validation
            if (!setOutput("planned_move_id", move_id))
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanningAction: Failed to set 'planned_move_id' output.");
                setStatus(BT::NodeStatus::FAILURE);
                return;
            }

            // Mark trajectory validity
            if (!setOutput("planning_validity", true))
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanningAction: Failed to set 'planning_validity' output.");
                setStatus(BT::NodeStatus::FAILURE);
                return;
            }

            setStatus(BT::NodeStatus::SUCCESS);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "PlanningAction: Planning failed with code %d.", static_cast<int>(result.code));
            PlanManipulator::Result failed_result;
            failed_result.success = false;
            this->result_ = failed_result;
            result_received_ = true;

            // Mark trajectory validity as false
            if (!setOutput("planning_validity", false))
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanningAction: Failed to set 'planning_validity' output.");
            }

            setStatus(BT::NodeStatus::FAILURE);
        }
    }

} // namespace manymove_cpp_trees
