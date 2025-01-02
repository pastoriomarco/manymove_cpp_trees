// src/manymove_cpp_trees/src/planning_action.cpp

#include "manymove_cpp_trees/planning_action.hpp"

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <memory>
#include <sstream>
#include <fstream>

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

        RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Initialized and connected to action server.", name.c_str());
    }

    BT::NodeStatus PlanningAction::tick()
    {
        // Log the tick call
        RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Tick called.", name().c_str());

        if (!goal_sent_)
        {
            RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Preparing to send goal...", name().c_str());

            // Retrieve the move_id from the input port
            std::string move_id;
            if (!getInput("move_id", move_id))
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Missing required input [move_id]", name().c_str());
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Retrieved move_id: %s", name().c_str(), move_id.c_str());

            // Retrieve the Move struct from the blackboard using move_id
            std::string move_key = "move_" + move_id;
            RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Looking up blackboard key: %s", name().c_str(), move_key.c_str());

            std::shared_ptr<Move> move_ptr;
            if (!config().blackboard->get(move_key, move_ptr))
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Failed to retrieve Move from blackboard with key [%s]", name().c_str(), move_key.c_str());
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Retrieved Move from blackboard.", name().c_str());

            // **Reset 'planning_validity' to false at the start of planning phase**
            config().blackboard->set("planning_validity", false);
            RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Reset 'planning_validity' to false on blackboard.", name().c_str());

            // Retrieve 'last_joint_values' from the blackboard if they exist
            std::vector<double> last_joint_values;
            if (config().blackboard->get("last_joint_values", last_joint_values))
            {
                RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Retrieved 'last_joint_values' from blackboard.", name().c_str());
                move_ptr->start_joint_values = last_joint_values;
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: 'last_joint_values' not found. Planning from current position.", name().c_str());
                // Ensure start_joint_values are empty to plan from current position
                move_ptr->start_joint_values.clear();
            }

            // Convert Move to MoveManipulatorGoal
            manymove_planner::msg::MoveManipulatorGoal move_goal = move_ptr->to_move_manipulator_goal();

            // Prepare the action goal
            PlanManipulator::Goal goal_msg;
            goal_msg.goal = move_goal;

            // Send the goal asynchronously
            auto send_goal_options = rclcpp_action::Client<PlanManipulator>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&PlanningAction::goal_response_callback, this, std::placeholders::_1);
            send_goal_options.result_callback = std::bind(&PlanningAction::result_callback, this, std::placeholders::_1);
            send_goal_options.feedback_callback = std::bind(&PlanningAction::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

            action_client_->async_send_goal(goal_msg, send_goal_options);
            goal_sent_ = true;

            RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Goal sent successfully.", name().c_str());
            return BT::NodeStatus::RUNNING;
        }

        // If goal has been sent, wait for the result via callbacks
        if (result_received_)
        {
            if (result_.success)
            {
                RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Planning succeeded.", name().c_str());

                // Retrieve the move_id again to echo it
                std::string move_id;
                if (!getInput("move_id", move_id))
                {
                    RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Missing required input [move_id]", name().c_str());
                    return BT::NodeStatus::FAILURE;
                }

                // Define unique blackboard keys based on move_id
                std::string traj_key = "traj_" + move_id;
                std::string planned_move_id_key = "planned_move_id_" + move_id;
                std::string planning_validity_key = "planning_validity_" + move_id;

                // Set outputs via setOutput
                auto traj_set = setOutput("trajectory", result_.trajectory);
                if (!traj_set)
                {
                    RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Failed to set 'trajectory' output.", name().c_str());
                    return BT::NodeStatus::FAILURE;
                }
                RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Successfully set 'trajectory'.", name().c_str());

                auto planned_move_id_set = setOutput("planned_move_id", move_id);
                if (!planned_move_id_set)
                {
                    RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Failed to set 'planned_move_id' output.", name().c_str());
                    return BT::NodeStatus::FAILURE;
                }
                RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Successfully set 'planned_move_id'.", name().c_str());

                auto planning_validity_set = setOutput("planning_validity", true);
                if (!planning_validity_set)
                {
                    RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Failed to set 'planning_validity' output.", name().c_str());
                    return BT::NodeStatus::FAILURE;
                }
                RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Successfully set 'planning_validity'.", name().c_str());

                // **Set 'last_joint_values' based on the planned trajectory**
                if (!result_.trajectory.joint_trajectory.points.empty())
                {
                    std::vector<double> last_joint_values = result_.trajectory.joint_trajectory.points.back().positions;
                    config().blackboard->set("last_joint_values", last_joint_values);
                    RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Set 'last_joint_values' on blackboard.", name().c_str());
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Planned trajectory has no points.", name().c_str());
                    return BT::NodeStatus::FAILURE;
                }

                // **Reset State Flags**
                goal_sent_ = false;
                result_received_ = false;

                setStatus(BT::NodeStatus::SUCCESS);
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Planning failed.", name().c_str());

                // Retrieve the move_id to set planning_validity to false
                std::string move_id;
                if (!getInput("move_id", move_id))
                {
                    RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Missing required input [move_id]", name().c_str());
                    return BT::NodeStatus::FAILURE;
                }

                // Set 'planning_validity' to false
                auto planning_validity_set = setOutput("planning_validity", false);
                if (!planning_validity_set)
                {
                    RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Failed to set 'planning_validity' output.", name().c_str());
                    return BT::NodeStatus::FAILURE;
                }
                RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Successfully set 'planning_validity' to false.", name().c_str());

                // **Reset State Flags**
                goal_sent_ = false;
                result_received_ = false;

                setStatus(BT::NodeStatus::FAILURE);
            }
        }

        RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Still waiting for result...", name().c_str());
        return BT::NodeStatus::RUNNING;
    }

    void PlanningAction::halt()
    {
        RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Halt called.", name().c_str());

        if (goal_sent_ && !result_received_)
        {
            RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Halting, cancelling goal.", name().c_str());
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
            RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Goal was rejected by server.", name().c_str());
            // Simulate result as failure
            PlanManipulator::Result failed_result;
            failed_result.success = false;
            // failed_result.message = "Goal rejected by server.";
            result_ = failed_result;
            result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Goal accepted by server.", name().c_str());
        }
    }

    void PlanningAction::feedback_callback(
        GoalHandlePlanManipulator::SharedPtr /*goal_handle*/,
        const std::shared_ptr<const PlanManipulator::Feedback> feedback)
    {
        RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Progress: %.2f%%", name().c_str(), feedback->progress * 100.0f);
    }

    void PlanningAction::result_callback(const GoalHandlePlanManipulator::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Planning succeeded. Result received.", name().c_str());
            this->result_ = *(result.result);
            result_received_ = true;

            std::string move_id;
            if (!getInput("move_id", move_id))
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Missing required input [move_id]", name().c_str());
                setStatus(BT::NodeStatus::FAILURE);
                // Reset state flags to allow next planning
                goal_sent_ = false;
                result_received_ = false;
                return;
            }

            // Set 'trajectory' output
            auto traj_set = setOutput("trajectory", result_.trajectory);
            if (!traj_set)
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Failed to set 'trajectory' output.", name().c_str());
                setStatus(BT::NodeStatus::FAILURE);
                // Reset state flags to allow next planning
                goal_sent_ = false;
                result_received_ = false;
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Successfully set 'trajectory'.", name().c_str());

            // Set 'planned_move_id' output
            auto planned_move_id_set = setOutput("planned_move_id", move_id);
            if (!planned_move_id_set)
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Failed to set 'planned_move_id' output.", name().c_str());
                setStatus(BT::NodeStatus::FAILURE);
                // Reset state flags to allow next planning
                goal_sent_ = false;
                result_received_ = false;
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Successfully set 'planned_move_id'.", name().c_str());

            // Set 'planning_validity' output
            auto planning_validity_set = setOutput("planning_validity", true);
            if (!planning_validity_set)
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Failed to set 'planning_validity' output.", name().c_str());
                setStatus(BT::NodeStatus::FAILURE);
                // Reset state flags to allow next planning
                goal_sent_ = false;
                result_received_ = false;
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Successfully set 'planning_validity'.", name().c_str());

            // **Set 'last_joint_values' based on the planned trajectory**
            if (!result_.trajectory.joint_trajectory.points.empty())
            {
                std::vector<double> last_joint_values = result_.trajectory.joint_trajectory.points.back().positions;
                config().blackboard->set("last_joint_values", last_joint_values);
                RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Set 'last_joint_values' on blackboard.", name().c_str());
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Planned trajectory has no points.", name().c_str());
                setStatus(BT::NodeStatus::FAILURE);
                // Reset state flags to allow next planning
                goal_sent_ = false;
                result_received_ = false;
                return;
            }

            // **Reset State Flags**
            goal_sent_ = false;
            result_received_ = false;

            setStatus(BT::NodeStatus::SUCCESS);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Planning failed with code %d.", name().c_str(), static_cast<int>(result.code));
            auto planning_validity_set = setOutput("planning_validity", false);
            if (!planning_validity_set)
            {
                RCLCPP_ERROR(node_->get_logger(), "PlanningAction [%s]: Failed to set 'planning_validity' output.", name().c_str());
                setStatus(BT::NodeStatus::FAILURE);
                // Reset state flags to allow next planning
                goal_sent_ = false;
                result_received_ = false;
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "PlanningAction [%s]: Successfully set 'planning_validity' to false.", name().c_str());

            // **Reset State Flags**
            goal_sent_ = false;
            result_received_ = false;

            setStatus(BT::NodeStatus::FAILURE);
        }
    }

} // namespace manymove_cpp_trees
