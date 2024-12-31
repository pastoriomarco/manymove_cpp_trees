// src/manymove_cpp_trees/src/execute_trajectory.cpp

#include "manymove_cpp_trees/execute_trajectory.hpp"

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <memory>

namespace manymove_cpp_trees
{

    ExecuteTrajectory::ExecuteTrajectory(const std::string &name, const BT::NodeConfiguration &config)
        : BT::AsyncActionNode(name, config),
          goal_sent_(false),
          result_received_(false)
    {
        // Obtain the ROS node from the blackboard
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        if (!node_)
        {
            throw BT::RuntimeError("ExecuteTrajectory: node not found on blackboard");
        }

        // Initialize the action client
        action_client_ = rclcpp_action::create_client<ExecuteTrajectoryAction>(node_, "execute_manipulator_traj");

        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            throw BT::RuntimeError("ExecuteTrajectory: Action server 'execute_manipulator_traj' not available after waiting");
        }
    }

    BT::NodeStatus ExecuteTrajectory::tick()
    {
        RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory: Fetching trajectory and planned_move_id from blackboard.");

        // Retrieve the trajectory from the input port
        moveit_msgs::msg::RobotTrajectory trajectory;
        if (!getInput("trajectory", trajectory))
        {
            RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory: Missing required input [trajectory]");
            return BT::NodeStatus::FAILURE;
        }

        // Retrieve the planned_move_id for this execution
        std::string planned_move_id;
        if (!getInput("planned_move_id", planned_move_id))
        {
            RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory: Missing required input [planned_move_id]");
            return BT::NodeStatus::FAILURE;
        }

        // Retrieve planning validity
        bool planning_validity;
        if (!getInput("planning_validity", planning_validity) || !planning_validity)
        {
            RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory: Invalid or missing planning validity.");
            return BT::NodeStatus::FAILURE;
        }

        // Proceed with trajectory execution
        RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory: Trajectory fetched with %zu points.",
                    trajectory.joint_trajectory.points.size());
        if (!goal_sent_)
        {
            RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory: Sending trajectory execution goal...");

            // Validate trajectory
            if (trajectory.joint_trajectory.points.empty())
            {
                RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory: Received empty trajectory.");
                return BT::NodeStatus::FAILURE;
            }

            // Log trajectory details
            RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory: Retrieved trajectory with %zu points.", trajectory.joint_trajectory.points.size());

            // Create the action goal
            ExecuteTrajectoryAction::Goal goal_msg;
            goal_msg.trajectory = trajectory;

            // Send the goal asynchronously
            auto send_goal_options = rclcpp_action::Client<ExecuteTrajectoryAction>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                std::bind(&ExecuteTrajectory::goal_response_callback, this, std::placeholders::_1);
            send_goal_options.feedback_callback =
                std::bind(&ExecuteTrajectory::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback =
                std::bind(&ExecuteTrajectory::result_callback, this, std::placeholders::_1);

            action_client_->async_send_goal(goal_msg, send_goal_options);
            goal_sent_ = true;

            // Record start time for timeout
            start_time_ = std::chrono::steady_clock::now();

            RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory: Goal sent. Waiting for result...");
            return BT::NodeStatus::RUNNING;
        }

        if (result_received_)
        {
            if (result_.success)
            {
                RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory: Execution succeeded.");
                // Store execution validity in output port
                if (!setOutput("validity", true))
                {
                    RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory: Failed to set 'validity' output.");
                    return BT::NodeStatus::FAILURE;
                }
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory: Execution failed.");
                // Store execution validity in output port
                if (!setOutput("validity", false))
                {
                    RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory: Failed to set 'validity' output.");
                }
                return BT::NodeStatus::FAILURE;
            }
        }

        // Timeout handling
        auto elapsed = std::chrono::steady_clock::now() - start_time_;
        if (elapsed > std::chrono::seconds(10)) // Timeout after 10 seconds
        {
            RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory: Timeout waiting for result.");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory: Still waiting for result...");
        return BT::NodeStatus::RUNNING;
    }

    void ExecuteTrajectory::halt()
    {
        if (goal_sent_ && !result_received_)
        {
            RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory: Halting, cancelling goal.");
            action_client_->async_cancel_all_goals();
        }

        // Reset state
        goal_sent_ = false;
        result_received_ = false;
    }

    void ExecuteTrajectory::goal_response_callback(std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory: Goal was rejected by server.");
            // Simulate result as failure
            ExecuteTrajectoryAction::Result failed_result;
            failed_result.success = false;
            result_ = failed_result;
            result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory: Goal accepted by server.");
        }
    }

    void ExecuteTrajectory::feedback_callback(
        GoalHandleExecuteTrajectory::SharedPtr /*goal_handle*/,
        const std::shared_ptr<const ExecuteTrajectoryAction::Feedback> feedback)
    {
        RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory: Progress: %.2f%%", feedback->progress * 100.0f);
    }

    void ExecuteTrajectory::result_callback(const GoalHandleExecuteTrajectory::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory: Execution succeeded. Result received.");
            this->result_ = *(result.result);
            result_received_ = true;

            // Store execution validity in output port
            if (!setOutput("validity", true))
            {
                RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory: Failed to set 'validity' output.");
                setStatus(BT::NodeStatus::FAILURE);
                return;
            }

            setStatus(BT::NodeStatus::SUCCESS);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory: Execution failed with code %d.", static_cast<int>(result.code));
            ExecuteTrajectoryAction::Result failed_result;
            failed_result.success = false;
            this->result_ = failed_result;
            result_received_ = true;

            // Mark execution validity as false
            if (!setOutput("validity", false))
            {
                RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory: Failed to set 'validity' output.");
            }

            setStatus(BT::NodeStatus::FAILURE);
        }
    }

} // namespace manymove_cpp_trees
