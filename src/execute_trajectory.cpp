// src/manymove_cpp_trees/src/execute_trajectory.cpp

#include "manymove_cpp_trees/execute_trajectory.hpp"
#include "manymove_cpp_trees/behavior_tree_xml_generator.hpp" // Include only if needed

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

        RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory [%s]: Initialized and connected to action server.", name.c_str());
    }

    BT::NodeStatus ExecuteTrajectory::tick()
    {
        // Log the tick call
        RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory [%s]: Tick called.", name().c_str());

        // Retrieve the trajectory from the input port
        moveit_msgs::msg::RobotTrajectory trajectory;
        if (!getInput("trajectory", trajectory))
        {
            RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory [%s]: Missing required input [trajectory]", name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory [%s]: Retrieved trajectory with %zu points.",
                        name().c_str(), trajectory.joint_trajectory.points.size());
            trajectory_ = trajectory; // Store the trajectory for later use
        }

        // Retrieve the planned_move_id from the input port
        std::string planned_move_id;
        if (!getInput("planned_move_id", planned_move_id))
        {
            RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory [%s]: Missing required input [planned_move_id]", name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory [%s]: Retrieved planned_move_id: %s",
                        name().c_str(), planned_move_id.c_str());
        }

        // Retrieve planning validity
        bool planning_validity;
        if (!getInput("planning_validity", planning_validity) || !planning_validity)
        {
            RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory [%s]: Invalid or missing planning validity.", name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory [%s]: Planning validity: %s",
                        name().c_str(), planning_validity ? "true" : "false");
        }

        // Proceed with trajectory execution
        if (!goal_sent_)
        {
            RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory [%s]: Sending trajectory execution goal...", name().c_str());

            // Validate trajectory
            if (trajectory.joint_trajectory.points.empty())
            {
                RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory [%s]: Received empty trajectory.", name().c_str());
                return BT::NodeStatus::FAILURE;
            }

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

            RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory [%s]: Goal sent successfully. Waiting for result...", name().c_str());
            return BT::NodeStatus::RUNNING;
        }

        // If goal has been sent, wait for the result via callbacks
        if (result_received_)
        {
            if (result_.success)
            {
                RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory [%s]: Execution succeeded.", name().c_str());

                // **Set 'planning_validity' to false upon successful execution**
                config().blackboard->set("planning_validity", false);
                RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory [%s]: Set 'planning_validity' to false on blackboard.", name().c_str());

                // **Reset State Flags**
                goal_sent_ = false;
                result_received_ = false;

                // Set node status to SUCCESS
                setStatus(BT::NodeStatus::SUCCESS);
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory [%s]: Execution failed.", name().c_str());

                // **Do not update 'planning_validity' on failure to ensure next planning starts from current position**

                // **Reset State Flags**
                goal_sent_ = false;
                result_received_ = false;

                // Set node status to FAILURE
                setStatus(BT::NodeStatus::FAILURE);
                return BT::NodeStatus::FAILURE;
            }
        }

        // Timeout handling
        auto elapsed = std::chrono::steady_clock::now() - start_time_;
        if (elapsed > std::chrono::seconds(10)) // Timeout after 10 seconds
        {
            RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory [%s]: Timeout waiting for result.", name().c_str());
            // **Reset State Flags**
            goal_sent_ = false;
            result_received_ = false;
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory [%s]: Still waiting for result...", name().c_str());
        return BT::NodeStatus::RUNNING;
    }

    void ExecuteTrajectory::halt()
    {
        RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory [%s]: Halt called.", name().c_str());

        if (goal_sent_ && !result_received_)
        {
            RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory [%s]: Halting, cancelling goal.", name().c_str());
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
            RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory [%s]: Goal was rejected by server.", name().c_str());
            // Simulate result as failure
            ExecuteTrajectoryAction::Result failed_result;
            failed_result.success = false;
            failed_result.message = "Goal rejected by server.";
            result_ = failed_result;
            result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory [%s]: Goal accepted by server.", name().c_str());
        }
    }

    void ExecuteTrajectory::feedback_callback(
        GoalHandleExecuteTrajectory::SharedPtr /*goal_handle*/,
        const std::shared_ptr<const ExecuteTrajectoryAction::Feedback> feedback)
    {
        RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory [%s]: Progress: %.2f%%",
                    name().c_str(), feedback->progress * 100.0f);
    }

    void ExecuteTrajectory::result_callback(const GoalHandleExecuteTrajectory::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory [%s]: Execution succeeded. Result received.", name().c_str());
            this->result_ = *(result.result);
            result_received_ = true;

            // **Set 'planning_validity' to false upon successful execution**
            config().blackboard->set("planning_validity", false);
            RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory [%s]: Set 'planning_validity' to false on blackboard.", name().c_str());

            // **Reset State Flags**
            goal_sent_ = false;
            result_received_ = false;

            // Set node status to SUCCESS
            setStatus(BT::NodeStatus::SUCCESS);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "ExecuteTrajectory [%s]: Execution failed with code %d.", name().c_str(), static_cast<int>(result.code));
            ExecuteTrajectoryAction::Result failed_result;
            failed_result.success = false;
            failed_result.message = "Execution failed.";
            this->result_ = failed_result;
            result_received_ = true;

            // **Do not update 'planning_validity' on failure to ensure next planning starts from current position**

            // **Reset State Flags**
            goal_sent_ = false;
            result_received_ = false;

            // Set node status to FAILURE
            setStatus(BT::NodeStatus::FAILURE);
        }
    }

} // namespace manymove_cpp_trees
