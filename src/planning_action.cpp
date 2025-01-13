// src/manymove_cpp_trees/src/planning_action.cpp

#include "manymove_cpp_trees/planning_action.hpp"
#include <behaviortree_cpp_v3/blackboard.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <sstream>

namespace manymove_cpp_trees
{

    PlanningAction::PlanningAction(const std::string &name,
                                   const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          goal_sent_(false),
          result_received_(false)
    {
        // Obtain the ROS node from the blackboard
        if (!config.blackboard)
        {
            throw BT::RuntimeError("PlanningAction: no blackboard provided.");
        }
        if (!config.blackboard->get("node", node_))
        {
            throw BT::RuntimeError("PlanningAction: 'node' not found in blackboard.");
        }

        // Initialize the action client
        action_client_ = rclcpp_action::create_client<PlanManipulator>(node_, "plan_manipulator");
        RCLCPP_INFO(node_->get_logger(),
                    "PlanningAction [%s]: waiting up to 5s for 'plan_manipulator' server...",
                    name.c_str());

        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            throw BT::RuntimeError("PlanningAction: 'plan_manipulator' server not available after 5s.");
        }

        RCLCPP_INFO(node_->get_logger(),
                    "PlanningAction [%s]: Constructed with node [%s].",
                    name.c_str(), node_->get_fully_qualified_name());
    }

    BT::NodeStatus PlanningAction::onStart()
    {
        RCLCPP_INFO(node_->get_logger(),
                    "PlanningAction [%s]: onStart() called.", name().c_str());

        goal_sent_ = false;
        result_received_ = false;
        action_result_ = PlanManipulator::Result();

        // Retrieve move_id from input port
        if (!getInput<std::string>("move_id", move_id_))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "PlanningAction [%s]: missing InputPort [move_id].",
                         name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Build blackboard key => "move_{move_id}"
        std::string move_key = "move_" + move_id_;
        std::shared_ptr<Move> move_ptr;
        if (!config().blackboard->get(move_key, move_ptr))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "PlanningAction [%s]: can't find key [%s] in blackboard.",
                         name().c_str(), move_key.c_str());
            return BT::NodeStatus::FAILURE;
        }

        // **Begin: Set start_joint_values based on previous move**
        try
        {
            int current_move_id_int = std::stoi(move_id_);
            if (current_move_id_int > 0)
            {
                int previous_move_id_int = current_move_id_int - 1;
                std::string validity_key = "validity_" + std::to_string(previous_move_id_int);
                bool prev_validity = false;
                if (config().blackboard->get(validity_key, prev_validity) && prev_validity)
                {
                    std::string trajectory_key = "trajectory_" + std::to_string(previous_move_id_int);
                    moveit_msgs::msg::RobotTrajectory previous_traj;
                    if (config().blackboard->get(trajectory_key, previous_traj) &&
                        !previous_traj.joint_trajectory.points.empty())
                    {
                        // Get the last point's joint positions
                        const auto &last_point = previous_traj.joint_trajectory.points.back();
                        move_ptr->start_joint_values = last_point.positions;
                        RCLCPP_INFO(node_->get_logger(),
                                    "PlanningAction [%s]: Set start_joint_values from move_%d's trajectory.",
                                    name().c_str(), previous_move_id_int);
                    }
                    else
                    {
                        // Trajectory missing or empty; use current joint state
                        move_ptr->start_joint_values = {};
                        RCLCPP_WARN(node_->get_logger(),
                                    "PlanningAction [%s]: Previous trajectory '%s' missing or empty. Using current joint state.",
                                    name().c_str(), trajectory_key.c_str());
                    }
                }
                else
                {
                    // Previous move invalid; use current joint state
                    move_ptr->start_joint_values = {};
                    RCLCPP_WARN(node_->get_logger(),
                                "PlanningAction [%s]: Previous move_%d is invalid. Using current joint state.",
                                name().c_str(), previous_move_id_int);
                }
            }
            else
            {
                // First move; use current joint state
                move_ptr->start_joint_values = {};
                RCLCPP_INFO(node_->get_logger(),
                            "PlanningAction [%s]: First move. Using current joint state.",
                            name().c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "PlanningAction [%s]: Error parsing move_id '%s': %s. Using current joint state.",
                         name().c_str(), move_id_.c_str(), e.what());
            move_ptr->start_joint_values = {};
        }
        // **End: Set start_joint_values based on previous move**

        RCLCPP_INFO(node_->get_logger(),
                    "PlanningAction [%s]: sending plan goal => move_id=%s",
                    name().c_str(), move_id_.c_str());

        // Convert Move to goal
        auto move_goal = move_ptr->to_move_manipulator_goal();
        PlanManipulator::Goal goal_msg;
        goal_msg.goal = move_goal;

        // Send goal
        auto send_goal_options = rclcpp_action::Client<PlanManipulator>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&PlanningAction::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&PlanningAction::resultCallback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus PlanningAction::onRunning()
    {
        if (result_received_)
        {
            if (action_result_.success)
            {
                // Set outputs
                setOutput("planned_move_id", move_id_);
                setOutput("trajectory", action_result_.trajectory);
                setOutput("planning_validity", true);

                RCLCPP_INFO(node_->get_logger(),
                            "PlanningAction [%s]: SUCCESS => planning_validity=true",
                            name().c_str());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                setOutput("planning_validity", false);
                RCLCPP_ERROR(node_->get_logger(),
                             "PlanningAction [%s]: FAIL => planning_validity=false",
                             name().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        // still waiting for action result
        return BT::NodeStatus::RUNNING;
    }

    void PlanningAction::onHalted()
    {
        RCLCPP_WARN(node_->get_logger(),
                    "PlanningAction [%s]: onHalted => cancel goal if needed.",
                    name().c_str());

        if (goal_sent_ && !result_received_)
        {
            action_client_->async_cancel_all_goals();
        }
        goal_sent_ = false;
        result_received_ = false;
    }

    void PlanningAction::goalResponseCallback(std::shared_ptr<GoalHandlePlanManipulator> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "PlanningAction [%s]: Goal REJECTED by server.",
                         name().c_str());
            PlanManipulator::Result fail;
            fail.success = false;
            action_result_ = fail;
            result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "PlanningAction [%s]: Goal ACCEPTED by server.",
                        name().c_str());
        }
    }

    void PlanningAction::resultCallback(const GoalHandlePlanManipulator::WrappedResult &wrapped_result)
    {
        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "PlanningAction [%s]: Plan action => SUCCEEDED.",
                        name().c_str());
            action_result_ = *(wrapped_result.result);
            result_received_ = true;
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "PlanningAction [%s]: Plan action => FAILED code=%d",
                         name().c_str(), static_cast<int>(wrapped_result.code));
            PlanManipulator::Result fail;
            fail.success = false;
            action_result_ = fail;
            result_received_ = true;
        }
    }

} // namespace manymove_cpp_trees
