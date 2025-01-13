#include "manymove_cpp_trees/execute_trajectory.hpp"
#include <behaviortree_cpp_v3/blackboard.h>
#include <rclcpp/rclcpp.hpp>

namespace manymove_cpp_trees
{

    ExecuteTrajectory::ExecuteTrajectory(const std::string &name,
                                         const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          goal_sent_(false),
          result_received_(false),
          is_data_ready_(false)
    {
        if (!config.blackboard)
        {
            throw BT::RuntimeError(
                "ExecuteTrajectory constructor: no blackboard provided.");
        }
        if (!config.blackboard->get("node", node_))
        {
            throw BT::RuntimeError(
                "ExecuteTrajectory constructor: 'node' not found in blackboard.");
        }

        action_client_ = rclcpp_action::create_client<ExecuteTrajectoryAction>(node_, "execute_manipulator_traj");
        RCLCPP_INFO(node_->get_logger(),
                    "ExecuteTrajectory [%s]: waiting 5s for 'execute_manipulator_traj' server...",
                    name.c_str());
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            throw BT::RuntimeError(
                "ExecuteTrajectory: 'execute_manipulator_traj' not available after 5s.");
        }

        RCLCPP_INFO(node_->get_logger(),
                    "ExecuteTrajectory [%s]: Constructed with node [%s].",
                    name.c_str(), node_->get_fully_qualified_name());

        // Store blackboard pointer
        blackboard_ = config.blackboard;
    }

    BT::NodeStatus ExecuteTrajectory::onStart()
    {
        RCLCPP_INFO(node_->get_logger(),
                    "ExecuteTrajectory [%s]: onStart() called.",
                    name().c_str());

        goal_sent_ = false;
        result_received_ = false;
        action_result_ = ExecuteTrajectoryAction::Result();
        is_data_ready_ = false;

        // Start polling
        wait_start_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(),
                    "ExecuteTrajectory [%s]: Polling for 'planned_move_id', 'trajectory', 'planning_validity'...",
                    name().c_str());

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus ExecuteTrajectory::onRunning()
    {
        // If we haven't determined data is ready, poll
        if (!is_data_ready_)
        {
            if (!dataReady())
            {
                // not ready -> keep polling
                auto elapsed = std::chrono::steady_clock::now() - wait_start_time_;
                if (elapsed > std::chrono::seconds(10))
                {
                    RCLCPP_ERROR(node_->get_logger(),
                                 "ExecuteTrajectory [%s]: Timed out (10s) waiting for plan data.",
                                 name().c_str());
                    return BT::NodeStatus::FAILURE;
                }
                else
                {
                    RCLCPP_INFO(node_->get_logger(),
                                "ExecuteTrajectory [%s]: Still polling => RUNNING",
                                name().c_str());
                    return BT::NodeStatus::RUNNING;
                }
            }
            else
            {
                // data is ready => send goal
                sendGoal();
                is_data_ready_ = true;
                return BT::NodeStatus::RUNNING;
            }
        }

        // If the goal was sent, check if we have the result
        if (result_received_)
        {
            if (action_result_.success)
            {
                RCLCPP_INFO(node_->get_logger(),
                            "ExecuteTrajectory [%s]: Execution SUCCEEDED => SUCCESS.",
                            name().c_str());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "ExecuteTrajectory [%s]: Execution FAILED => FAILURE.",
                             name().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        return BT::NodeStatus::RUNNING; // still waiting for final result
    }

    void ExecuteTrajectory::onHalted()
    {
        RCLCPP_WARN(node_->get_logger(),
                    "ExecuteTrajectory [%s]: onHalted => cancel goal if needed.",
                    name().c_str());

        if (goal_sent_ && !result_received_)
        {
            action_client_->async_cancel_all_goals();
        }
        goal_sent_ = false;
        result_received_ = false;
        is_data_ready_ = false;
    }

    bool ExecuteTrajectory::dataReady()
    {
        // Attempt to read planned_move_id
        std::string pmid;
        if (!getInput<std::string>("planned_move_id", pmid) || pmid.empty())
        {
            RCLCPP_INFO(node_->get_logger(),
                        "ExecuteTrajectory [%s]: 'planned_move_id' not set => poll again",
                        name().c_str());
            return false;
        }
        move_id_ = pmid;

        // Attempt to read planning_validity
        bool validity = false;
        if (!getInput<bool>("planning_validity", validity) || !validity)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "ExecuteTrajectory [%s]: planning_validity=false => poll again",
                        name().c_str());
            return false;
        }
        planning_valid_ = validity;

        // Attempt to read trajectory
        moveit_msgs::msg::RobotTrajectory t;
        if (!getInput<moveit_msgs::msg::RobotTrajectory>("trajectory", t) ||
            t.joint_trajectory.points.empty())
        {
            RCLCPP_INFO(node_->get_logger(),
                        "ExecuteTrajectory [%s]: 'trajectory' missing/empty => poll again",
                        name().c_str());
            return false;
        }
        traj_ = t;

        // If all are present, we are ready
        RCLCPP_INFO(node_->get_logger(),
                    "ExecuteTrajectory [%s]: Data is ready => planned_move_id=%s, valid=%s, %zu pts",
                    name().c_str(), move_id_.c_str(), (planning_valid_ ? "true" : "false"),
                    traj_.joint_trajectory.points.size());
        return true;
    }

    void ExecuteTrajectory::sendGoal()
    {
        if (goal_sent_)
        {
            return;
        }
        RCLCPP_INFO(node_->get_logger(),
                    "ExecuteTrajectory [%s]: Sending ExecuteTrajectory goal => move_id=%s",
                    name().c_str(), move_id_.c_str());

        ExecuteTrajectoryAction::Goal goal_msg;
        goal_msg.trajectory = traj_;

        auto opts = rclcpp_action::Client<ExecuteTrajectoryAction>::SendGoalOptions();
        opts.goal_response_callback =
            std::bind(&ExecuteTrajectory::goalResponseCallback, this, std::placeholders::_1);
        opts.result_callback =
            std::bind(&ExecuteTrajectory::resultCallback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, opts);
        goal_sent_ = true;
    }

    void ExecuteTrajectory::goalResponseCallback(std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "ExecuteTrajectory [%s]: Goal REJECTED by server.",
                         name().c_str());
            ExecuteTrajectoryAction::Result fail;
            fail.success = false;
            fail.message = "Goal rejected by server.";
            action_result_ = fail;
            result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "ExecuteTrajectory [%s]: Goal ACCEPTED by server => waiting for result.",
                        name().c_str());
        }
    }

    void ExecuteTrajectory::resultCallback(const GoalHandleExecuteTrajectory::WrappedResult &wrapped_result)
    {
        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "ExecuteTrajectory [%s]: Execution => SUCCEEDED.",
                        name().c_str());
            action_result_ = *(wrapped_result.result);
            result_received_ = true;

            // Retrieve the last joint positions from the trajectory
            if (!traj_.joint_trajectory.points.empty())
            {
                auto last_point = traj_.joint_trajectory.points.back();
                std::vector<double> end_joint_positions = last_point.positions;

                // Set 'current_joint_positions' in blackboard
                blackboard_->set("current_joint_positions", end_joint_positions);
                RCLCPP_INFO(node_->get_logger(),
                            "ExecuteTrajectory [%s]: Updated 'current_joint_positions' on blackboard.",
                            name().c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "ExecuteTrajectory [%s]: Execution => code=%d => FAIL.",
                         name().c_str(), static_cast<int>(wrapped_result.code));
            ExecuteTrajectoryAction::Result fail;
            fail.success = false;
            fail.message = "Execution failed.";
            action_result_ = fail;
            result_received_ = true;
        }
    }

} // namespace manymove_cpp_trees
