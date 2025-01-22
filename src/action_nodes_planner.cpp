#include "manymove_cpp_trees/action_nodes_planner.hpp"
#include <behaviortree_cpp_v3/blackboard.h>

#include <memory>
#include <stdexcept>

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
                        // Trajectory missing or empty; use empty joint state
                        move_ptr->start_joint_values = {};
                        RCLCPP_WARN(node_->get_logger(),
                                    "PlanningAction [%s]: Previous trajectory '%s' missing or empty. Using empty joint state.",
                                    name().c_str(), trajectory_key.c_str());
                    }
                }
                else
                {
                    // Previous move invalid; use empty joint state
                    move_ptr->start_joint_values = {};
                    RCLCPP_WARN(node_->get_logger(),
                                "PlanningAction [%s]: Previous move_%d is invalid. Using empty joint state.",
                                name().c_str(), previous_move_id_int);
                }
            }
            else
            {
                // First move; use empty joint state
                move_ptr->start_joint_values = {};
                RCLCPP_INFO(node_->get_logger(),
                            "PlanningAction [%s]: First move. Using empty joint state.",
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

        geometry_msgs::msg::Pose dynamic_pose;
        manymove_planner::msg::MoveManipulatorGoal move_goal;

        // Assign Move to goal
        move_goal = move_ptr->to_move_manipulator_goal();

        // **Begin: Retrieve dynamic pose using pose_key**
        if (move_ptr->type == "pose" || move_ptr->type == "cartesian")
        {
            pose_key_ = move_ptr->pose_key;
            if (!config().blackboard->get(pose_key_, dynamic_pose))
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "PlanningAction [%s]: Failed to retrieve pose from blackboard key '%s'.",
                             name().c_str(), pose_key_.c_str());
                return BT::NodeStatus::FAILURE;
            }

            // Log the retrieved pose
            RCLCPP_DEBUG(node_->get_logger(),
                         "PlanningAction [%s]: Retrieved pose from '%s' - Position (%.3f, %.3f, %.3f), Orientation (%.3f, %.3f, %.3f, %.3f)",
                         name().c_str(), pose_key_.c_str(),
                         dynamic_pose.position.x, dynamic_pose.position.y, dynamic_pose.position.z,
                         dynamic_pose.orientation.x, dynamic_pose.orientation.y, dynamic_pose.orientation.z, dynamic_pose.orientation.w);

            // Assign the dynamic pose to the goal
            move_goal.pose_target = dynamic_pose;

            RCLCPP_INFO(node_->get_logger(),
                        "PlanningAction [%s]: Final move_goal.pose_target set to Position (%.3f, %.3f, %.3f), Orientation (%.3f, %.3f, %.3f, %.3f)",
                        name().c_str(),
                        move_goal.pose_target.position.x, move_goal.pose_target.position.y, move_goal.pose_target.position.z,
                        move_goal.pose_target.orientation.x, move_goal.pose_target.orientation.y, move_goal.pose_target.orientation.z, move_goal.pose_target.orientation.w);
        }
        // **End: Retrieve dynamic pose using pose_key**

        RCLCPP_INFO(node_->get_logger(),
                    "PlanningAction [%s]: sending plan goal => move_id=%s",
                    name().c_str(), move_id_.c_str());

        // Assign goal message to send
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

        /**
         * The stop_motion action servers takes as input any traj and just stops the motion of the manipulator
         * by overriding the current trajectory execution by traj_controller with the current position,
         * zero velocity, and deceleration time. The robot will try to "spring back" to the position it was
         * when the stop command is issued within the deceleration time. The higher the time, the smoother
         * the stop, but the higher the move lenght to decelerate and come back to the stop point.
         * At current time, the deceleration_time is hardcoded in the manymove_planner in execute_stop() function
         * inside the action_server implementation.
         */
        stop_client_ = rclcpp_action::create_client<ExecuteTrajectoryAction>(node_, "stop_motion");
        RCLCPP_INFO(node_->get_logger(),
                    "ExecuteTrajectory [%s]: waiting 5s for 'stop_motion' server...",
                    name.c_str());
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            throw BT::RuntimeError(
                "ExecuteTrajectory: 'stop_motion' not available after 5s.");
        }

        RCLCPP_INFO(node_->get_logger(),
                    "ExecuteTrajectory [%s]: Constructed with node [%s].",
                    name.c_str(), node_->get_fully_qualified_name());
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
        RCLCPP_DEBUG(node_->get_logger(),
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
                int max_time = 64;
                if (elapsed > std::chrono::seconds(max_time))
                {
                    RCLCPP_ERROR(node_->get_logger(),
                                 "ExecuteTrajectory [%s]: Timed out (%is) waiting for plan data.",
                                 name().c_str(),
                                 max_time);
                    return BT::NodeStatus::FAILURE;
                }
                else
                {
                    RCLCPP_DEBUG(node_->get_logger(),
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
            // Invalidate trajectory after execution, regardless of result
            setOutput("trajectory", moveit_msgs::msg::RobotTrajectory());
            setOutput("planning_validity", false);

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

        // Cancel any in-progress "execute_manipulator_traj" goal
        if (goal_sent_ && !result_received_)
        {
            action_client_->async_cancel_all_goals();
        }
        goal_sent_ = false;
        result_received_ = false;
        is_data_ready_ = false;

        // Invalidate trajectory on halt
        setOutput("trajectory", moveit_msgs::msg::RobotTrajectory());
        setOutput("planning_validity", false);

        // Reset blackboard on halt
        std::string validity_key = "validity_" + move_id_;
        config().blackboard->set(validity_key, false);

        std::string trajectory_key = "trajectory_" + move_id_;
        moveit_msgs::msg::RobotTrajectory empty_traj;
        config().blackboard->set(trajectory_key, empty_traj);

        // *** Call STOP_MOTION server ***
        if (!stop_client_)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "ExecuteTrajectory [%s]: stop_motion_client_ is not initialized. Cannot stop motion.",
                        name().c_str());
            return;
        }

        // Create an empty goal (we won't send a real trajectory)
        manymove_planner::action::ExecuteTrajectory::Goal stop_goal;
        // e.g. stop_goal.trajectory is empty

        RCLCPP_INFO(node_->get_logger(),
                    "ExecuteTrajectory [%s]: Sending STOP goal to 'stop_motion' server...",
                    name().c_str());

        auto send_goal_options = rclcpp_action::Client<manymove_planner::action::ExecuteTrajectory>::SendGoalOptions();

        // Optional: add result callback to see if STOP completed
        send_goal_options.result_callback =
            [this](const typename rclcpp_action::ClientGoalHandle<
                   manymove_planner::action::ExecuteTrajectory>::WrappedResult &wrapped_result)
        {
            if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(node_->get_logger(), "StopMotion => SUCCEEDED: %s",
                            wrapped_result.result->message.c_str());
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "StopMotion => code=%d => message: %s",
                            static_cast<int>(wrapped_result.code),
                            wrapped_result.result ? wrapped_result.result->message.c_str() : "");
            }
        };

        // Send goal asynchronously
        stop_client_->async_send_goal(stop_goal, send_goal_options);
    }

    bool ExecuteTrajectory::dataReady()
    {
        // Attempt to read planned_move_id
        std::string pmid;
        if (!getInput<std::string>("planned_move_id", pmid) || pmid.empty())
        {
            RCLCPP_DEBUG(node_->get_logger(),
                         "ExecuteTrajectory [%s]: 'planned_move_id' not set => poll again",
                         name().c_str());
            return false;
        }
        move_id_ = pmid;

        // Attempt to read planning_validity
        bool validity = false;
        if (!getInput<bool>("planning_validity", validity) || !validity)
        {
            RCLCPP_DEBUG(node_->get_logger(),
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
            RCLCPP_DEBUG(node_->get_logger(),
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
        // **Begin: Update blackboard before returning**
        std::string validity_key = "validity_" + move_id_;
        config().blackboard->set(validity_key, false);

        std::string trajectory_key = "trajectory_" + move_id_;
        moveit_msgs::msg::RobotTrajectory empty_traj;
        config().blackboard->set(trajectory_key, empty_traj);
        // **End: Update blackboard before returning**

        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "ExecuteTrajectory [%s]: Execution => SUCCEEDED.",
                        name().c_str());
            action_result_ = *(wrapped_result.result);
            result_received_ = true;
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

    ResetTrajectories::ResetTrajectories(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        // Obtain the ROS node from the blackboard
        if (!config.blackboard)
        {
            throw BT::RuntimeError("ResetTrajectories: no blackboard provided.");
        }
        if (!config.blackboard->get("node", node_))
        {
            throw BT::RuntimeError("ResetTrajectories: 'node' not found in blackboard.");
        }

        RCLCPP_INFO(node_->get_logger(),
                    "ResetTrajectories [%s]: Constructed with node [%s].",
                    name.c_str(), node_->get_fully_qualified_name());
    }

    BT::NodeStatus ResetTrajectories::tick()
    {
        // Get move_ids from input port
        std::string move_ids_str;
        if (!getInput<std::string>("move_ids", move_ids_str))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "ResetTrajectories [%s]: missing InputPort [move_ids].",
                         name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Split move_ids_str by comma
        std::vector<std::string> move_ids;
        std::stringstream ss(move_ids_str);
        std::string id;
        while (std::getline(ss, id, ','))
        {
            // Trim whitespace
            id.erase(0, id.find_first_not_of(" \t"));
            id.erase(id.find_last_not_of(" \t") + 1);
            if (!id.empty())
            {
                move_ids.push_back(id);
            }
        }

        if (move_ids.empty())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "ResetTrajectories [%s]: No move_ids provided to reset.",
                        name().c_str());
            return BT::NodeStatus::SUCCESS;
        }

        // Perform reset for each move_id
        for (const auto &mid_str : move_ids)
        {
            try
            {
                int mid = std::stoi(mid_str);

                // Reset trajectory_{id} to empty
                moveit_msgs::msg::RobotTrajectory empty_traj;
                std::string traj_key = "trajectory_" + mid_str;
                config().blackboard->set(traj_key, empty_traj);

                // Reset validity_{id} to false
                std::string validity_key = "validity_" + mid_str;
                config().blackboard->set(validity_key, false);

                RCLCPP_INFO(node_->get_logger(),
                            "ResetTrajectories [%s]: Reset move_id=%d => %s cleared, %s set to false.",
                            name().c_str(), mid, traj_key.c_str(), validity_key.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "ResetTrajectories [%s]: Invalid move_id '%s'. Exception: %s",
                             name().c_str(), mid_str.c_str(), e.what());
                // Continue resetting other IDs
            }
        }

        return BT::NodeStatus::SUCCESS;
    }

} // namespace manymove_cpp_trees
