#include "manymove_cpp_trees/action_nodes_signals.hpp"
#include <behaviortree_cpp_v3/blackboard.h>

#include <memory>
#include <stdexcept>

namespace manymove_cpp_trees
{

    // ------------------------------------------------------------------
    // SetOutputAction
    // ------------------------------------------------------------------

    SetOutputAction::SetOutputAction(const std::string &name,
                                     const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          goal_sent_(false),
          result_received_(false)
    {
        // Obtain the ROS node from the blackboard
        if (!config.blackboard)
        {
            throw BT::RuntimeError("SetOutputAction: no blackboard provided.");
        }
        if (!config.blackboard->get("node", node_))
        {
            throw BT::RuntimeError("SetOutputAction: 'node' not found in blackboard.");
        }

        std::string prefix;
        if (!getInput<std::string>("robot_prefix", prefix))
        {
            prefix = ""; // default if not provided
        }

        std::string server_name = prefix + "set_output";

        // Create the action client
        action_client_ = rclcpp_action::create_client<SetOutput>(node_, server_name);

        RCLCPP_INFO(node_->get_logger(),
                    "SetOutputAction [%s]: waiting up to 5s for '%s' action server...",
                    name.c_str(), server_name.c_str());

        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            throw BT::RuntimeError(
                "SetOutputAction: server '" + server_name + "' not available after 5s.");
        }
    }

    BT::NodeStatus SetOutputAction::onStart()
    {
        RCLCPP_INFO(node_->get_logger(),
                    "SetOutputAction [%s]: onStart() called.",
                    name().c_str());

        goal_sent_ = false;
        result_received_ = false;
        action_result_ = SetOutput::Result();

        // Read input ports
        if (!getInput<std::string>("io_type", io_type_))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "SetOutputAction [%s]: missing 'io_type'.",
                         name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (!getInput<int>("ionum", ionum_))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "SetOutputAction [%s]: missing 'ionum'.",
                         name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (!getInput<int>("value", value_))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "SetOutputAction [%s]: missing 'value'.",
                         name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Build and send goal
        SetOutput::Goal goal_msg;
        goal_msg.io_type = io_type_;
        goal_msg.ionum = ionum_;
        goal_msg.value = value_;

        // Callbacks
        auto send_goal_options = rclcpp_action::Client<SetOutput>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&SetOutputAction::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&SetOutputAction::resultCallback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus SetOutputAction::onRunning()
    {
        if (!result_received_)
        {
            return BT::NodeStatus::RUNNING; // still waiting
        }

        // We have the final result now
        setOutput("success", action_result_.success);

        if (action_result_.success)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "SetOutputAction [%s]: IO type='%s', ionum=%d => SUCCESS",
                        name().c_str(), io_type_.c_str(), ionum_);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "SetOutputAction [%s]: IO type='%s', ionum=%d => FAIL: %s",
                         name().c_str(), io_type_.c_str(), ionum_,
                         action_result_.message.c_str());
            return BT::NodeStatus::FAILURE;
        }
    }

    void SetOutputAction::onHalted()
    {
        RCLCPP_WARN(node_->get_logger(),
                    "SetOutputAction [%s]: onHalted => cancel goal if needed.",
                    name().c_str());

        if (goal_sent_ && !result_received_)
        {
            action_client_->async_cancel_all_goals();
        }
        goal_sent_ = false;
        result_received_ = false;
    }

    void SetOutputAction::goalResponseCallback(std::shared_ptr<GoalHandleSetOutput> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "SetOutputAction [%s]: Goal REJECTED by server.",
                         name().c_str());
            action_result_.success = false;
            result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "SetOutputAction [%s]: Goal ACCEPTED by server.",
                        name().c_str());
        }
    }

    void SetOutputAction::resultCallback(const GoalHandleSetOutput::WrappedResult &wrapped_result)
    {
        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            action_result_ = *(wrapped_result.result);
        }
        else
        {
            action_result_.success = false;
            action_result_.message = "SetOutput failed or aborted";
        }
        result_received_ = true;
    }

    // ------------------------------------------------------------------
    // GetInputAction
    // ------------------------------------------------------------------

    GetInputAction::GetInputAction(const std::string &name,
                                   const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          goal_sent_(false),
          result_received_(false)
    {
        if (!config.blackboard)
        {
            throw BT::RuntimeError("GetInputAction: no blackboard provided.");
        }
        if (!config.blackboard->get("node", node_))
        {
            throw BT::RuntimeError("GetInputAction: 'node' not found in blackboard.");
        }

        std::string prefix;
        if (!getInput<std::string>("robot_prefix", prefix))
        {
            prefix = "";
        }
        std::string server_name = prefix + "get_input";

        // Create action client
        action_client_ = rclcpp_action::create_client<GetInput>(node_, server_name);
        RCLCPP_INFO(node_->get_logger(),
                    "GetInputAction [%s]: waiting up to 5s for '%s' server...",
                    name.c_str(), server_name.c_str());

        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            throw BT::RuntimeError("'" + server_name + "' action server not available.");
        }
    }

    BT::NodeStatus GetInputAction::onStart()
    {
        RCLCPP_INFO(node_->get_logger(),
                    "GetInputAction [%s]: onStart()",
                    name().c_str());

        goal_sent_ = false;
        result_received_ = false;
        action_result_ = GetInput::Result();

        // Read input ports
        if (!getInput<std::string>("io_type", io_type_))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "GetInputAction [%s]: missing 'io_type'",
                         name().c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (!getInput<int>("ionum", ionum_))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "GetInputAction [%s]: missing 'ionum'",
                         name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Build the goal
        GetInput::Goal goal_msg;
        goal_msg.io_type = io_type_;
        goal_msg.ionum = ionum_;

        // Send goal
        auto send_goal_options = rclcpp_action::Client<GetInput>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&GetInputAction::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&GetInputAction::resultCallback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus GetInputAction::onRunning()
    {
        if (!result_received_)
        {
            return BT::NodeStatus::RUNNING; // still waiting
        }

        // We have the final result
        setOutput("success", action_result_.success);
        setOutput("value", static_cast<int>(action_result_.value));

        if (action_result_.success)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "GetInputAction [%s]: read IO type='%s', ionum=%d => value=%d",
                        name().c_str(), io_type_.c_str(), ionum_, action_result_.value);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "GetInputAction [%s]: FAILED => %s",
                         name().c_str(), action_result_.message.c_str());
            return BT::NodeStatus::FAILURE;
        }
    }

    void GetInputAction::onHalted()
    {
        RCLCPP_WARN(node_->get_logger(),
                    "GetInputAction [%s]: onHalted => cancel if needed",
                    name().c_str());

        if (goal_sent_ && !result_received_)
        {
            action_client_->async_cancel_all_goals();
        }
        goal_sent_ = false;
        result_received_ = false;
    }

    void GetInputAction::goalResponseCallback(std::shared_ptr<GoalHandleGetInput> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "GetInputAction [%s]: Goal REJECTED.",
                         name().c_str());
            action_result_.success = false;
            result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "GetInputAction [%s]: Goal ACCEPTED.",
                        name().c_str());
        }
    }

    void GetInputAction::resultCallback(const GoalHandleGetInput::WrappedResult &wrapped_result)
    {
        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            action_result_ = *(wrapped_result.result);
        }
        else
        {
            action_result_.success = false;
            action_result_.message = "GetInput failed or aborted";
        }
        result_received_ = true;
    }

    // ------------------------------------------------------------------
    // CheckRobotStateAction
    // ------------------------------------------------------------------

    CheckRobotStateAction::CheckRobotStateAction(const std::string &name,
                                                 const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          goal_sent_(false),
          result_received_(false)
    {
        // Retrieve the ROS node from the blackboard
        if (!config.blackboard)
        {
            throw BT::RuntimeError("CheckRobotStateAction: no blackboard provided.");
        }
        if (!config.blackboard->get("node", node_))
        {
            throw BT::RuntimeError("CheckRobotStateAction: 'node' not found in blackboard.");
        }

        std::string prefix;
        if (!getInput<std::string>("robot_prefix", prefix))
        {
            prefix = "";
        }
        std::string server_name = prefix + "check_robot_state";

        action_client_ = rclcpp_action::create_client<CheckRobotState>(node_, server_name);

        RCLCPP_INFO(node_->get_logger(),
                    "CheckRobotStateAction [%s]: waiting 5s for server '%s'...",
                    name.c_str(), server_name.c_str());

        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            throw BT::RuntimeError("CheckRobotStateAction: server '" + server_name + "' not available after 5s.");
        }
    }

    BT::NodeStatus CheckRobotStateAction::onStart()
    {
        RCLCPP_INFO(node_->get_logger(),
                    "CheckRobotStateAction [%s]: onStart()",
                    name().c_str());

        // Reset flags and result
        goal_sent_ = false;
        result_received_ = false;
        action_result_ = CheckRobotState::Result();

        // Typically, an empty goal if the server just checks the robot
        CheckRobotState::Goal goal_msg;

        // SendGoal Options
        auto send_goal_options = rclcpp_action::Client<CheckRobotState>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&CheckRobotStateAction::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&CheckRobotStateAction::resultCallback, this, std::placeholders::_1);

        // Send goal asynchronously
        action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus CheckRobotStateAction::onRunning()
    {
        if (!result_received_)
        {
            // Still waiting for the result
            return BT::NodeStatus::RUNNING;
        }

        // We have the final result. Set output ports
        setOutput("ready", action_result_.ready);
        setOutput("err", (int)action_result_.err);
        setOutput("mode", (int)action_result_.mode);
        setOutput("state", (int)action_result_.state);
        setOutput("message", action_result_.message);

        if (action_result_.ready)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "CheckRobotStateAction [%s]: Robot is READY. (mode=%d, state=%d)",
                        name().c_str(), action_result_.mode, action_result_.state);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "CheckRobotStateAction [%s]: Robot is NOT ready => err=%d, mode=%d, state=%d. Msg=%s",
                        name().c_str(),
                        action_result_.err,
                        action_result_.mode,
                        action_result_.state,
                        action_result_.message.c_str());
            return BT::NodeStatus::FAILURE;
        }
    }

    void CheckRobotStateAction::onHalted()
    {
        RCLCPP_WARN(node_->get_logger(),
                    "CheckRobotStateAction [%s]: onHalted => cancel if needed.",
                    name().c_str());

        if (goal_sent_ && !result_received_)
        {
            action_client_->async_cancel_all_goals();
        }
        goal_sent_ = false;
        result_received_ = false;
    }

    void CheckRobotStateAction::goalResponseCallback(std::shared_ptr<GoalHandleCheckRobotState> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "CheckRobotStateAction [%s]: Goal REJECTED by server.",
                         name().c_str());
            action_result_.ready = false;
            action_result_.message = "check_robot_state goal was rejected";
            result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "CheckRobotStateAction [%s]: Goal ACCEPTED by server.",
                        name().c_str());
        }
    }

    void CheckRobotStateAction::resultCallback(const GoalHandleCheckRobotState::WrappedResult &wrapped_result)
    {
        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            // The server returned a valid result
            action_result_ = *(wrapped_result.result);
        }
        else
        {
            // If aborted, canceled, or an error code => not ready
            RCLCPP_ERROR(node_->get_logger(),
                         "CheckRobotStateAction [%s]: Action finished with code %d (not successful).",
                         name().c_str(),
                         static_cast<int>(wrapped_result.code));
            action_result_.ready = false;
            action_result_.message = "check_robot_state aborted/canceled or failed";
        }
        result_received_ = true;
    }

    // ------------------------------------------------------------------
    // ResetRobotStateAction
    // ------------------------------------------------------------------

    ResetRobotStateAction::ResetRobotStateAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          goal_sent_(false),
          result_received_(false),
          unload_traj_success_(false),
          load_traj_success_(false)
    {
        if (!config.blackboard)
        {
            throw BT::RuntimeError("ResetRobotStateAction: no blackboard provided.");
        }
        if (!config.blackboard->get("node", node_))
        {
            throw BT::RuntimeError("ResetRobotStateAction: 'node' not found in blackboard.");
        }

        std::string prefix;
        if (!getInput<std::string>("robot_prefix", prefix))
        {
            prefix = "";
        }

        std::string model;
        if (!getInput<std::string>("robot_model", model))
        {
            model = ""; 
        }

        computed_controller_name_ = prefix + model + "_traj_controller";

        RCLCPP_INFO(node_->get_logger(),
                    "ResetRobotStateAction [%s]: Using controller '%s'.",
                    name.c_str(), computed_controller_name_.c_str());

        // Initialize ResetRobotState action client
        std::string reset_robot_state_server = prefix + "reset_robot_state";
        action_client_ = rclcpp_action::create_client<ResetRobotState>(node_, reset_robot_state_server);

        // Initialize UnloadTrajController action client
        std::string unload_traj_server = prefix + "unload_trajectory_controller";
        unload_traj_client_ = rclcpp_action::create_client<UnloadTrajController>(node_, unload_traj_server);

        // Initialize LoadTrajController action client
        std::string load_traj_server = prefix + "load_trajectory_controller";
        load_traj_client_ = rclcpp_action::create_client<LoadTrajController>(node_, load_traj_server);

        RCLCPP_INFO(node_->get_logger(),
                    "ResetRobotStateAction [%s]: waiting up to 5s for '%s', '%s', and '%s' servers...",
                    name.c_str(), reset_robot_state_server.c_str(), unload_traj_server.c_str(), load_traj_server.c_str());

        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)) ||
            !unload_traj_client_->wait_for_action_server(std::chrono::seconds(5)) ||
            !load_traj_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            throw BT::RuntimeError("ResetRobotStateAction: one or more action servers not available.");
        }
    }

    BT::NodeStatus ResetRobotStateAction::onStart()
    {
        RCLCPP_INFO(node_->get_logger(),
                    "ResetRobotStateAction [%s]: onStart()",
                    name().c_str());

        // Reset all flags
        goal_sent_ = false;
        result_received_ = false;
        unload_traj_success_ = false;
        load_traj_success_ = false;
        unload_goal_sent_ = false;
        reset_goal_sent_ = false;
        load_goal_sent_ = false;

        // Step 1: Call UnloadTrajController action
        UnloadTrajController::Goal unload_traj_goal;
        unload_traj_goal.controller_name = computed_controller_name_;

        auto unload_traj_options = rclcpp_action::Client<UnloadTrajController>::SendGoalOptions();
        unload_traj_options.goal_response_callback =
            std::bind(&ResetRobotStateAction::goalResponseCallbackUnloadTraj, this, std::placeholders::_1);
        unload_traj_options.result_callback =
            std::bind(&ResetRobotStateAction::resultCallbackUnloadTraj, this, std::placeholders::_1);

        unload_traj_client_->async_send_goal(unload_traj_goal, unload_traj_options);
        unload_goal_sent_ = true; // Mark that the Unload goal has been sent

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus ResetRobotStateAction::onRunning()
    {
        // Step 1: Wait for UnloadTrajController to complete
        if (!unload_traj_success_)
        {
            // Still waiting for UnloadTrajController to complete
            return BT::NodeStatus::RUNNING;
        }

        // Step 2: Send ResetRobotState goal if not already sent
        if (!reset_goal_sent_)
        {
            ResetRobotState::Goal goal_msg;
            auto send_goal_options = rclcpp_action::Client<ResetRobotState>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                std::bind(&ResetRobotStateAction::goalResponseCallback, this, std::placeholders::_1);
            send_goal_options.result_callback =
                std::bind(&ResetRobotStateAction::resultCallback, this, std::placeholders::_1);

            action_client_->async_send_goal(goal_msg, send_goal_options);
            reset_goal_sent_ = true; // Mark that the Reset goal has been sent
            return BT::NodeStatus::RUNNING;
        }

        // Step 3: Wait for ResetRobotState to complete
        if (!result_received_)
        {
            return BT::NodeStatus::RUNNING;
        }

        // Step 4: Send LoadTrajController goal if not already sent
        if (!load_goal_sent_)
        {
            LoadTrajController::Goal load_traj_goal;
            load_traj_goal.controller_name = computed_controller_name_;

            auto load_traj_options = rclcpp_action::Client<LoadTrajController>::SendGoalOptions();
            load_traj_options.goal_response_callback =
                std::bind(&ResetRobotStateAction::goalResponseCallbackLoadTraj, this, std::placeholders::_1);
            load_traj_options.result_callback =
                std::bind(&ResetRobotStateAction::resultCallbackLoadTraj, this, std::placeholders::_1);

            load_traj_client_->async_send_goal(load_traj_goal, load_traj_options);
            load_goal_sent_ = true; // Mark that the Load goal has been sent
            return BT::NodeStatus::RUNNING;
        }

        // Step 5: Wait for LoadTrajController to complete
        if (!load_traj_success_)
        {
            return BT::NodeStatus::RUNNING;
        }

        // All actions completed successfully
        setOutput("success", true);
        RCLCPP_INFO(node_->get_logger(),
                    "ResetRobotStateAction [%s]: SUCCESS => Full sequence completed",
                    name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    void ResetRobotStateAction::onHalted()
    {
        RCLCPP_WARN(node_->get_logger(),
                    "ResetRobotStateAction [%s]: onHalted => cancel if needed.",
                    name().c_str());

        if (goal_sent_ && !result_received_)
        {
            action_client_->async_cancel_all_goals();
        }
        if (!unload_traj_success_)
        {
            unload_traj_client_->async_cancel_all_goals();
        }
        if (!load_traj_success_)
        {
            load_traj_client_->async_cancel_all_goals();
        }
        goal_sent_ = false;
        result_received_ = false;
        unload_goal_sent_ = false;
        reset_goal_sent_ = false;
        load_goal_sent_ = false;
    }

    void ResetRobotStateAction::goalResponseCallback(std::shared_ptr<GoalHandleResetRobotState> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "ResetRobotStateAction [%s]: ResetRobotState Goal REJECTED by server.",
                         name().c_str());
            action_result_.success = false;
            result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "ResetRobotStateAction [%s]: ResetRobotState Goal ACCEPTED by server.",
                        name().c_str());
        }
    }

    void ResetRobotStateAction::resultCallback(const GoalHandleResetRobotState::WrappedResult &wrapped_result)
    {
        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            action_result_ = *(wrapped_result.result);
        }
        else
        {
            action_result_.success = false;
            action_result_.message = "ResetRobotState aborted or failed";
        }
        result_received_ = true;
    }

    void ResetRobotStateAction::goalResponseCallbackUnloadTraj(std::shared_ptr<GoalHandleUnloadTrajController> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "ResetRobotStateAction [%s]: UnloadTrajController Goal REJECTED by server.",
                         name().c_str());
            unload_traj_success_ = false;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "ResetRobotStateAction [%s]: UnloadTrajController Goal ACCEPTED by server.",
                        name().c_str());
        }
    }

    void ResetRobotStateAction::resultCallbackUnloadTraj(const GoalHandleUnloadTrajController::WrappedResult &wrapped_result)
    {
        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            unload_traj_success_ = wrapped_result.result->success;
        }
        else
        {
            unload_traj_success_ = false;
        }
    }

    void ResetRobotStateAction::goalResponseCallbackLoadTraj(std::shared_ptr<GoalHandleLoadTrajController> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "ResetRobotStateAction [%s]: LoadTrajController Goal REJECTED by server.",
                         name().c_str());
            load_traj_success_ = false;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "ResetRobotStateAction [%s]: LoadTrajController Goal ACCEPTED by server.",
                        name().c_str());
        }
    }

    void ResetRobotStateAction::resultCallbackLoadTraj(const GoalHandleLoadTrajController::WrappedResult &wrapped_result)
    {
        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            load_traj_success_ = wrapped_result.result->success;
        }
        else
        {
            load_traj_success_ = false;
        }
    }

    // ---------------------------------------------------------------
    // CheckBlackboardValue Implementation
    // ---------------------------------------------------------------
    CheckBlackboardValue::CheckBlackboardValue(const std::string &name,
                                               const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {
        // If you need to confirm the blackboard is present:
        if (!config.blackboard)
        {
            throw BT::RuntimeError("CheckBlackboardValue: no blackboard provided.");
        }
        // If you needed an rclcpp node for logging, you could do:
        // config.blackboard->get("node", node_);
        // but this condition node typically doesn't require an ROS node.
    }

    BT::NodeStatus CheckBlackboardValue::tick()
    {
        // 1) Extract input ports "key" and "value"
        std::string key;
        int expected_value;
        if (!getInput<std::string>("key", key))
        {
            throw BT::RuntimeError("CheckBlackboardValue: Missing required input [key]");
        }
        if (!getInput<int>("value", expected_value))
        {
            throw BT::RuntimeError("CheckBlackboardValue: Missing required input [value]");
        }

        // 2) Read the blackboard
        int actual_value = 0;
        if (!config().blackboard->get(key, actual_value))
        {
            // Key not found => we fail
            return BT::NodeStatus::FAILURE;
        }

        // 3) Compare the blackboard value to the expected value
        if (actual_value == expected_value)
        {
            // Condition satisfied => SUCCESS
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            // Condition not satisfied => FAILURE
            return BT::NodeStatus::FAILURE;
        }
    }

} // namespace manymove_cpp_trees
