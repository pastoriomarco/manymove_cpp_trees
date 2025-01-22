#include "manymove_cpp_trees/action_nodes_objects.hpp"
#include <behaviortree_cpp_v3/blackboard.h>

#include <memory>
#include <stdexcept>

namespace manymove_cpp_trees
{
    // ---------------------- AddCollisionObjectAction ----------------------

    AddCollisionObjectAction::AddCollisionObjectAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          goal_sent_(false),
          result_received_(false)
    {
        // Obtain the ROS node from the blackboard
        if (!config.blackboard)
        {
            throw BT::RuntimeError("AddCollisionObjectAction: no blackboard provided.");
        }
        if (!config.blackboard->get("node", node_))
        {
            throw BT::RuntimeError("AddCollisionObjectAction: 'node' not found in blackboard.");
        }

        // Initialize the action client
        action_client_ = rclcpp_action::create_client<AddCollisionObject>(node_, "add_collision_object");
        RCLCPP_INFO(node_->get_logger(), "AddCollisionObjectAction: Waiting for 'add_collision_object' server...");
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            throw BT::RuntimeError("AddCollisionObjectAction: 'add_collision_object' server not available after waiting.");
        }
        RCLCPP_INFO(node_->get_logger(), "AddCollisionObjectAction: Connected to 'add_collision_object' server.");
    }

    BT::NodeStatus AddCollisionObjectAction::onStart()
    {
        RCLCPP_INFO(node_->get_logger(), "AddCollisionObjectAction: onStart() called.");

        goal_sent_ = false;
        result_received_ = false;
        action_result_ = AddCollisionObject::Result();

        // Retrieve input ports
        if (!getInput<std::string>("object_id", object_id_))
        {
            RCLCPP_ERROR(node_->get_logger(), "AddCollisionObjectAction: Missing required input 'object_id'.");
            return BT::NodeStatus::FAILURE;
        }

        std::string shape;
        if (!getInput<std::string>("shape", shape))
        {
            RCLCPP_ERROR(node_->get_logger(), "AddCollisionObjectAction: Missing required input 'shape'.");
            return BT::NodeStatus::FAILURE;
        }

        std::vector<double> dimensions;
        if (shape != "mesh")
        {
            if (!getInput<std::vector<double>>("dimensions", dimensions))
            {
                RCLCPP_ERROR(node_->get_logger(), "AddCollisionObjectAction: Missing required input 'dimensions' for shape '%s'.", shape.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        geometry_msgs::msg::Pose pose;
        if (!getInput<geometry_msgs::msg::Pose>("pose", pose))
        {
            RCLCPP_ERROR(node_->get_logger(), "AddCollisionObjectAction: Missing required input 'pose'.");
            return BT::NodeStatus::FAILURE;
        }

        std::string mesh_file;
        if (shape == "mesh")
        {
            if (!getInput<std::string>("mesh_file", mesh_file))
            {
                RCLCPP_ERROR(node_->get_logger(), "AddCollisionObjectAction: Missing required input 'mesh_file' for mesh shape.");
                return BT::NodeStatus::FAILURE;
            }
        }

        double scale_x, scale_y, scale_z;
        getInput<double>("scale_mesh_x", scale_x);
        getInput<double>("scale_mesh_y", scale_y);
        getInput<double>("scale_mesh_z", scale_z);

        // Create and send the goal
        auto goal_msg = AddCollisionObject::Goal();
        goal_msg.id = object_id_;
        goal_msg.shape = shape;
        goal_msg.pose = pose;

        if (shape == "mesh")
        {
            goal_msg.mesh_file = mesh_file;
            goal_msg.scale_mesh_x = scale_x;
            goal_msg.scale_mesh_y = scale_y;
            goal_msg.scale_mesh_z = scale_z;
        }
        else
        {
            goal_msg.dimensions = dimensions;
        }

        RCLCPP_INFO(node_->get_logger(), "AddCollisionObjectAction: Sending goal for object '%s'.", object_id_.c_str());

        auto send_goal_options = rclcpp_action::Client<AddCollisionObject>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&AddCollisionObjectAction::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&AddCollisionObjectAction::resultCallback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus AddCollisionObjectAction::onRunning()
    {
        if (result_received_)
        {
            if (action_result_.success)
            {
                RCLCPP_INFO(node_->get_logger(), "AddCollisionObjectAction: Successfully added object '%s'.", object_id_.c_str());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "AddCollisionObjectAction: Failed to add object '%s'. Message: %s", object_id_.c_str(), action_result_.message.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        return BT::NodeStatus::RUNNING;
    }

    void AddCollisionObjectAction::onHalted()
    {
        RCLCPP_WARN(node_->get_logger(), "AddCollisionObjectAction: onHalted() called. Cancelling goal if sent.");

        if (goal_sent_ && !result_received_)
        {
            action_client_->async_cancel_all_goals();
            RCLCPP_INFO(node_->get_logger(), "AddCollisionObjectAction: Goal canceled.");
        }

        goal_sent_ = false;
        result_received_ = false;
    }

    void AddCollisionObjectAction::goalResponseCallback(std::shared_ptr<GoalHandleAddCollisionObject> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "AddCollisionObjectAction: Goal was rejected by the server.");
            // You can set action_result_ here if needed
            result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "AddCollisionObjectAction: Goal accepted by the server, waiting for result.");
        }
    }

    void AddCollisionObjectAction::resultCallback(const GoalHandleAddCollisionObject::WrappedResult &wrapped_result)
    {
        switch (wrapped_result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "AddCollisionObjectAction: Goal succeeded.");
            action_result_ = *(wrapped_result.result);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "AddCollisionObjectAction: Goal was aborted.");
            action_result_.success = false;
            action_result_.message = "Action aborted.";
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(node_->get_logger(), "AddCollisionObjectAction: Goal was canceled.");
            action_result_.success = false;
            action_result_.message = "Action canceled.";
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "AddCollisionObjectAction: Unknown result code.");
            action_result_.success = false;
            action_result_.message = "Unknown result code.";
            break;
        }

        result_received_ = true;
    }

    // ---------------------- RemoveCollisionObjectAction ----------------------

    RemoveCollisionObjectAction::RemoveCollisionObjectAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          goal_sent_(false),
          result_received_(false)
    {
        // Obtain the ROS node from the blackboard
        if (!config.blackboard)
        {
            throw BT::RuntimeError("RemoveCollisionObjectAction: no blackboard provided.");
        }
        if (!config.blackboard->get("node", node_))
        {
            throw BT::RuntimeError("RemoveCollisionObjectAction: 'node' not found in blackboard.");
        }

        // Initialize the action client
        action_client_ = rclcpp_action::create_client<RemoveCollisionObject>(node_, "remove_collision_object");
        RCLCPP_INFO(node_->get_logger(), "RemoveCollisionObjectAction: Waiting for 'remove_collision_object' server...");
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            throw BT::RuntimeError("RemoveCollisionObjectAction: 'remove_collision_object' server not available after waiting.");
        }
        RCLCPP_INFO(node_->get_logger(), "RemoveCollisionObjectAction: Connected to 'remove_collision_object' server.");
    }

    BT::NodeStatus RemoveCollisionObjectAction::onStart()
    {
        RCLCPP_INFO(node_->get_logger(), "RemoveCollisionObjectAction: onStart() called.");

        goal_sent_ = false;
        result_received_ = false;
        action_result_ = RemoveCollisionObject::Result();

        // Retrieve input port
        if (!getInput<std::string>("object_id", object_id_))
        {
            RCLCPP_ERROR(node_->get_logger(), "RemoveCollisionObjectAction: Missing required input 'object_id'.");
            return BT::NodeStatus::FAILURE;
        }

        // Create and send the goal
        auto goal_msg = RemoveCollisionObject::Goal();
        goal_msg.id = object_id_;

        RCLCPP_INFO(node_->get_logger(), "RemoveCollisionObjectAction: Sending goal to remove object '%s'.", object_id_.c_str());

        auto send_goal_options = rclcpp_action::Client<RemoveCollisionObject>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&RemoveCollisionObjectAction::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&RemoveCollisionObjectAction::resultCallback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus RemoveCollisionObjectAction::onRunning()
    {
        if (result_received_)
        {
            if (action_result_.success)
            {
                RCLCPP_INFO(node_->get_logger(), "RemoveCollisionObjectAction: Successfully removed object '%s'.", object_id_.c_str());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "RemoveCollisionObjectAction: Failed to remove object '%s'. Message: %s", object_id_.c_str(), action_result_.message.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        return BT::NodeStatus::RUNNING;
    }

    void RemoveCollisionObjectAction::onHalted()
    {
        RCLCPP_WARN(node_->get_logger(), "RemoveCollisionObjectAction: onHalted() called. Cancelling goal if sent.");

        if (goal_sent_ && !result_received_)
        {
            action_client_->async_cancel_all_goals();
            RCLCPP_INFO(node_->get_logger(), "RemoveCollisionObjectAction: Goal canceled.");
        }

        goal_sent_ = false;
        result_received_ = false;
    }

    void RemoveCollisionObjectAction::goalResponseCallback(std::shared_ptr<GoalHandleRemoveCollisionObject> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "RemoveCollisionObjectAction: Goal was rejected by the server.");
            // You can set action_result_ here if needed
            result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "RemoveCollisionObjectAction: Goal accepted by the server, waiting for result.");
        }
    }

    void RemoveCollisionObjectAction::resultCallback(const GoalHandleRemoveCollisionObject::WrappedResult &wrapped_result)
    {
        switch (wrapped_result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "RemoveCollisionObjectAction: Goal succeeded.");
            action_result_ = *(wrapped_result.result);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "RemoveCollisionObjectAction: Goal was aborted.");
            action_result_.success = false;
            action_result_.message = "Action aborted.";
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(node_->get_logger(), "RemoveCollisionObjectAction: Goal was canceled.");
            action_result_.success = false;
            action_result_.message = "Action canceled.";
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "RemoveCollisionObjectAction: Unknown result code.");
            action_result_.success = false;
            action_result_.message = "Unknown result code.";
            break;
        }

        result_received_ = true;
    }

    // ---------------------- AttachDetachObjectAction ----------------------

    AttachDetachObjectAction::AttachDetachObjectAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          goal_sent_(false),
          result_received_(false),
          attach_(true) // Default to attach
    {
        // Obtain the ROS node from the blackboard
        if (!config.blackboard)
        {
            throw BT::RuntimeError("AttachDetachObjectAction: no blackboard provided.");
        }
        if (!config.blackboard->get("node", node_))
        {
            throw BT::RuntimeError("AttachDetachObjectAction: 'node' not found in blackboard.");
        }

        // Initialize the action client
        action_client_ = rclcpp_action::create_client<AttachDetachObject>(node_, "attach_detach_object");
        RCLCPP_INFO(node_->get_logger(), "AttachDetachObjectAction: Waiting for 'attach_detach_object' server...");
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            throw BT::RuntimeError("AttachDetachObjectAction: 'attach_detach_object' server not available after waiting.");
        }
        RCLCPP_INFO(node_->get_logger(), "AttachDetachObjectAction: Connected to 'attach_detach_object' server.");
    }

    BT::NodeStatus AttachDetachObjectAction::onStart()
    {
        RCLCPP_INFO(node_->get_logger(), "AttachDetachObjectAction: onStart() called.");

        goal_sent_ = false;
        result_received_ = false;
        action_result_ = AttachDetachObject::Result();

        // Retrieve input ports
        if (!getInput<std::string>("object_id", object_id_))
        {
            RCLCPP_ERROR(node_->get_logger(), "AttachDetachObjectAction: Missing required input 'object_id'.");
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput<std::string>("link_name", link_name_))
        {
            RCLCPP_ERROR(node_->get_logger(), "AttachDetachObjectAction: Missing required input 'link_name'.");
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput<bool>("attach", attach_))
        {
            RCLCPP_WARN(node_->get_logger(), "AttachDetachObjectAction: Missing input 'attach'. Defaulting to true.");
            attach_ = true;
        }

        // Create and send the goal
        auto goal_msg = AttachDetachObject::Goal();
        goal_msg.object_id = object_id_;
        goal_msg.link_name = link_name_;
        goal_msg.attach = attach_;

        std::string action = attach_ ? "attaching" : "detaching";
        RCLCPP_INFO(node_->get_logger(), "AttachDetachObjectAction: Sending goal for %s object '%s' to link '%s'.",
                    action.c_str(), object_id_.c_str(), link_name_.c_str());

        auto send_goal_options = rclcpp_action::Client<AttachDetachObject>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&AttachDetachObjectAction::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&AttachDetachObjectAction::resultCallback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus AttachDetachObjectAction::onRunning()
    {
        if (result_received_)
        {
            if (action_result_.success)
            {
                std::string action = attach_ ? "attached" : "detached";
                RCLCPP_INFO(node_->get_logger(), "AttachDetachObjectAction: Successfully %s object '%s' to link '%s'.",
                            action.c_str(), object_id_.c_str(), link_name_.c_str());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                std::string action = attach_ ? "attach" : "detach";
                RCLCPP_ERROR(node_->get_logger(), "AttachDetachObjectAction: Failed to %s object '%s' to link '%s'. Message: %s",
                             action.c_str(), object_id_.c_str(), link_name_.c_str(), action_result_.message.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        return BT::NodeStatus::RUNNING;
    }

    void AttachDetachObjectAction::onHalted()
    {
        RCLCPP_WARN(node_->get_logger(), "AttachDetachObjectAction: onHalted() called. Cancelling goal if sent.");

        if (goal_sent_ && !result_received_)
        {
            action_client_->async_cancel_all_goals();
            RCLCPP_INFO(node_->get_logger(), "AttachDetachObjectAction: Goal canceled.");
        }

        goal_sent_ = false;
        result_received_ = false;
    }

    void AttachDetachObjectAction::goalResponseCallback(std::shared_ptr<GoalHandleAttachDetachObject> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "AttachDetachObjectAction: Goal was rejected by the server.");
            // You can set action_result_ here if needed
            result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "AttachDetachObjectAction: Goal accepted by the server, waiting for result.");
        }
    }

    void AttachDetachObjectAction::resultCallback(const GoalHandleAttachDetachObject::WrappedResult &wrapped_result)
    {
        switch (wrapped_result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "AttachDetachObjectAction: Goal succeeded.");
            action_result_ = *(wrapped_result.result);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "AttachDetachObjectAction: Goal was aborted.");
            action_result_.success = false;
            action_result_.message = "Action aborted.";
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(node_->get_logger(), "AttachDetachObjectAction: Goal was canceled.");
            action_result_.success = false;
            action_result_.message = "Action canceled.";
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "AttachDetachObjectAction: Unknown result code.");
            action_result_.success = false;
            action_result_.message = "Unknown result code.";
            break;
        }

        result_received_ = true;
    }

    // ---------------------- CheckObjectExistsAction ----------------------

    CheckObjectExistsAction::CheckObjectExistsAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          goal_sent_(false),
          result_received_(false)
    {
        // Obtain the ROS node from the blackboard
        if (!config.blackboard)
        {
            throw BT::RuntimeError("CheckObjectExistsAction: no blackboard provided.");
        }
        if (!config.blackboard->get("node", node_))
        {
            throw BT::RuntimeError("CheckObjectExistsAction: 'node' not found in blackboard.");
        }

        // Initialize the action client
        action_client_ = rclcpp_action::create_client<CheckObjectExists>(node_, "check_object_exists");
        RCLCPP_INFO(node_->get_logger(), "CheckObjectExistsAction: Waiting for 'check_object_exists' server...");
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            throw BT::RuntimeError("CheckObjectExistsAction: 'check_object_exists' server not available after waiting.");
        }
        RCLCPP_INFO(node_->get_logger(), "CheckObjectExistsAction: Connected to 'check_object_exists' server.");
    }

    BT::NodeStatus CheckObjectExistsAction::onStart()
    {
        RCLCPP_INFO(node_->get_logger(), "CheckObjectExistsAction: onStart() called.");

        goal_sent_ = false;
        result_received_ = false;
        action_result_ = CheckObjectExists::Result();

        // Retrieve input port
        if (!getInput<std::string>("object_id", object_id_))
        {
            RCLCPP_ERROR(node_->get_logger(), "CheckObjectExistsAction: Missing required input 'object_id'.");
            return BT::NodeStatus::FAILURE;
        }

        // Create and send the goal
        auto goal_msg = CheckObjectExists::Goal();
        goal_msg.object_id = object_id_;

        RCLCPP_INFO(node_->get_logger(), "CheckObjectExistsAction: Sending goal to check existence of object '%s'.", object_id_.c_str());

        auto send_goal_options = rclcpp_action::Client<CheckObjectExists>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&CheckObjectExistsAction::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&CheckObjectExistsAction::resultCallback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus CheckObjectExistsAction::onRunning()
    {
        if (result_received_)
        {
            // Set outputs
            setOutput("exists", action_result_.exists);
            setOutput("is_attached", action_result_.is_attached);
            setOutput("link_name", action_result_.link_name);

            if (action_result_.exists)
            {
                RCLCPP_INFO(node_->get_logger(), "CheckObjectExistsAction: Object '%s' exists.", object_id_.c_str());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "CheckObjectExistsAction: Object '%s' does not exist.", object_id_.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        return BT::NodeStatus::RUNNING;
    }

    void CheckObjectExistsAction::onHalted()
    {
        RCLCPP_WARN(node_->get_logger(), "CheckObjectExistsAction: onHalted() called. Cancelling goal if sent.");

        if (goal_sent_ && !result_received_)
        {
            action_client_->async_cancel_all_goals();
            RCLCPP_INFO(node_->get_logger(), "CheckObjectExistsAction: Goal canceled.");
        }

        goal_sent_ = false;
        result_received_ = false;
    }

    void CheckObjectExistsAction::goalResponseCallback(std::shared_ptr<GoalHandleCheckObjectExists> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "CheckObjectExistsAction: Goal was rejected by the server.");
            result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "CheckObjectExistsAction: Goal accepted by the server, waiting for result.");
        }
    }

    void CheckObjectExistsAction::resultCallback(const GoalHandleCheckObjectExists::WrappedResult &wrapped_result)
    {
        switch (wrapped_result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "CheckObjectExistsAction: Goal succeeded.");
            action_result_ = *(wrapped_result.result);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "CheckObjectExistsAction: Goal was aborted.");
            action_result_.exists = false;
            action_result_.is_attached = false;
            action_result_.link_name = "";
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(node_->get_logger(), "CheckObjectExistsAction: Goal was canceled.");
            action_result_.exists = false;
            action_result_.is_attached = false;
            action_result_.link_name = "";
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "CheckObjectExistsAction: Unknown result code.");
            action_result_.exists = false;
            action_result_.is_attached = false;
            action_result_.link_name = "";
            break;
        }

        result_received_ = true;
    }

    // ---------------------- GetObjectPoseAction ----------------------

    GetObjectPoseAction::GetObjectPoseAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          goal_sent_(false),
          result_received_(false)
    {
        // Obtain the ROS node from the blackboard
        if (!config.blackboard)
        {
            throw BT::RuntimeError("GetObjectPoseAction: no blackboard provided.");
        }
        if (!config.blackboard->get("node", node_))
        {
            throw BT::RuntimeError("GetObjectPoseAction: 'node' not found in blackboard.");
        }

        // Initialize the action client
        action_client_ = rclcpp_action::create_client<GetObjectPose>(node_, "get_object_pose");
        RCLCPP_INFO(node_->get_logger(), "GetObjectPoseAction: Waiting for 'get_object_pose' server...");
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            throw BT::RuntimeError("GetObjectPoseAction: 'get_object_pose' server not available after waiting.");
        }
        RCLCPP_INFO(node_->get_logger(), "GetObjectPoseAction: Connected to 'get_object_pose' server.");
    }

    BT::NodeStatus GetObjectPoseAction::onStart()
    {
        RCLCPP_INFO(node_->get_logger(), "GetObjectPoseAction: onStart() called.");

        goal_sent_ = false;
        result_received_ = false;
        action_result_ = GetObjectPose::Result();

        // Retrieve input ports
        if (!getInput<std::string>("object_id", object_id_))
        {
            RCLCPP_ERROR(node_->get_logger(), "GetObjectPoseAction: Missing required input 'object_id'.");
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput<std::vector<double>>("pre_transform_xyz_rpy", pre_transform_xyz_rpy_))
        {
            RCLCPP_ERROR(node_->get_logger(), "GetObjectPoseAction: Missing required input 'pre_transform_xyz_rpy'.");
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput<std::vector<double>>("post_transform_xyz_rpy", post_transform_xyz_rpy_))
        {
            RCLCPP_ERROR(node_->get_logger(), "GetObjectPoseAction: Missing required input 'post_transform_xyz_rpy'.");
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput<std::string>("pose_key", pose_key_))
        {
            RCLCPP_ERROR(node_->get_logger(), "GetObjectPoseAction: Missing required input 'pose_key'.");
            return BT::NodeStatus::FAILURE;
        }

        // Validate input sizes
        if (pre_transform_xyz_rpy_.size() != 6)
        {
            RCLCPP_ERROR(node_->get_logger(), "GetObjectPoseAction: 'pre_transform_xyz_rpy' must have exactly 6 elements.");
            return BT::NodeStatus::FAILURE;
        }

        if (post_transform_xyz_rpy_.size() != 6)
        {
            RCLCPP_ERROR(node_->get_logger(), "GetObjectPoseAction: 'post_transform_xyz_rpy' must have exactly 6 elements.");
            return BT::NodeStatus::FAILURE;
        }

        // Create and send the goal
        GetObjectPose::Goal goal_msg;
        goal_msg.object_id = object_id_;
        goal_msg.pre_transform_xyz_rpy = pre_transform_xyz_rpy_;
        goal_msg.post_transform_xyz_rpy = post_transform_xyz_rpy_;

        RCLCPP_INFO(node_->get_logger(), "GetObjectPoseAction: Sending goal for object '%s'.", object_id_.c_str());

        auto send_goal_options = rclcpp_action::Client<GetObjectPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&GetObjectPoseAction::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&GetObjectPoseAction::resultCallback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus GetObjectPoseAction::onRunning()
    {
        if (result_received_)
        {
            if (action_result_.success)
            {
                // Set the output port "pose"
                setOutput("pose", action_result_.pose);

                // **Important**: Also set the pose in the blackboard under pose_key_
                if (!pose_key_.empty())
                {
                    // Attempt to set the pose on the blackboard
                    auto blackboard = config().blackboard;
                    blackboard->set(pose_key_, action_result_.pose);
                    RCLCPP_INFO(node_->get_logger(),
                                "GetObjectPoseAction: Successfully set pose to blackboard key '%s'.",
                                pose_key_.c_str());
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(),
                                 "GetObjectPoseAction: pose_key_ is empty. Cannot set pose on blackboard.");
                    return BT::NodeStatus::FAILURE;
                }

                // Log the new pose
                RCLCPP_DEBUG(node_->get_logger(), "GetObjectPoseAction: New pose for '%s': Position (%.3f, %.3f, %.3f), Orientation (%.3f, %.3f, %.3f, %.3f)",
                             pose_key_.c_str(),
                             action_result_.pose.position.x,
                             action_result_.pose.position.y,
                             action_result_.pose.position.z,
                             action_result_.pose.orientation.x,
                             action_result_.pose.orientation.y,
                             action_result_.pose.orientation.z,
                             action_result_.pose.orientation.w);

                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "GetObjectPoseAction: Failed to retrieve pose. Message: %s", action_result_.message.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        return BT::NodeStatus::RUNNING;
    }

    void GetObjectPoseAction::onHalted()
    {
        RCLCPP_WARN(node_->get_logger(), "GetObjectPoseAction: onHalted() called. Cancelling goal if sent.");

        if (goal_sent_ && !result_received_)
        {
            action_client_->async_cancel_all_goals();
            RCLCPP_INFO(node_->get_logger(), "GetObjectPoseAction: Goal canceled.");
        }

        goal_sent_ = false;
        result_received_ = false;
    }

    void GetObjectPoseAction::goalResponseCallback(std::shared_ptr<GoalHandleGetObjectPose> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "GetObjectPoseAction: Goal was rejected by the server.");
            result_received_ = true;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "GetObjectPoseAction: Goal accepted by the server, waiting for result.");
        }
    }

    void GetObjectPoseAction::resultCallback(const GoalHandleGetObjectPose::WrappedResult &wrapped_result)
    {
        switch (wrapped_result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "GetObjectPoseAction: Goal succeeded.");
            action_result_ = *(wrapped_result.result);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "GetObjectPoseAction: Goal was aborted.");
            action_result_.success = false;
            action_result_.message = "Action aborted.";
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(node_->get_logger(), "GetObjectPoseAction: Goal was canceled.");
            action_result_.success = false;
            action_result_.message = "Action canceled.";
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "GetObjectPoseAction: Unknown result code.");
            action_result_.success = false;
            action_result_.message = "Unknown result code.";
            break;
        }

        result_received_ = true;
    }

} // namespace manymove_cpp_trees
