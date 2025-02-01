#include "manymove_cpp_trees/action_nodes_logic.hpp"
#include <behaviortree_cpp_v3/blackboard.h>

namespace manymove_cpp_trees
{

    RetryPauseAbortNode::RetryPauseAbortNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::DecoratorNode(name, config)
    {
    }

    BT::NodeStatus RetryPauseAbortNode::tick()
    {
        // Read the two controlling blackboard keys:
        bool abort_mission = false;
        bool stop_execution = false;

        if (!getInput("abort_mission", abort_mission))
            abort_mission = false;
        if (!getInput("stop_execution", stop_execution))
            stop_execution = false;

        // Priority 1: abort_mission is true: halt child and return FAILURE.
        if (abort_mission)
        {
            if (child_node_ && child_node_->status() == BT::NodeStatus::RUNNING)
                child_node_->halt();
            return BT::NodeStatus::FAILURE;
        }

        // Priority 2: stop_execution is true: halt child and return RUNNING (i.e. pause the retry loop).
        if (stop_execution)
        {
            if (child_node_ && child_node_->status() == BT::NodeStatus::RUNNING)
                child_node_->halt();
            return BT::NodeStatus::RUNNING;
        }

        // Otherwise, tick the child normally.
        if (!child_node_)
            throw BT::RuntimeError("RetryPauseAbortNode: missing child");

        BT::NodeStatus child_status = child_node_->executeTick();

        // If child returns FAILURE, then (like an infinite retry) we return RUNNING so that on the next tick it will be retried.
        if (child_status == BT::NodeStatus::FAILURE)
            return BT::NodeStatus::RUNNING;
        else if (child_status == BT::NodeStatus::SUCCESS)
            return BT::NodeStatus::SUCCESS;
        else // if (child_status == RUNNING)
            return BT::NodeStatus::RUNNING;
    }

    void RetryPauseAbortNode::halt()
    {
        if (child_node_ && child_node_->status() == BT::NodeStatus::RUNNING)
            child_node_->halt();
        BT::DecoratorNode::halt();
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
