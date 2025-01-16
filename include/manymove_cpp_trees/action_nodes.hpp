#ifndef MANYMOVE_CPP_TREES_ACTION_NODES_HPP
#define MANYMOVE_CPP_TREES_ACTION_NODES_HPP

#include "manymove_cpp_trees/move.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp_v3/action_node.h>

#include <manymove_planner/action/plan_manipulator.hpp>
#include <manymove_planner/action/execute_trajectory.hpp>

#include <manymove_object_manager/action/add_collision_object.hpp>
#include <manymove_object_manager/action/remove_collision_object.hpp>
#include <manymove_object_manager/action/attach_detach_object.hpp>
#include <manymove_object_manager/action/check_object_exists.hpp>
#include <manymove_object_manager/action/get_object_pose.hpp>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <string>
#include <vector>

namespace manymove_cpp_trees
{

    /**
     * @class PlanningAction
     * @brief A stateful BT node that sends a planning request to the plan_manipulator action server.
     *
     * We use StatefulActionNode to illustrate best practices:
     *   - onStart() => send goal
     *   - onRunning() => check result
     *   - onHalted() => cancel if needed
     */
    class PlanningAction : public BT::StatefulActionNode
    {
    public:
        using PlanManipulator = manymove_planner::action::PlanManipulator;
        using GoalHandlePlanManipulator = rclcpp_action::ClientGoalHandle<PlanManipulator>;

        /**
         * @brief Constructor for the PlanningAction node.
         * @param name The name of this BT node.
         * @param config The BT NodeConfiguration (ports, blackboard, etc.).
         */
        PlanningAction(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Define the required/optional ports for this node.
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("move_id", "Unique identifier for the move"),
                BT::OutputPort<moveit_msgs::msg::RobotTrajectory>("trajectory", "Planned trajectory"),
                BT::OutputPort<std::string>("planned_move_id", "Echoes move_id for validation"),
                BT::OutputPort<bool>("planning_validity", "Indicates if planning was successful")};
        }

    protected:
        /**
         * @brief Called once when transitioning from IDLE to RUNNING.
         */
        BT::NodeStatus onStart() override;

        /**
         * @brief Called every tick while in RUNNING state.
         */
        BT::NodeStatus onRunning() override;

        /**
         * @brief Called if this node is halted by force.
         */
        void onHalted() override;

    private:
        // Callbacks for action client
        void goalResponseCallback(std::shared_ptr<GoalHandlePlanManipulator> goal_handle);
        void resultCallback(const GoalHandlePlanManipulator::WrappedResult &result);

        // ROS2 members
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<PlanManipulator>::SharedPtr action_client_;

        // Internal state
        bool goal_sent_;
        bool result_received_;

        PlanManipulator::Result action_result_;
        std::string move_id_; ///< Unique move identifier
    };

    /**
     * @class ExecuteTrajectory
     * @brief A stateful BT node that requests trajectory execution from an action server,
     *        implementing a polling approach if needed.
     *
     * Using StatefulActionNode:
     *   - onStart() => check if data is ready (or poll).
     *   - onRunning() => send goal if data is ready, then wait for result.
     *   - onHalted() => cancel if needed.
     */
    class ExecuteTrajectory : public BT::StatefulActionNode
    {
    public:
        using ExecuteTrajectoryAction = manymove_planner::action::ExecuteTrajectory;
        using GoalHandleExecuteTrajectory = rclcpp_action::ClientGoalHandle<ExecuteTrajectoryAction>;

        ExecuteTrajectory(const std::string &name, const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory", "Planned trajectory"),
                BT::InputPort<std::string>("planned_move_id", "Echoes move_id for validation"),
                BT::InputPort<bool>("planning_validity", "Indicates if planning was successful")};
        }

    protected:
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        // Poll data from blackboard
        bool dataReady();
        void sendGoal();

        // Callbacks
        void goalResponseCallback(std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle);
        void resultCallback(const GoalHandleExecuteTrajectory::WrappedResult &result);

        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<ExecuteTrajectoryAction>::SharedPtr action_client_;

        bool goal_sent_;
        bool result_received_;

        // Polled data
        bool planning_valid_;
        std::string move_id_;
        moveit_msgs::msg::RobotTrajectory traj_;

        ExecuteTrajectoryAction::Result action_result_;

        // Timestamps for polling / timeout
        std::chrono::steady_clock::time_point wait_start_time_;
        bool is_data_ready_;

        // Pointer to the blackboard
        BT::Blackboard::Ptr blackboard_;
    };

    /**
     * @class ResetTrajectories
     * @brief A synchronous BT node that resets trajectories and their validity in the blackboard.
     *
     * It takes a comma-separated list of move_ids and for each, sets 'trajectory_{id}' to empty
     * and 'validity_{id}' to false in the blackboard.
     */
    class ResetTrajectories : public BT::SyncActionNode
    {
    public:
        /**
         * @brief Constructor for the ResetTrajectories node.
         * @param name The name of the BT node.
         * @param config The BT NodeConfiguration (ports, blackboard, etc.).
         */
        ResetTrajectories(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Define the required/optional ports for this node.
         */
        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<std::string>("move_ids", "Comma-separated list of move IDs to reset")};
        }

        /**
         * @brief Tick function that performs the reset actions.
         */
        BT::NodeStatus tick() override;

    private:
        // ROS2 node
        rclcpp::Node::SharedPtr node_;
    };

    /**
     * @class AddCollisionObjectAction
     * @brief A Behavior Tree node that adds a collision object to the planning scene using the AddCollisionObject action server.
     */
    class AddCollisionObjectAction : public BT::StatefulActionNode
    {
    public:
        using AddCollisionObject = manymove_object_manager::action::AddCollisionObject;
        using GoalHandleAddCollisionObject = rclcpp_action::ClientGoalHandle<AddCollisionObject>;

        /**
         * @brief Constructor for the AddCollisionObjectAction node.
         * @param name The name of this BT node.
         * @param config The BT NodeConfiguration (ports, blackboard, etc.).
         */
        AddCollisionObjectAction(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Define the required ports for this node.
         * @return A list of input ports.
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("object_id", "Unique identifier for the object"),
                BT::InputPort<std::string>("shape", "Shape type (e.g., box, mesh)"),
                BT::InputPort<std::vector<double>>("dimensions", "Dimensions for primitive shapes"),
                BT::InputPort<geometry_msgs::msg::Pose>("pose", "Pose of the object"),
                BT::InputPort<std::string>("mesh_file", "", "Mesh file path (for mesh objects)"),
                BT::InputPort<double>("scale_mesh_x", 1.0, "Scale factor along X-axis (for mesh)"),
                BT::InputPort<double>("scale_mesh_y", 1.0, "Scale factor along Y-axis (for mesh)"),
                BT::InputPort<double>("scale_mesh_z", 1.0, "Scale factor along Z-axis (for mesh)")};
        }

    protected:
        /**
         * @brief Called once when transitioning from IDLE to RUNNING.
         * @return The initial state of the node after starting.
         */
        BT::NodeStatus onStart() override;

        /**
         * @brief Called every tick while in RUNNING state.
         * @return The current status of the node.
         */
        BT::NodeStatus onRunning() override;

        /**
         * @brief Called if this node is halted by force.
         */
        void onHalted() override;

    private:
        // Callbacks for action client
        void goalResponseCallback(std::shared_ptr<GoalHandleAddCollisionObject> goal_handle);
        void resultCallback(const GoalHandleAddCollisionObject::WrappedResult &result);

        // ROS2 members
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<AddCollisionObject>::SharedPtr action_client_;

        // Internal state
        bool goal_sent_;
        bool result_received_;

        AddCollisionObject::Result action_result_;
        std::string object_id_; ///< Unique object identifier
    };

    /**
     * @class RemoveCollisionObjectAction
     * @brief A Behavior Tree node that removes a collision object from the planning scene using the RemoveCollisionObject action server.
     */
    class RemoveCollisionObjectAction : public BT::StatefulActionNode
    {
    public:
        using RemoveCollisionObject = manymove_object_manager::action::RemoveCollisionObject;
        using GoalHandleRemoveCollisionObject = rclcpp_action::ClientGoalHandle<RemoveCollisionObject>;

        /**
         * @brief Constructor for the RemoveCollisionObjectAction node.
         * @param name The name of this BT node.
         * @param config The BT NodeConfiguration (ports, blackboard, etc.).
         */
        RemoveCollisionObjectAction(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Define the required ports for this node.
         * @return A list of input ports.
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("object_id", "Unique identifier for the object to remove")};
        }

    protected:
        /**
         * @brief Called once when transitioning from IDLE to RUNNING.
         * @return The initial state of the node after starting.
         */
        BT::NodeStatus onStart() override;

        /**
         * @brief Called every tick while in RUNNING state.
         * @return The current status of the node.
         */
        BT::NodeStatus onRunning() override;

        /**
         * @brief Called if this node is halted by force.
         */
        void onHalted() override;

    private:
        // Callbacks for action client
        void goalResponseCallback(std::shared_ptr<GoalHandleRemoveCollisionObject> goal_handle);
        void resultCallback(const GoalHandleRemoveCollisionObject::WrappedResult &result);

        // ROS2 members
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<RemoveCollisionObject>::SharedPtr action_client_;

        // Internal state
        bool goal_sent_;
        bool result_received_;

        RemoveCollisionObject::Result action_result_;
        std::string object_id_; ///< Unique object identifier
    };

    /**
     * @class AttachDetachObjectAction
     * @brief A Behavior Tree node that attaches or detaches a collision object to/from a robot link using the AttachDetachObject action server.
     */
    class AttachDetachObjectAction : public BT::StatefulActionNode
    {
    public:
        using AttachDetachObject = manymove_object_manager::action::AttachDetachObject;
        using GoalHandleAttachDetachObject = rclcpp_action::ClientGoalHandle<AttachDetachObject>;

        /**
         * @brief Constructor for the AttachDetachObjectAction node.
         * @param name The name of this BT node.
         * @param config The BT NodeConfiguration (ports, blackboard, etc.).
         */
        AttachDetachObjectAction(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Define the required ports for this node.
         * @return A list of input ports.
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("object_id", "Unique identifier for the object"),
                BT::InputPort<std::string>("link_name", "Name of the link to attach/detach the object"),
                BT::InputPort<bool>("attach", true, "True to attach, False to detach")};
        }

    protected:
        /**
         * @brief Called once when transitioning from IDLE to RUNNING.
         * @return The initial state of the node after starting.
         */
        BT::NodeStatus onStart() override;

        /**
         * @brief Called every tick while in RUNNING state.
         * @return The current status of the node.
         */
        BT::NodeStatus onRunning() override;

        /**
         * @brief Called if this node is halted by force.
         */
        void onHalted() override;

    private:
        // Callbacks for action client
        void goalResponseCallback(std::shared_ptr<GoalHandleAttachDetachObject> goal_handle);
        void resultCallback(const GoalHandleAttachDetachObject::WrappedResult &result);

        // ROS2 members
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<AttachDetachObject>::SharedPtr action_client_;

        // Internal state
        bool goal_sent_;
        bool result_received_;

        AttachDetachObject::Result action_result_;
        std::string object_id_; ///< Unique object identifier
        std::string link_name_; ///< Link name to attach/detach
        bool attach_;           ///< True to attach, False to detach
    };

    /**
     * @class CheckObjectExistsAction
     * @brief A Behavior Tree node that checks if a collision object exists and whether it's attached using the CheckObjectExists action server.
     */
    class CheckObjectExistsAction : public BT::StatefulActionNode
    {
    public:
        using CheckObjectExists = manymove_object_manager::action::CheckObjectExists;
        using GoalHandleCheckObjectExists = rclcpp_action::ClientGoalHandle<CheckObjectExists>;

        /**
         * @brief Constructor for the CheckObjectExistsAction node.
         * @param name The name of this BT node.
         * @param config The BT NodeConfiguration (ports, blackboard, etc.).
         */
        CheckObjectExistsAction(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Define the required ports for this node.
         * @return A list of input and output ports.
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("object_id", "Unique identifier for the object"),
                BT::OutputPort<bool>("exists", "Indicates if the object exists"),
                BT::OutputPort<bool>("is_attached", "Indicates if the object is attached to a link"),
                BT::OutputPort<std::string>("link_name", "Name of the link the object is attached to, if any")};
        }

    protected:
        /**
         * @brief Called once when transitioning from IDLE to RUNNING.
         * @return The initial state of the node after starting.
         */
        BT::NodeStatus onStart() override;

        /**
         * @brief Called every tick while in RUNNING state.
         * @return The current status of the node.
         */
        BT::NodeStatus onRunning() override;

        /**
         * @brief Called if this node is halted by force.
         */
        void onHalted() override;

    private:
        // Callbacks for action client
        void goalResponseCallback(std::shared_ptr<GoalHandleCheckObjectExists> goal_handle);
        void resultCallback(const GoalHandleCheckObjectExists::WrappedResult &result);

        // ROS2 members
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<CheckObjectExists>::SharedPtr action_client_;

        // Internal state
        bool goal_sent_;
        bool result_received_;

        CheckObjectExists::Result action_result_;
        std::string object_id_; ///< Unique object identifier
    };

    /**
     * @class GetObjectPoseAction
     * @brief A Behavior Tree node that retrieves and modifies the pose of a collision object using the GetObjectPose action server.
     */
    class GetObjectPoseAction : public BT::StatefulActionNode
    {
    public:
        using GetObjectPose = manymove_object_manager::action::GetObjectPose;
        using GoalHandleGetObjectPose = rclcpp_action::ClientGoalHandle<GetObjectPose>;

        /**
         * @brief Constructor for the GetObjectPoseAction node.
         * @param name The name of this BT node.
         * @param config The BT NodeConfiguration (ports, blackboard, etc.).
         */
        GetObjectPoseAction(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Define the required ports for this node.
         * @return A list of input and output ports.
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("object_id", "Identifier of the object"),
                BT::InputPort<std::string>("first_rotation_axis", "", "First rotation axis (X, Y, Z)"),
                BT::InputPort<double>("first_rotation_rad", 0.0, "First rotation in radians"),
                BT::InputPort<std::string>("second_rotation_axis", "", "Second rotation axis (X, Y, Z)"),
                BT::InputPort<double>("second_rotation_rad", 0.0, "Second rotation in radians"),
                BT::OutputPort<geometry_msgs::msg::Pose>("modified_pose", "Pose after rotations")};
        }

    protected:
        /**
         * @brief Called once when transitioning from IDLE to RUNNING.
         * @return The initial state of the node after starting.
         */
        BT::NodeStatus onStart() override;

        /**
         * @brief Called every tick while in RUNNING state.
         * @return The current status of the node.
         */
        BT::NodeStatus onRunning() override;

        /**
         * @brief Called if this node is halted by force.
         */
        void onHalted() override;

    private:
        // Callbacks for action client
        void goalResponseCallback(std::shared_ptr<GoalHandleGetObjectPose> goal_handle);
        void resultCallback(const GoalHandleGetObjectPose::WrappedResult &result);

        // ROS2 members
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<GetObjectPose>::SharedPtr action_client_;

        // Internal state
        bool goal_sent_;
        bool result_received_;

        GetObjectPose::Result action_result_;
        std::string object_id_;            ///< Unique object identifier
        std::string first_rotation_axis_;  ///< First rotation axis
        double first_rotation_rad_;        ///< First rotation angle in radians
        std::string second_rotation_axis_; ///< Second rotation axis
        double second_rotation_rad_;       ///< Second rotation angle in radians
    };

} // namespace manymove_cpp_trees

#endif
