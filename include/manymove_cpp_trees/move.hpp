#ifndef MANYMOVE_CPP_TREES_MOVE_HPP
#define MANYMOVE_CPP_TREES_MOVE_HPP

#include <string>
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "manymove_planner/msg/movement_config.hpp"
#include "manymove_planner/msg/move_manipulator_goal.hpp"

namespace manymove_cpp_trees
{
    inline manymove_planner::msg::MovementConfig defaultMovementConfig()
    {
        manymove_planner::msg::MovementConfig config;
        config.velocity_scaling_factor = 1.0;
        config.acceleration_scaling_factor = 1.0;
        config.step_size = 0.01;
        config.jump_threshold = 0.0;
        config.max_cartesian_speed = 0.5;
        config.max_exec_tries = 5;
        config.plan_number_target = 8;
        config.plan_number_limit = 32;
        config.smoothing_type = "time_optimal";
        return config;
    }

    struct Move
    {
        std::string type;                             // "pose", "joint", or "named"
        geometry_msgs::msg::Pose pose_target;         // For "pose" type
        std::vector<double> joint_values;             // For "joint" type
        std::string named_target;                     // For "named" type
        manymove_planner::msg::MovementConfig config; // Configuration parameters
        std::vector<double> start_joint_values;       // Starting joint values for planning

        // Constructor with start_joint_values defaulted to empty
        Move(const std::string &type,
             const std::vector<double> &joint_values = {},
             const geometry_msgs::msg::Pose &pose_target = geometry_msgs::msg::Pose(),
             const std::string &named_target = "",
             const manymove_planner::msg::MovementConfig &config = defaultMovementConfig(),
             const std::vector<double> &start_joint_values = {})
            : type(type),
              pose_target(pose_target),
              joint_values(joint_values),
              named_target(named_target),
              config(config),
              start_joint_values(start_joint_values)
        {
        }

        // Method to convert to MoveManipulatorGoal
        manymove_planner::msg::MoveManipulatorGoal to_move_manipulator_goal() const
        {
            manymove_planner::msg::MoveManipulatorGoal goal;
            goal.movement_type = type;

            if (type == "pose")
            {
                goal.pose_target = pose_target;
            }
            else if (type == "joint")
            {
                goal.joint_values = joint_values;
            }
            else if (type == "named")
            {
                goal.named_target = named_target;
            }

            goal.start_joint_values = start_joint_values;
            goal.config = config;
            return goal;
        }
    };

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_MOVE_HPP
