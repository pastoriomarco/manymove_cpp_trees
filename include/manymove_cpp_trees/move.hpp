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

    /**
     * @struct Move
     * @brief Represents a single move command with a specified type and configuration.
     *
     * This struct is used to store details of a single move, including its type
     * (e.g., "pose", "joint", "named", or "cartesian"), any relevant pose or joint data,
     * and the associated MovementConfig.
     */
    struct Move
    {
        std::string type;                             ///< The movement type: "pose", "joint", "named", or "cartesian".
        geometry_msgs::msg::Pose pose_target;         ///< Pose target for "pose" or "cartesian" type.
        std::vector<double> joint_values;             ///< Joint values for "joint" type.
        std::string named_target;                     ///< Named target for "named" type.
        manymove_planner::msg::MovementConfig config; ///< Movement configuration parameters.
        std::vector<double> start_joint_values;       ///< Starting joint values for planning (optional).

        /**
         * @brief Constructor to create a Move object.
         * @param type The movement type ("pose", "joint", "named", or "cartesian").
         * @param joint_values The joint values (for "joint" type).
         * @param pose_target The pose (for "pose" or "cartesian" type).
         * @param named_target The named target (for "named" type).
         * @param config The movement configuration.
         * @param start_joint_values The optional starting joint values.
         */
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

        /**
         * @brief Convert this Move struct into a MoveManipulatorGoal message for planning/execution.
         * @return A populated MoveManipulatorGoal message reflecting the content of this Move.
         */
        manymove_planner::msg::MoveManipulatorGoal to_move_manipulator_goal() const
        {
            manymove_planner::msg::MoveManipulatorGoal goal;
            goal.movement_type = type;

            // "pose" and "cartesian" both rely on pose_target
            if (type == "pose" || type == "cartesian")
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
