#include "manymove_cpp_trees/serialization_helper.hpp"
#include <sstream>
#include <stdexcept>

namespace manymove_cpp_trees
{

std::string serializeMoveManipulatorGoal(const manymove_planner::msg::MoveManipulatorGoal& goal)
{
    std::ostringstream oss;
    oss << "movement_type=" << goal.movement_type << ";";

    if (goal.movement_type == "pose" || goal.movement_type == "cartesian")
    {
        const auto& pos = goal.pose_target.position;
        const auto& ori = goal.pose_target.orientation;
        oss << "position=" << pos.x << "," << pos.y << "," << pos.z << ";";
        oss << "orientation=" << ori.x << "," << ori.y << "," << ori.z << "," << ori.w << ";";
    }
    else if (goal.movement_type == "joint")
    {
        oss << "joint_values=";
        for (size_t i = 0; i < goal.joint_values.size(); ++i)
        {
            oss << goal.joint_values[i];
            if (i < goal.joint_values.size() - 1)
                oss << ",";
        }
        oss << ";";
    }
    else if (goal.movement_type == "named")
    {
        oss << "named_target=" << goal.named_target << ";";
    }

    const auto& config = goal.config;
    oss << "velocity_scaling_factor=" << config.velocity_scaling_factor << ";";
    oss << "acceleration_scaling_factor=" << config.acceleration_scaling_factor << ";";
    oss << "step_size=" << config.step_size << ";";
    oss << "jump_threshold=" << config.jump_threshold << ";";
    oss << "max_cartesian_speed=" << config.max_cartesian_speed << ";";
    oss << "max_exec_tries=" << config.max_exec_tries << ";";
    oss << "plan_number_target=" << config.plan_number_target << ";";
    oss << "plan_number_limit=" << config.plan_number_limit << ";";
    oss << "smoothing_type=" << config.smoothing_type << ";";

    return oss.str();
}

manymove_planner::msg::MoveManipulatorGoal deserializeMoveManipulatorGoal(const std::string& serialized_goal)
{
    manymove_planner::msg::MoveManipulatorGoal goal;
    std::istringstream iss(serialized_goal);
    std::string token;

    while (std::getline(iss, token, ';'))
    {
        auto delimiter_pos = token.find('=');
        if (delimiter_pos == std::string::npos)
            continue;

        std::string key = token.substr(0, delimiter_pos);
        std::string value = token.substr(delimiter_pos + 1);

        if (key == "movement_type")
        {
            goal.movement_type = value;
        }
        else if (key == "position")
        {
            std::istringstream pos_stream(value);
            std::string coord;
            std::getline(pos_stream, coord, ',');
            goal.pose_target.position.x = std::stod(coord);
            std::getline(pos_stream, coord, ',');
            goal.pose_target.position.y = std::stod(coord);
            std::getline(pos_stream, coord, ',');
            goal.pose_target.position.z = std::stod(coord);
        }
        else if (key == "orientation")
        {
            std::istringstream ori_stream(value);
            std::string coord;
            std::getline(ori_stream, coord, ',');
            goal.pose_target.orientation.x = std::stod(coord);
            std::getline(ori_stream, coord, ',');
            goal.pose_target.orientation.y = std::stod(coord);
            std::getline(ori_stream, coord, ',');
            goal.pose_target.orientation.z = std::stod(coord);
            std::getline(ori_stream, coord, ',');
            goal.pose_target.orientation.w = std::stod(coord);
        }
        else if (key == "joint_values")
        {
            std::istringstream joint_stream(value);
            std::string joint_value;
            while (std::getline(joint_stream, joint_value, ','))
            {
                goal.joint_values.push_back(std::stod(joint_value));
            }
        }
        else if (key == "named_target")
        {
            goal.named_target = value;
        }
        else if (key == "velocity_scaling_factor")
        {
            goal.config.velocity_scaling_factor = std::stod(value);
        }
        else if (key == "acceleration_scaling_factor")
        {
            goal.config.acceleration_scaling_factor = std::stod(value);
        }
        else if (key == "step_size")
        {
            goal.config.step_size = std::stod(value);
        }
        else if (key == "jump_threshold")
        {
            goal.config.jump_threshold = std::stod(value);
        }
        else if (key == "max_cartesian_speed")
        {
            goal.config.max_cartesian_speed = std::stod(value);
        }
        else if (key == "max_exec_tries")
        {
            goal.config.max_exec_tries = std::stoi(value);
        }
        else if (key == "plan_number_target")
        {
            goal.config.plan_number_target = std::stoi(value);
        }
        else if (key == "plan_number_limit")
        {
            goal.config.plan_number_limit = std::stoi(value);
        }
        else if (key == "smoothing_type")
        {
            goal.config.smoothing_type = value;
        }
    }

    return goal;
}

} // namespace manymove_cpp_trees
