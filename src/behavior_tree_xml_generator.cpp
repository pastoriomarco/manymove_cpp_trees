// src/manymove_cpp_trees/src/behavior_tree_xml_generator.cpp

#include "manymove_cpp_trees/behavior_tree_xml_generator.hpp"
#include <sstream>
#include <rclcpp/rclcpp.hpp> // For logging (optional)

namespace manymove_cpp_trees
{

BehaviorTreeXMLGenerator::BehaviorTreeXMLGenerator(const std::vector<std::vector<Move>>& sequences)
    : sequences_(sequences)
{
}

std::string BehaviorTreeXMLGenerator::generateXML() const
{
    std::ostringstream xml;
    xml << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    xml << "<root main_tree_to_execute=\"RootSequence\">\n";
    xml << "  <BehaviorTree ID=\"RootSequence\">\n";

    // Create the root sequence node
    xml << "    <Sequence name=\"Root\">\n";

    if (sequences_.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("bt_client_node"), "No move sequences provided.");
        xml << "    </Sequence>\n";
        xml << "  </BehaviorTree>\n";
        xml << "</root>\n";
        return xml.str();
    }

    // Iterate over each sequence
    for (size_t i = 0; i < sequences_.size(); ++i)
    {
        const auto& sequence = sequences_[i];
        std::ostringstream plan_sequence_name;
        plan_sequence_name << "PlanSequence_" << i;
        xml << "      <Sequence name=\"" << plan_sequence_name.str() << "\">\n";

        for (size_t j = 0; j < sequence.size(); ++j)
        {
            const auto& move = sequence[j];

            // 1) Convert Move -> MoveManipulatorGoal
            manymove_planner::msg::MoveManipulatorGoal move_goal;
            move_goal.movement_type = move.type;

            if (move.type == "pose")
            {
                move_goal.pose_target = move.pose_target;
            }
            else if (move.type == "joint")
            {
                move_goal.joint_values = move.joint_values;
            }
            else if (move.type == "named")
            {
                move_goal.named_target = move.named_target;
            }

            // Copy config
            move_goal.config = move.config;

            // 2) Serialize the goal
            std::string serialized_goal = serializeMoveManipulatorGoal(move_goal);

            // 3) Build a move_id from (i, move.type, j)
            std::ostringstream move_id_oss;
            move_id_oss << i << "_" << move.type << "_" << j;
            std::string move_id_str = move_id_oss.str();

            // 4) Define the blackboard keys for trajectory, planned_move_id, and planning_validity
            std::string traj_key = "traj_" + move_id_str;
            std::string planned_move_id_key = "planned_move_id_" + move_id_str;
            std::string planning_validity_key = "planning_validity_" + move_id_str;

            // 5) Create the <PlanningAction> node with proper port mappings
            xml << "        <PlanningAction name=\"PlanAction_" << move_id_str
                << "\" goal=\"" << serialized_goal
                << "\" move_id=\"" << move_id_str << "\" "
                << "trajectory=\"{" << traj_key << "}\" "
                << "planned_move_id=\"{" << planned_move_id_key << "}\" "
                << "planning_validity=\"{" << planning_validity_key << "}\" />\n";

            // 6) Create the corresponding <ExecuteTrajectory> node
            xml << "        <ExecuteTrajectory name=\"ExecAction_" << move_id_str
                << "\" trajectory=\"{" << traj_key << "}\" "
                << "planned_move_id=\"{" << planned_move_id_key << "}\" "
                << "planning_validity=\"{" << planning_validity_key << "}\" "
                << "validity=\"{validity_" << move_id_str << "}\" />\n";
        }

        xml << "      </Sequence>\n";
    }

    // Close the root sequence node
    xml << "    </Sequence>\n";
    xml << "  </BehaviorTree>\n";
    xml << "</root>\n";

    return xml.str();
}

} // namespace manymove_cpp_trees
