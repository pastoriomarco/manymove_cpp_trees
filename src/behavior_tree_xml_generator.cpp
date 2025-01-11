#include "manymove_cpp_trees/behavior_tree_xml_generator.hpp"
#include <sstream>
#include <rclcpp/rclcpp.hpp>

namespace manymove_cpp_trees
{

    BehaviorTreeXMLGenerator::BehaviorTreeXMLGenerator(const std::vector<std::vector<Move>> &sequences)
        : sequences_(sequences)
    {
    }

    std::string BehaviorTreeXMLGenerator::generateXML() const
    {
        std::ostringstream xml;
        xml << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        xml << "<root main_tree_to_execute=\"RootSequence\">\n";
        xml << "  <BehaviorTree ID=\"RootSequence\">\n";

        // Root Sequence node handles infinite looping by never returning SUCCESS
        xml << "    <Sequence name=\"Root\">\n";

        if (sequences_.empty())
        {
            RCLCPP_WARN(rclcpp::get_logger("bt_client_node"), "No move sequences provided.");
            xml << "      </Sequence>\n";
            xml << "  </BehaviorTree>\n";
            xml << "</root>\n";
            return xml.str();
        }

        // Flatten all moves into a single list with unique move indices
        std::vector<Move> all_moves;
        size_t global_move_id = 0;
        for (const auto &sequence : sequences_)
        {
            for (const auto &move : sequence)
            {
                all_moves.emplace_back(move);
                global_move_id++;
            }
        }

        // Iterate through all moves and build Plan and Exec sequences
        for (size_t i = 0; i < all_moves.size(); ++i)
        {
            const auto &move = all_moves[i];
            std::ostringstream move_id_oss;
            move_id_oss << i;
            std::string move_id_str = move_id_oss.str();

            // Define the blackboard keys for trajectory, planned_move_id, and planning_validity
            std::string traj_key = "traj_" + move_id_str;
            std::string planned_move_id_key = "planned_move_id_" + move_id_str;
            std::string planning_validity_key = "planning_validity_" + move_id_str;

            if (i == 0)
            {
                // Initial PlanSequence for the first move
                xml << "      <Sequence name=\"PlanSequence_" << i << "\">\n";
                xml << "        <PlanningAction name=\"PlanAction_" << move_id_str
                    << "\" move_id=\"" << move_id_str << "\" "
                    << "trajectory=\"{" << traj_key << "}\" "
                    << "planned_move_id=\"{" << planned_move_id_key << "}\" "
                    << "planning_validity=\"{" << planning_validity_key << "}\" />\n";
                xml << "      </Sequence>\n";

                // Execute the first move while planning the second move
                // Determine number of children for Parallel node
                size_t num_children = (i + 1 < all_moves.size()) ? 2 : 1;

                xml << "      <Parallel name=\"Parallel_Exec" << i << "_Plan" << (i + 1)
                    << "\" success_threshold=\"" << num_children << "\" failure_threshold=\"1\">\n";

                // ExecSequence_0
                xml << "        <Sequence name=\"ExecSequence_" << i << "\">\n";
                xml << "          <ExecuteTrajectory name=\"ExecAction_" << move_id_str
                    << "\" trajectory=\"{" << traj_key << "}\" "
                    << "planned_move_id=\"{" << planned_move_id_key << "}\" "
                    << "planning_validity=\"{" << planning_validity_key << "}\" />\n"; // Removed validity port
                xml << "        </Sequence>\n";

                // PlanSequence_1 (if exists)
                if (i + 1 < all_moves.size())
                {
                    const auto &next_move = all_moves[i + 1];
                    std::ostringstream next_move_id_oss;
                    next_move_id_oss << (i + 1);
                    std::string next_move_id_str = next_move_id_oss.str();

                    std::string next_traj_key = "traj_" + next_move_id_str;
                    std::string next_planned_move_id_key = "planned_move_id_" + next_move_id_str;
                    std::string next_planning_validity_key = "planning_validity_" + next_move_id_str;

                    xml << "        <Sequence name=\"PlanSequence_" << (i + 1) << "\">\n";
                    xml << "          <PlanningAction name=\"PlanAction_" << next_move_id_str
                        << "\" move_id=\"" << next_move_id_str << "\" "
                        << "trajectory=\"{" << next_traj_key << "}\" "
                        << "planned_move_id=\"{" << next_planned_move_id_key << "}\" "
                        << "planning_validity=\"{" << next_planning_validity_key << "}\" />\n";
                    xml << "        </Sequence>\n";
                }

                xml << "      </Parallel>\n";
            }
            else
            {
                // For subsequent moves, wrap them in Parallel nodes
                // Determine number of children for Parallel node
                size_t num_children = (i + 1 < all_moves.size()) ? 2 : 1;

                xml << "      <Parallel name=\"Parallel_Exec" << i << "_Plan" << (i + 1)
                    << "\" success_threshold=\"" << num_children << "\" failure_threshold=\"1\">\n";

                // ExecSequence_i
                xml << "        <Sequence name=\"ExecSequence_" << i << "\">\n";
                xml << "          <ExecuteTrajectory name=\"ExecAction_" << move_id_str
                    << "\" trajectory=\"{" << traj_key << "}\" "
                    << "planned_move_id=\"{" << planned_move_id_key << "}\" "
                    << "planning_validity=\"{" << planning_validity_key << "}\" />\n"; // Removed validity port
                xml << "        </Sequence>\n";

                // PlanSequence_{i+1} (if exists)
                if (i + 1 < all_moves.size())
                {
                    const auto &next_move = all_moves[i + 1];
                    std::ostringstream next_move_id_oss;
                    next_move_id_oss << (i + 1);
                    std::string next_move_id_str = next_move_id_oss.str();

                    std::string next_traj_key = "traj_" + next_move_id_str;
                    std::string next_planned_move_id_key = "planned_move_id_" + next_move_id_str;
                    std::string next_planning_validity_key = "planning_validity_" + next_move_id_str;

                    xml << "        <Sequence name=\"PlanSequence_" << (i + 1) << "\">\n";
                    xml << "          <PlanningAction name=\"PlanAction_" << next_move_id_str
                        << "\" move_id=\"" << next_move_id_str << "\" "
                        << "trajectory=\"{" << next_traj_key << "}\" "
                        << "planned_move_id=\"{" << next_planned_move_id_key << "}\" "
                        << "planning_validity=\"{" << next_planning_validity_key << "}\" />\n";
                    xml << "        </Sequence>\n";
                }

                xml << "      </Parallel>\n";
            }
        }

        // Close the root sequence
        xml << "    </Sequence>\n";
        xml << "  </BehaviorTree>\n";
        xml << "</root>\n";

        return xml.str();
    }

} // namespace manymove_cpp_trees
