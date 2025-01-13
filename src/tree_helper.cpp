#include "manymove_cpp_trees/tree_helper.hpp"
#include <sstream>
#include <rclcpp/rclcpp.hpp>

// A static global counter to ensure unique move IDs across the entire tree
static int g_global_move_id = 0;

namespace manymove_cpp_trees
{

    std::unordered_map<std::string, manymove_planner::msg::MovementConfig>
    defineMovementConfigs()
    {
        using manymove_planner::msg::MovementConfig;

        MovementConfig max_move_config;
        max_move_config.velocity_scaling_factor = 1.0;
        max_move_config.acceleration_scaling_factor = 1.0;
        max_move_config.step_size = 0.01;
        max_move_config.jump_threshold = 0.0;
        max_move_config.max_cartesian_speed = 0.5;
        max_move_config.max_exec_tries = 5;
        max_move_config.plan_number_target = 8;
        max_move_config.plan_number_limit = 32;
        max_move_config.smoothing_type = "time_optimal";

        MovementConfig mid_move_config = max_move_config;
        mid_move_config.velocity_scaling_factor /= 2.0;
        mid_move_config.acceleration_scaling_factor /= 2.0;
        mid_move_config.max_cartesian_speed = 0.2;

        MovementConfig slow_move_config = max_move_config;
        slow_move_config.velocity_scaling_factor /= 4.0;
        slow_move_config.acceleration_scaling_factor /= 4.0;
        slow_move_config.max_cartesian_speed = 0.05;

        return {
            {"max_move", max_move_config},
            {"mid_move", mid_move_config},
            {"slow_move", slow_move_config}};
    }

    geometry_msgs::msg::Pose createPose(double x, double y, double z,
                                        double qx, double qy, double qz, double qw)
    {
        geometry_msgs::msg::Pose p;
        p.position.x = x;
        p.position.y = y;
        p.position.z = z;
        p.orientation.x = qx;
        p.orientation.y = qy;
        p.orientation.z = qz;
        p.orientation.w = qw;
        return p;
    }

    std::string buildParallelPlanExecuteXML(const std::string &prefix,
                                            const std::vector<Move> &moves)
    {
        /**
         * We'll create something like:
         *
         *   int blockStartID = g_global_move_id; // store the first ID used for this block
         * <Sequence name="PlanExecute_{prefix}_{blockStartID}">
         *   <Sequence name="ResetTrajectories_{prefix}_{blockStartID}">
         *      [reset all traj and validities for the moves of this sequences]
         *   </Sequence>
         *   <Parallel name="ParallelPlanExecute_{prefix}_{blockStartID}" success_threshold="2" failure_threshold="1">
         *     <Sequence name="PlanningSequence_{prefix}_{blockStartID}">
         *       [ for each Move in moves => use g_global_move_id for the naming and blackboard keys, then increment ]
         *         <PlanningAction name="PlanMove_{g_global_move_id}" move_id="{g_global_move_id}"
         *                         planned_move_id="{planned_move_id_{g_global_move_id}}"
         *                         trajectory="{trajectory_{g_global_move_id}}"
         *                         planning_validity="{validity_{g_global_move_id}}"/>
         *     </Sequence>
         *     <Sequence name="ExecutionSequence_{prefix}_{blockStartID}">
         *       [ same for ExecMove_{g_global_move_id} ... ]
         *     </Sequence>
         *   </Parallel>
         * </Sequence>
         */

        std::ostringstream xml;

        // The first ID used in this block
        int blockStartID = g_global_move_id;

        // Parallel tag
        xml << "  <Parallel name=\"ParallelPlanExecute_" << prefix << "_" << blockStartID
            << "\" success_threshold=\"2\" failure_threshold=\"1\">\n";

        // 1) Planning Sequence
        xml << "    <Sequence name=\"PlanningSequence_" << prefix << "_" << blockStartID << "\">\n";

        // We'll store the IDs of each move in a local array so that the execution side uses the same IDs
        std::vector<int> move_ids;
        move_ids.reserve(moves.size());

        for (size_t i = 0; i < moves.size(); i++)
        {
            int this_move_id = g_global_move_id; // unique ID for this move
            move_ids.push_back(this_move_id);

            xml << "      <PlanningAction"
                << " name=\"PlanMove_" << this_move_id << "\""
                << " move_id=\"" << this_move_id << "\""
                << " planned_move_id=\"{planned_move_id_" << this_move_id << "}\""
                << " trajectory=\"{trajectory_" << this_move_id << "}\""
                << " planning_validity=\"{validity_" << this_move_id << "}\""
                << "/>\n";

            // increment the global ID for the next move
            g_global_move_id++;
        }

        xml << "    </Sequence>\n";

        // 2) Execution Sequence
        xml << "    <Sequence name=\"ExecutionSequence_" << prefix << "_" << blockStartID << "\">\n";

        // Use the same move_ids array
        for (int mid : move_ids)
        {
            xml << "      <ExecuteTrajectory"
                << " name=\"ExecMove_" << mid << "\""
                << " planned_move_id=\"{planned_move_id_" << mid << "}\""
                << " trajectory=\"{trajectory_" << mid << "}\""
                << " planning_validity=\"{validity_" << mid << "}\""
                << "/>\n";
        }

        xml << "    </Sequence>\n";

        xml << "  </Parallel>\n";

        return xml.str();
    }

    std::string sequenceWrapperXML(const std::string &sequence_name,
                                   const std::vector<std::string> &branches)
    {
        std::ostringstream xml;
        xml << "  <Sequence name=\"" << sequence_name << "\">\n";
        for (auto &b : branches)
        {
            xml << b << "\n";
        }
        xml << "  </Sequence>\n";
        return xml.str();
    }

    std::string reactiveWrapperXML(const std::string &sequence_name,
                                   const std::vector<std::string> &branches)
    {
        /**
         * Creates a ReactiveSequence XML snippet that can be used to wrap multiple branches.
         *
         * Example output:
         *
         *   <ReactiveSequence name="ReactiveSequence_{sequence_name}">
         *     {branches}
         *   </ReactiveSequence>
         */
        std::ostringstream xml;
        xml << "  <ReactiveSequence name=\"" << sequence_name << "\">\n";
        for (auto &b : branches)
        {
            xml << b << "\n";
        }
        xml << "  </ReactiveSequence>\n";
        return xml.str();
    }

    std::string mainTreeWrapperXML(const std::string &tree_id,
                                   const std::string &content)
    {
        std::ostringstream xml;
        xml << R"(<?xml version="1.0" encoding="UTF-8"?>)" << "\n";
        xml << "<root main_tree_to_execute=\"" << tree_id << "\">\n";
        xml << "  <BehaviorTree ID=\"" << tree_id << "\">\n";
        xml << content << "\n";
        xml << "  </BehaviorTree>\n";
        xml << "</root>\n";
        return xml.str();
    }

} // namespace manymove_cpp_trees
