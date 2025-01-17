#include "manymove_cpp_trees/tree_helper.hpp"
#include <sstream>
#include <rclcpp/rclcpp.hpp>

// A static global counter to ensure unique move IDs across the entire tree
static int g_global_move_id = 0;

namespace manymove_cpp_trees
{

    // ----------------------------------------------------------------------------
    // Setup functions
    // ----------------------------------------------------------------------------

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

    // ----------------------------------------------------------------------------
    // Builder functions to build xml tree snippets programmatically
    // ----------------------------------------------------------------------------

    std::string buildParallelPlanExecuteXML(const std::string &prefix,
                                            const std::vector<Move> &moves,
                                            BT::Blackboard::Ptr blackboard,
                                            bool reset_trajs)
    {
        std::ostringstream xml;

        // The first ID used in this block
        int blockStartID = g_global_move_id;

        // Collect move_ids
        std::vector<int> move_ids;
        move_ids.reserve(moves.size());

        // Planning Sequence
        std::ostringstream planning_seq;
        planning_seq << "    <Sequence name=\"PlanningSequence_" << prefix << "_" << blockStartID << "\">\n";

        for (const auto &move : moves)
        {
            int this_move_id = g_global_move_id; // unique ID for this move
            move_ids.push_back(this_move_id);

            // Populate the blackboard with the move
            std::string key = "move_" + std::to_string(this_move_id);
            blackboard->set(key, std::make_shared<Move>(move));
            RCLCPP_INFO(rclcpp::get_logger("bt_client_node"),
                        "BB set: %s", key.c_str());

            planning_seq << "      <PlanningAction"
                         << " name=\"PlanMove_" << this_move_id << "\""
                         << " move_id=\"" << this_move_id << "\""
                         << " planned_move_id=\"{planned_move_id_" << this_move_id << "}\""
                         << " trajectory=\"{trajectory_" << this_move_id << "}\""
                         << " planning_validity=\"{validity_" << this_move_id << "}\""
                         << " pose_key=\"" << move.pose_key << "\""
                         << "/>\n";

            // increment the global ID for the next move
            g_global_move_id++;
        }

        planning_seq << "    </Sequence>\n";

        // Execution Sequence
        std::ostringstream execution_seq;
        execution_seq << "    <Sequence name=\"ExecutionSequence_" << prefix << "_" << blockStartID << "\">\n";

        for (int mid : move_ids)
        {
            execution_seq << "      <ExecuteTrajectory"
                          << " name=\"ExecMove_" << mid << "\""
                          << " planned_move_id=\"{planned_move_id_" << mid << "}\""
                          << " trajectory=\"{trajectory_" << mid << "}\""
                          << " planning_validity=\"{validity_" << mid << "}\""
                          << "/>\n";
        }

        execution_seq << "    </Sequence>\n";

        // Parallel node
        std::ostringstream parallel_node;
        parallel_node << "  <Parallel name=\"ParallelPlanExecute_" << prefix << "_" << blockStartID
                      << "\" success_threshold=\"2\" failure_threshold=\"1\">\n"
                      << planning_seq.str()
                      << execution_seq.str()
                      << "  </Parallel>\n";

        if (reset_trajs)
        { // ResetTrajectories node
            std::ostringstream reset_node;
            reset_node << "  <ResetTrajectories move_ids=\"";
            for (size_t i = 0; i < move_ids.size(); i++)
            {
                reset_node << move_ids[i];
                if (i != move_ids.size() - 1)
                    reset_node << ",";
            }
            reset_node << "\"/>\n";

            // Insert ResetTrajectories first
            xml << reset_node.str();
        }

        // Insert Parallel nodes
        xml << parallel_node.str();

        return xml.str();
    }

    std::string buildObjectActionXML(const std::string &prefix, const ObjectAction &action)
    {
        // Generate a unique node name using the prefix and object_id
        std::string node_name = prefix + "_" + action.object_id + "_" + objectActionTypeToString(action.type);

        // Start constructing the XML node
        std::ostringstream xml;
        xml << "<" << objectActionTypeToString(action.type) << " ";
        xml << "name=\"" << node_name << "\" ";
        xml << "object_id=\"" << action.object_id << "\" ";

        // Handle different action types
        switch (action.type)
        {
        case ObjectActionType::ADD:
        {
            // Shape attribute
            xml << "shape=\"" << action.shape << "\" ";

            if (action.shape != "mesh")
            {
                // Serialize dimensions
                std::string dimensions_str = serializeVector(action.dimensions);
                xml << "dimensions=\"" << dimensions_str << "\" ";
            }
            else
            {
                // Mesh-specific attributes
                xml << "mesh_file=\"" << action.mesh_file << "\" ";
                xml << "scale_mesh_x=\"" << action.scale_mesh_x << "\" ";
                xml << "scale_mesh_y=\"" << action.scale_mesh_y << "\" ";
                xml << "scale_mesh_z=\"" << action.scale_mesh_z << "\" ";
            }

            // Serialize pose
            std::string pose_str = serializePose(action.pose);
            xml << "pose=\"" << pose_str << "\" ";
            break;
        }
        case ObjectActionType::REMOVE:
        {
            // No additional attributes needed
            break;
        }
        case ObjectActionType::ATTACH:
        case ObjectActionType::DETACH:
        {
            // Link name and attach flag
            xml << "link_name=\"" << action.link_name << "\" ";
            xml << "attach=\"" << (action.attach ? "true" : "false") << "\" ";
            break;
        }
        case ObjectActionType::CHECK:
        {
            // No additional attributes needed
            break;
        }
        case ObjectActionType::GET_POSE:
        {
            // Serialize pre_transform_xyz_rpy and post_transform_xyz_rpy
            std::string transform_str = serializeVector(action.pre_transform_xyz_rpy);
            std::string reference_orient_str = serializeVector(action.post_transform_xyz_rpy);
            xml << "pre_transform_xyz_rpy=\"" << transform_str << "\" ";
            xml << "post_transform_xyz_rpy=\"" << reference_orient_str << "\" ";

            // Serialize pose_key if it's not empty
            if (!action.pose_key.empty())
            {
                xml << "pose_key=\"" << action.pose_key << "\" ";
            }
            break;
        }
        default:
            throw std::invalid_argument("Unsupported ObjectActionType in buildObjectActionXML");
        }

        // Close the XML node
        xml << "/>";

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
        std::ostringstream xml;
        xml << "  <ReactiveSequence name=\"" << sequence_name << "\">\n";
        for (auto &b : branches)
        {
            xml << b << "\n";
        }
        xml << "  </ReactiveSequence>\n";
        return xml.str();
    }

    std::string repeatWrapperXML(const std::string &sequence_name,
                                 const std::vector<std::string> &branches,
                                 const int num_cycles)
    {
        std::ostringstream xml;
        xml << "  <Repeat name=\"" << sequence_name << "\" num_cycles=\"" << num_cycles << "\">\n";
        xml << "    <Sequence name=\"" << sequence_name << "_sequence\">\n";
        for (const auto &b : branches)
        {
            xml << b << "\n";
        }
        xml << "    </Sequence>\n";
        xml << "  </Repeat>\n";
        return xml.str();
    }

    std::string fallbackWrapperXML(const std::string &sequence_name,
                                   const std::vector<std::string> &branches)
    {
        std::ostringstream xml;
        xml << "  <Fallback name=\"" << sequence_name << "\">\n";
        for (auto &b : branches)
        {
            xml << b << "\n";
        }
        xml << "  </Fallback>\n";
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

    // ----------------------------------------------------------------------------
    // Helper functions
    // ----------------------------------------------------------------------------

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

    geometry_msgs::msg::Pose createPoseRPY(const double &x,
                                           const double &y,
                                           const double &z,
                                           const double &roll,
                                           const double &pitch,
                                           const double &yaw)
    {
        auto pose = geometry_msgs::msg::Pose();

        // Set position
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        // Convert roll, pitch, yaw to a quaternion
        tf2::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);

        // Set orientation
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pose.orientation.w = quaternion.w();

        return pose;
    }

    std::string objectActionTypeToString(ObjectActionType type)
    {
        switch (type)
        {
        case ObjectActionType::ADD:
            return "AddCollisionObjectAction";
        case ObjectActionType::REMOVE:
            return "RemoveCollisionObjectAction";
        case ObjectActionType::ATTACH:
        case ObjectActionType::DETACH:
            return "AttachDetachObjectAction";
        case ObjectActionType::CHECK:
            return "CheckObjectExistsAction";
        case ObjectActionType::GET_POSE:
            return "GetObjectPoseAction";
        default:
            throw std::invalid_argument("Unsupported ObjectActionType");
        }
    }

    std::string serializePose(const geometry_msgs::msg::Pose &pose)
    {
        std::ostringstream oss;
        oss << "position: {x: " << pose.position.x
            << ", y: " << pose.position.y
            << ", z: " << pose.position.z
            << "}, orientation: {x: " << pose.orientation.x
            << ", y: " << pose.orientation.y
            << ", z: " << pose.orientation.z
            << ", w: " << pose.orientation.w
            << "}";
        return oss.str();
    }

    std::string serializeVector(const std::vector<double> &vec)
    {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < vec.size(); ++i)
        {
            oss << vec[i];
            if (i != vec.size() - 1)
                oss << ",";
        }
        oss << "]";
        return oss.str();
    }

} // namespace manymove_cpp_trees
