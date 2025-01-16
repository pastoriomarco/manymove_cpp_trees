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
        oss << "position: {x: " << pose.position.x << ", y: " << pose.position.y << ", z: " << pose.position.z << "}, ";
        oss << "orientation: {x: " << pose.orientation.x << ", y: " << pose.orientation.y << ", ";
        oss << "z: " << pose.orientation.z << ", w: " << pose.orientation.w << "}";
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

    std::string buildObjectActionXML(const std::string &prefix, const ObjectAction &action, BT::Blackboard::Ptr blackboard)
    {
        // Generate a unique node name using the prefix and object_id
        std::string node_name = prefix + "_" + action.object_id + "_" + objectActionTypeToString(action.type);

        // Start constructing the XML node
        std::ostringstream xml;
        xml << "<" << objectActionTypeToString(action.type) << " ";
        xml << "name=\"" << node_name << "\" ";

        // Common attribute: object_id
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
            // Rotation attributes
            xml << "first_rotation_axis=\"" << action.first_rotation_axis << "\" ";
            xml << "first_rotation_rad=\"" << action.first_rotation_rad << "\" ";
            xml << "second_rotation_axis=\"" << action.second_rotation_axis << "\" ";
            xml << "second_rotation_rad=\"" << action.second_rotation_rad << "\" ";
            break;
        }
        default:
            throw std::invalid_argument("Unsupported ObjectActionType in buildObjectActionXML");
        }

        // Close the XML node
        xml << "/>";

        return xml.str();
    }

    // std::string buildAddObjectActionXML(const std::string &prefix,
    //                                     const std::string &object_id,
    //                                     const std::string &shape,
    //                                     const std::vector<double> &dimensions,
    //                                     const geometry_msgs::msg::Pose &pose,
    //                                     const std::string &mesh_file,
    //                                     double scale_x,
    //                                     double scale_y,
    //                                     double scale_z)
    // {
    //     std::string node_name = prefix + "_AddObject";
    //     std::string xml = "<AddCollisionObjectAction name=\"" + node_name + "\">\n";
    //     xml += "    <InputPort name=\"object_id\" type=\"std::string\" value=\"" + object_id + "\"/>\n";
    //     xml += "    <InputPort name=\"shape\" type=\"std::string\" value=\"" + shape + "\"/>\n";

    //     if (shape != "mesh")
    //     {
    //         xml += "    <InputPort name=\"dimensions\" type=\"std::vector<double>\" value=\"[";
    //         for (size_t i = 0; i < dimensions.size(); ++i)
    //         {
    //             xml += std::to_string(dimensions[i]);
    //             if (i != dimensions.size() - 1)
    //                 xml += ", ";
    //         }
    //         xml += "]\"/>\n";
    //     }

    //     // Serialize Pose
    //     std::string pose_str = "[" + std::to_string(pose.position.x) + ", " + std::to_string(pose.position.y) + ", " + std::to_string(pose.position.z) + ", " + std::to_string(pose.orientation.x) + ", " + std::to_string(pose.orientation.y) + ", " + std::to_string(pose.orientation.z) + ", " + std::to_string(pose.orientation.w) + "]";
    //     xml += "    <InputPort name=\"pose\" type=\"geometry_msgs::msg::Pose\" value=\"" + pose_str + "\"/>\n";

    //     if (shape == "mesh")
    //     {
    //         xml += "    <InputPort name=\"mesh_file\" type=\"std::string\" value=\"" + mesh_file + "\"/>\n";
    //         xml += "    <InputPort name=\"scale_mesh_x\" type=\"double\" value=\"" + std::to_string(scale_x) + "\"/>\n";
    //         xml += "    <InputPort name=\"scale_mesh_y\" type=\"double\" value=\"" + std::to_string(scale_y) + "\"/>\n";
    //         xml += "    <InputPort name=\"scale_mesh_z\" type=\"double\" value=\"" + std::to_string(scale_z) + "\"/>\n";
    //     }

    //     // Define output ports if needed (e.g., for logging or confirmation)
    //     xml += "</AddCollisionObjectAction>\n";
    //     return xml;
    // }

    // std::string buildRemoveObjectActionXML(const std::string &prefix,
    //                                        const std::string &object_id)
    // {
    //     std::string node_name = prefix + "_RemoveObject";
    //     std::string xml = "<RemoveCollisionObjectAction name=\"" + node_name + "\">\n";
    //     xml += "    <InputPort name=\"object_id\" type=\"std::string\" value=\"" + object_id + "\"/>\n";
    //     xml += "</RemoveCollisionObjectAction>\n";
    //     return xml;
    // }

    // std::string buildAttachDetachObjectActionXML(const std::string &prefix,
    //                                              const std::string &object_id,
    //                                              const std::string &link_name,
    //                                              bool attach)
    // {
    //     std::string node_type = attach ? "AttachDetachObjectAction" : "AttachDetachObjectAction"; // Same node type, different parameters
    //     std::string action = attach ? "attach" : "detach";
    //     std::string node_name = prefix + "_" + (attach ? "Attach" : "Detach") + "Object";
    //     std::string xml = "<AttachDetachObjectAction name=\"" + node_name + "\">\n";
    //     xml += "    <InputPort name=\"object_id\" type=\"std::string\" value=\"" + object_id + "\"/>\n";
    //     xml += "    <InputPort name=\"link_name\" type=\"std::string\" value=\"" + link_name + "\"/>\n";
    //     xml += "    <InputPort name=\"attach\" type=\"bool\" value=\"" + std::string(attach ? "true" : "false") + "\"/>\n";
    //     xml += "</AttachDetachObjectAction>\n";
    //     return xml;
    // }

    // std::string buildCheckObjectExistsActionXML(const std::string &prefix,
    //                                             const std::string &object_id)
    // {
    //     std::string node_name = prefix + "_CheckObjectExists";
    //     std::string xml = "<CheckObjectExistsAction name=\"" + node_name + "\">\n";
    //     xml += "    <InputPort name=\"object_id\" type=\"std::string\" value=\"" + object_id + "\"/>\n";
    //     xml += "    <OutputPort name=\"exists\" type=\"bool\"/>\n";
    //     xml += "    <OutputPort name=\"is_attached\" type=\"bool\"/>\n";
    //     xml += "    <OutputPort name=\"link_name\" type=\"std::string\"/>\n";
    //     xml += "</CheckObjectExistsAction>\n";
    //     return xml;
    // }

    // std::string buildGetObjectPoseActionXML(const std::string &prefix,
    //                                         const std::string &object_id,
    //                                         const std::string &first_rotation_axis,
    //                                         double first_rotation_rad,
    //                                         const std::string &second_rotation_axis,
    //                                         double second_rotation_rad)
    // {
    //     std::string node_name = prefix + "_GetObjectPose";
    //     std::string xml = "<GetObjectPoseAction name=\"" + node_name + "\">\n";
    //     xml += "    <InputPort name=\"object_id\" type=\"std::string\" value=\"" + object_id + "\"/>\n";
    //     xml += "    <InputPort name=\"first_rotation_axis\" type=\"std::string\" value=\"" + first_rotation_axis + "\"/>\n";
    //     xml += "    <InputPort name=\"first_rotation_rad\" type=\"double\" value=\"" + std::to_string(first_rotation_rad) + "\"/>\n";
    //     xml += "    <InputPort name=\"second_rotation_axis\" type=\"std::string\" value=\"" + second_rotation_axis + "\"/>\n";
    //     xml += "    <InputPort name=\"second_rotation_rad\" type=\"double\" value=\"" + std::to_string(second_rotation_rad) + "\"/>\n";
    //     xml += "    <OutputPort name=\"modified_pose\" type=\"geometry_msgs::msg::Pose\"/>\n";
    //     xml += "</GetObjectPoseAction>\n";
    //     return xml;
    // }

} // namespace manymove_cpp_trees
