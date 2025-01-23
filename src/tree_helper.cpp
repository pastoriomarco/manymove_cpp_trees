#include "manymove_cpp_trees/tree_helper.hpp"
#include <sstream>
#include <rclcpp/rclcpp.hpp>

// A static global counter to ensure unique move IDs across the entire tree
static int g_global_move_id = 0;

namespace manymove_cpp_trees
{
    // ----------------------------------------------------------------------------
    // Builder functions to build xml tree snippets programmatically
    // ----------------------------------------------------------------------------

    std::string buildParallelPlanExecuteXML(const std::string &node_prefix,
                                            const std::vector<Move> &moves,
                                            BT::Blackboard::Ptr blackboard,
                                            const std::string &robot_prefix,
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
        planning_seq << "    <Sequence name=\"PlanningSequence_" << node_prefix << "_" << blockStartID << "\">\n";

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
                         << " robot_prefix=\"" << robot_prefix << "\""
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
        execution_seq << "    <Sequence name=\"ExecutionSequence_" << node_prefix << "_" << blockStartID << "\">\n";

        for (int mid : move_ids)
        {
            execution_seq << "      <ExecuteTrajectory"
                          << " name=\"ExecMove_" << mid << "\""
                          << " robot_prefix=\"" << robot_prefix << "\""
                          << " planned_move_id=\"{planned_move_id_" << mid << "}\""
                          << " trajectory=\"{trajectory_" << mid << "}\""
                          << " planning_validity=\"{validity_" << mid << "}\""
                          << "/>\n";
        }

        execution_seq << "    </Sequence>\n";

        // Parallel node
        std::ostringstream parallel_node;
        parallel_node << "  <Parallel name=\"ParallelPlanExecute_" << node_prefix << "_" << blockStartID
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

    std::string buildObjectActionXML(const std::string &node_prefix, const ObjectAction &action)
    {
        // Generate a unique node name using the node_prefix and object_id
        std::string node_name = node_prefix + "_" + action.object_id + "_" + objectActionTypeToString(action.type);

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

    std::string buildSetOutputXML(const std::string &node_prefix,
                                  const std::string &io_type,
                                  int ionum,
                                  int value,
                                  const std::string &robot_prefix)
    {
        // Construct a node name
        std::string node_name = node_prefix + "_SetOutput";

        std::ostringstream xml;
        xml << "<SetOutputAction "
            << "name=\"" << node_name << "\" "
            << "io_type=\"" << io_type << "\" "
            << "ionum=\"" << ionum << "\" "
            << "robot_prefix=\"" << robot_prefix << "\" "
            << "value=\"" << ((value == 0) ? "0" : "1") << "\"";

        // If the user wants the success output on blackboard
        xml << " success=\"{" << io_type << "_" << ionum << "_success" << "}\"";

        xml << "/>";
        return xml.str();
    }

    std::string buildGetInputXML(const std::string &node_prefix,
                                 const std::string &io_type,
                                 int ionum,
                                 const std::string &robot_prefix)
    {
        // Construct a node name
        std::string node_name = node_prefix + "_GetInput";

        std::ostringstream xml;
        xml << "<GetInputAction "
            << "name=\"" << node_name << "\" "
            << "io_type=\"" << io_type << "\" "
            << "ionum=\"" << ionum << "\""
            << "robot_prefix=\"" << robot_prefix << "\" ";

        // If user wants the read value on the blackboard
        xml << " value=\"{" << io_type << "_" << ionum << "}\"";

        // If user wants the success output on the blackboard
        xml << " success=\"{" << io_type << "_" << ionum << "_success" << "}\"";

        xml << "/>";
        return xml.str();
    }

    std::string buildCheckInputXML(const std::string &node_prefix,
                                   const std::string &io_type,
                                   int ionum,
                                   int value,
                                   const std::string &robot_prefix,
                                   bool wait,
                                   int timeout_ms)
    {
        // Construct a node name
        std::string node_name = node_prefix + "_CheckInput";

        // The value can be 0 or 1, so we trim anything different from 0 or 1. If it's not 0, then it is 1.
        int value_to_check = (value == 0 ? 0 : 1);

        // Build GetInputAction
        std::string check_condition_xml = buildGetInputXML(node_name, io_type, ionum, robot_prefix);

        // Build the CheckBlackboardValue node
        std::ostringstream inner_xml;
        inner_xml << "<Condition ID=\"CheckBlackboardValue\""
                  << " key=\"" << io_type << "_" << ionum << "\""
                  << " value=\"" << value_to_check << "\" />";

        // Wrap in a Sequence
        std::ostringstream sequence_xml;
        sequence_xml << sequenceWrapperXML(node_name + "_Sequence", {check_condition_xml, inner_xml.str()});

        if (wait)
        {
            // TODO: hardcoded dalay, evaluate if it should be set by user or not:
            int delay_ms = 200;

            // Check here for details about <Delay> : https://github.com/BehaviorTree/BehaviorTree.CPP/issues/413
            std::ostringstream delay_and_fail_xml;
            delay_and_fail_xml << "<Delay delay_msec=\"" << delay_ms << "\">\n"
                               << "<AlwaysFailure />" << "\n"
                               << "</Delay>" << "\n";

            std::string fallback_check_or_delay_xml = fallbackWrapperXML((node_name + "_Fallback"), {sequence_xml.str(), delay_and_fail_xml.str()});

            std::ostringstream wait_xml;

            // Tree modified after finding this issue:
            // https://github.com/BehaviorTree/BehaviorTree.CPP/issues/395
            // wait_xml << "<RetryUntilSuccessful name=\"" << node_name << "_Retry\" max_attempts=\"-1\">\n"
            //       << fallback_check_or_delay << "\n"
            //       << "</RetryUntilSuccessful>";

            wait_xml << "<Inverter>\n"
                     << "<KeepRunningUntilFailure>\n"
                     << "<Inverter>\n"
                     << fallback_check_or_delay_xml << "\n"
                     << "</Inverter>\n"
                     << "</KeepRunningUntilFailure>\n"
                     << "</Inverter>\n";

            if (timeout_ms > 0)
            {
                std::ostringstream timeout_xml;
                timeout_xml << "<Timeout msec=\"" << timeout_ms << "\">\n"
                            << wait_xml.str()
                            << "</Timeout>";

                return sequenceWrapperXML(node_name + "_WaitTimeout", {timeout_xml.str()});
            }

            return sequenceWrapperXML(node_name + "_WaitTimeout", {wait_xml.str()});
        }

        // If wait was set to false, return the sequence without further additions
        return sequence_xml.str();
    }

    std::string buildCheckRobotStateXML(const std::string &node_prefix,
                                        const std::string &robot_prefix,
                                        const std::string &ready_key,
                                        const std::string &err_key,
                                        const std::string &mode_key,
                                        const std::string &state_key,
                                        const std::string &message_key)
    {
        // Construct a node name
        std::string node_name = node_prefix + "_CheckRobotState";

        std::ostringstream xml;
        xml << "<CheckRobotStateAction "
            << "name=\"" << node_name << "\""
            << "robot_prefix=\"" << robot_prefix << "\" ";

        // Optional outputs
        if (!ready_key.empty())
        {
            xml << " ready=\"{" << ready_key << "}\"";
        }
        if (!err_key.empty())
        {
            xml << " err=\"{" << err_key << "}\"";
        }
        if (!mode_key.empty())
        {
            xml << " mode=\"{" << mode_key << "}\"";
        }
        if (!state_key.empty())
        {
            xml << " state=\"{" << state_key << "}\"";
        }
        if (!message_key.empty())
        {
            xml << " message=\"{" << message_key << "}\"";
        }

        xml << "/>";
        return xml.str();
    }

    std::string buildResetRobotStateXML(const std::string &node_prefix,
                                        const std::string &robot_prefix)
    {
        // Construct a node name
        std::string node_name = node_prefix + "_ResetRobotState";

        std::ostringstream xml;
        xml << "<ResetRobotStateAction "
            << "name=\"" << node_name << "\""
            << "robot_prefix=\"" << robot_prefix << "\" ";

        // Output
        xml << " success=\"{" << "robot_state_success" << "}\"";

        xml << "/>";

        return sequenceWrapperXML(
            node_name + "_WaitTimeout",
            {xml.str(), buildStopMotionXML(node_prefix, robot_prefix, 0.25)});

        return xml.str();
    }

    std::string buildStopMotionXML(const std::string &node_prefix,
                                   const std::string &robot_prefix,
                                   double deceleration_time)
    {
        // Construct a node name
        std::string node_name = node_prefix + "_StopMotion";

        std::ostringstream xml;
        xml << "<StopMotionAction "
            << "name=\"" << node_name << "\" "
            << "robot_prefix=\"" << robot_prefix << "\" "
            << "deceleration_time=\"" << deceleration_time << "\" ";

        // Output
        // xml << " success=\"{" << "stop_motion_success" << "}\"";

        xml << "/>";
        return xml.str();
    }

    // ----------------------------------------------------------------------------
    // Wrappers
    // ----------------------------------------------------------------------------

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
