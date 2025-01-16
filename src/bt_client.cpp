// bt_client.cpp

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/decorators/repeat_node.h>

#include "manymove_cpp_trees/action_nodes.hpp"
#include "manymove_cpp_trees/move.hpp"
#include "manymove_cpp_trees/object.hpp"
#include "manymove_cpp_trees/tree_helper.hpp"
#include "manymove_cpp_trees/bt_converters.hpp"

#include <string>
#include <vector>
#include <unordered_map>

using geometry_msgs::msg::Pose;
using manymove_cpp_trees::Move;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("bt_client_node");
    RCLCPP_INFO(node->get_logger(), "BT Client Node started (Purely Programmatic XML).");

    // ----------------------------------------------------------------------------
    // 1) Create a blackboard and set "node"
    // ----------------------------------------------------------------------------
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    RCLCPP_INFO(node->get_logger(), "Blackboard: set('node', <rclcpp::Node>)");

    // ----------------------------------------------------------------------------
    // 2) Setup moves
    // ----------------------------------------------------------------------------
    auto move_configs = manymove_cpp_trees::defineMovementConfigs();

    std::vector<double> joint_rest = {0.0, -0.785, 0.785, 0.0, 1.57, 0.0};
    std::vector<double> joint_look_sx = {-0.175, -0.419, 1.378, 0.349, 1.535, -0.977};
    std::vector<double> joint_look_dx = {0.733, -0.297, 1.378, -0.576, 1.692, 1.291};
    std::string named_home = "home";

    Pose pick_target = manymove_cpp_trees::createPose(0.2, -0.1, 0.15, 1.0, 0.0, 0.0, 0.0);
    Pose approach_target = pick_target;
    approach_target.position.z += 0.02;

    // Populate the blackboard with the poses
    blackboard->set("pick_target", pick_target);
    blackboard->set("approach_target", approach_target);
    RCLCPP_INFO(node->get_logger(), "Blackboard: set('pick_target', pick_target Pose)");
    RCLCPP_INFO(node->get_logger(), "Blackboard: set('approach_target', approach_target Pose)");

    // Sequences for Prep
    std::vector<Move> rest_position = {
        {"joint", "", joint_rest, "", move_configs["max_move"]}};

    std::vector<Move> scan_surroundings = {
        {"joint", "", joint_look_sx, "", move_configs["max_move"]},
        {"joint", "", joint_look_dx, "", move_configs["max_move"]},
    };

    std::vector<std::vector<Move>> preparatory_sequences = {
        rest_position,
        scan_surroundings};

    // Sequences for Pick/Homing
    std::vector<Move> pick_sequence = {
        {"pose", "approach_target", {}, "", move_configs["mid_move"]},
        {"cartesian", "pick_target", {}, "", move_configs["slow_move"]},
    };
    std::vector<Move> home_position = {
        {"cartesian", "approach_target", {}, "", move_configs["max_move"]},
        {"named", "", {}, named_home, move_configs["max_move"]},
    };

    // Build parallel move sequence blocks
    std::string par0 = manymove_cpp_trees::buildParallelPlanExecuteXML(
        "toRest", rest_position, blackboard, true);

    std::string par1 = manymove_cpp_trees::buildParallelPlanExecuteXML(
        "scanAround", scan_surroundings, blackboard, true);

    std::string par2 = manymove_cpp_trees::buildParallelPlanExecuteXML(
        "pick", pick_sequence, blackboard, true);

    std::string par3 = manymove_cpp_trees::buildParallelPlanExecuteXML(
        "home", home_position, blackboard, true);

    // Combine them in logic sequences for the entire "preparatory" logic
    // Each subsequence will plan and execute in parallel, but
    // each subsequent move sequences will start planning all its moves after the last move of the previous sequence executes
    std::vector<std::string> prep_parallels = {par0};
    std::vector<std::string> pick_parallels = {par2, par3};

    // Translate it to xml tree leaf or branch
    std::string prep_sequence_xml = manymove_cpp_trees::sequenceWrapperXML("ComposedPrepSequence", prep_parallels);
    std::string pick_sequence_xml = manymove_cpp_trees::sequenceWrapperXML("ComposedPickSequence", pick_parallels);

    // Combine prep_sequence_xml and pick_sequence_xml in a <Repeat> node single <Sequence>
    //    => RepeatForever with two children
    std::vector<std::string> composite_move_sequences = {prep_sequence_xml, pick_sequence_xml};
    std::string repeat_wrapper_xml = manymove_cpp_trees::repeatWrapperXML("RepeatForever", composite_move_sequences, 1); // num_cycles=-1 for infinite

    // ----------------------------------------------------------------------------
    // 3) Build blocks for objects handling
    // ----------------------------------------------------------------------------
    std::vector<double> ground_dimension = {0.8, 0.8, 0.1};
    auto ground_pose = manymove_cpp_trees::poseBuilderRPY(0.0, 0.0, -0.05, 0.0, 0.0, 0.0);

    std::vector<double> wall_dimension = {0.8, 0.02, 0.8};
    auto wall_pose = manymove_cpp_trees::poseBuilderRPY(0.0, 0.4, 0.3, 0.0, 0.0, 0.0);

    std::vector<double> cylinderdimension = {0.1, 0.005};
    auto cylinderpose = manymove_cpp_trees::poseBuilderRPY(0.1, 0.2, 0.005, 0.0, 1.57, 0.0);

    std::string mesh_file = "package://manymove_object_manager/meshes/unit_tube.stl";
    std::vector<double> mesh_scale = {0.01, 0.01, 0.1};
    auto mesh_pose = manymove_cpp_trees::poseBuilderRPY(0.1, -0.2, 0.005, 0.0, 1.57, 0.0);

    // Create object actions
    manymove_cpp_trees::ObjectAction add_ground = manymove_cpp_trees::createAddPrimitiveObject("obstacle_ground", "box", ground_dimension, ground_pose);
    manymove_cpp_trees::ObjectAction add_wall = manymove_cpp_trees::createAddPrimitiveObject("obstacle_wall", "box", wall_dimension, wall_pose);
    manymove_cpp_trees::ObjectAction add_cylinder = manymove_cpp_trees::createAddPrimitiveObject("graspable_cylinder", "cylinder", cylinderdimension, cylinderpose);
    manymove_cpp_trees::ObjectAction add_mesh = manymove_cpp_trees::createAddMeshObject("graspable_mesh", mesh_pose, mesh_file, mesh_scale[0], mesh_scale[1], mesh_scale[2]);

    // Translate objects to xml tree leaf or branch
    std::string add_ground_xml = manymove_cpp_trees::buildObjectActionXML("add_ground", add_ground, blackboard);
    std::string add_wall_xml = manymove_cpp_trees::buildObjectActionXML("add_wall", add_wall, blackboard);
    std::string add_cylinder_xml = manymove_cpp_trees::buildObjectActionXML("add_cylinder", add_cylinder, blackboard);
    std::string add_mesh_xml = manymove_cpp_trees::buildObjectActionXML("add_mesh", add_mesh, blackboard);

    // ----------------------------------------------------------------------------
    // 4) Add GetObjectPoseAction Node
    // ----------------------------------------------------------------------------
    // Define the object ID and pose_key where the pose will be stored
    std::string object_id_for_pose = "graspable_mesh"; // Example object ID
    std::string pick_pose_key = "pick_target";
    std::string approach_pose_key = "approach_target";

    // Define the transformation and reference orientation
    std::vector<double> pick_transform_xyz_rpy = {0.0, 0.0, 0.002, 3.14, 1.57, 0.0};
    std::vector<double> approach_transform_xyz_rpy = {0.0, 0.0, 0.02, 3.14, 1.57, 0.0};
    std::vector<double> reference_orientation_rpy = {0.0, 0.0, 0.0};

    // Create the GetObjectPoseAction
    manymove_cpp_trees::ObjectAction get_pick_pose_action = manymove_cpp_trees::createGetObjectPose(
        object_id_for_pose,
        pick_pose_key,
        pick_transform_xyz_rpy,
        reference_orientation_rpy);

    // Create the GetObjectPoseAction
    manymove_cpp_trees::ObjectAction get_approach_pose_action = manymove_cpp_trees::createGetObjectPose(
        object_id_for_pose,
        approach_pose_key,
        approach_transform_xyz_rpy,
        reference_orientation_rpy);

    // Translate get_pose_action to xml tree leaf
    std::string get_pick_pose_xml = manymove_cpp_trees::buildObjectActionXML("get_pick_pose", get_pick_pose_action, blackboard);
    std::string get_approach_pose_xml = manymove_cpp_trees::buildObjectActionXML("get_approach_pose", get_approach_pose_action, blackboard);

    // Add get_pose_xml to the object_then_moves vector
    // For example, after adding all objects and before the repeat_wrapper_xml
    std::vector<std::string> object_then_moves = {add_ground_xml, add_wall_xml, add_cylinder_xml, add_mesh_xml, get_pick_pose_xml, get_approach_pose_xml, repeat_wrapper_xml};

    // ----------------------------------------------------------------------------
    // 5) Combine the objects and moves in a sequences:
    // ----------------------------------------------------------------------------
    std::string object_then_moves_xml = manymove_cpp_trees::sequenceWrapperXML("ObjectMovesSequence", object_then_moves);

    //    => MasterSequence with RepeatForever as child to set BehaviorTree ID and root main_tree_to_execute in the XML
    std::vector<std::string> master_branches = {object_then_moves_xml};
    std::string master_body = manymove_cpp_trees::sequenceWrapperXML("GlobalMasterSequence", master_branches);

    // ----------------------------------------------------------------------------
    // 6) Wrap everything into a top-level <root> with <BehaviorTree ID="MasterTree">
    // ----------------------------------------------------------------------------
    std::string final_tree_xml = manymove_cpp_trees::mainTreeWrapperXML("MasterTree", master_body);

    RCLCPP_INFO(node->get_logger(), "=== Programmatically Generated Tree XML ===\n%s", final_tree_xml.c_str());

    // 7) Register node types
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<manymove_cpp_trees::PlanningAction>("PlanningAction");
    factory.registerNodeType<manymove_cpp_trees::ExecuteTrajectory>("ExecuteTrajectory");
    factory.registerNodeType<manymove_cpp_trees::ResetTrajectories>("ResetTrajectories");

    factory.registerNodeType<manymove_cpp_trees::AddCollisionObjectAction>("AddCollisionObjectAction");
    factory.registerNodeType<manymove_cpp_trees::RemoveCollisionObjectAction>("RemoveCollisionObjectAction");
    factory.registerNodeType<manymove_cpp_trees::AttachDetachObjectAction>("AttachDetachObjectAction");
    factory.registerNodeType<manymove_cpp_trees::CheckObjectExistsAction>("CheckObjectExistsAction");
    factory.registerNodeType<manymove_cpp_trees::GetObjectPoseAction>("GetObjectPoseAction");

    // 8) Create the tree from final_tree_xml
    BT::Tree tree;
    try
    {
        tree = factory.createTreeFromText(final_tree_xml, blackboard);
    }
    catch (const std::exception &ex)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to create tree: %s", ex.what());
        return 1;
    }

    // 9) ZMQ publisher (optional, to visualize in Groot)
    BT::PublisherZMQ publisher(tree);

    // 10) Tick the tree
    rclcpp::Rate rate(1000);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        BT::NodeStatus status = tree.tickRoot();

        if (status == BT::NodeStatus::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "BT ended SUCCESS.");
            break;
        }
        else if (status == BT::NodeStatus::FAILURE)
        {
            RCLCPP_ERROR(node->get_logger(), "BT ended FAILURE.");
            break;
        }
        rate.sleep();
    }

    tree.rootNode()->halt();
    rclcpp::shutdown();
    return 0;
}
