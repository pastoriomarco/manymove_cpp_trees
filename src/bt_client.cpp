#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "manymove_cpp_trees/action_nodes.hpp"
#include "manymove_cpp_trees/move.hpp"
#include "manymove_cpp_trees/tree_helper.hpp"

// std, etc.
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

    // 1) Create a blackboard and set "node"
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    RCLCPP_INFO(node->get_logger(), "Blackboard: set('node', <rclcpp::Node>)");

    // 2) Setup moves
    auto move_configs = manymove_cpp_trees::defineMovementConfigs();

    std::vector<double> joint_rest = {0.0, -0.785, 0.785, 0.0, 1.57, 0.0};
    std::vector<double> joint_look_sx = {-0.175, -0.419, 1.378, 0.349, 1.535, -0.977};
    std::vector<double> joint_look_dx = {0.733, -0.297, 1.378, -0.576, 1.692, 1.291};
    std::string named_home = "home";

    Pose pick_target = manymove_cpp_trees::createPose(0.2, -0.1, 0.15, 1.0, 0.0, 0.0, 0.0);
    Pose approach_target = pick_target;
    approach_target.position.z += 0.02;

    // Sequences for Prep
    std::vector<Move> rest_position = {
        {"joint", joint_rest, Pose(), "", move_configs["max_move"]}};

    std::vector<Move> scan_surroundings = {
        {"joint", joint_look_sx, Pose(), "", move_configs["max_move"]},
        {"joint", joint_look_dx, Pose(), "", move_configs["max_move"]}};

    std::vector<std::vector<Move>> preparatory_sequences = {
        rest_position,
        scan_surroundings};

    // Sequences for Pick/Homing
    std::vector<Move> pick_sequence = {
        {"pose", {}, approach_target, "", move_configs["mid_move"]},
        {"cartesian", {}, pick_target, "", move_configs["slow_move"]},
        {"cartesian", {}, approach_target, "", move_configs["max_move"]}};
    std::vector<Move> home_position = {
        {"named", {}, Pose(), named_home, move_configs["max_move"]},
    };

    // Initialize a move_id counter to ensure unique IDs
    size_t move_id = 0;

    // 3) Build parallel blocks without local_index
    std::string par0 = manymove_cpp_trees::buildParallelPlanExecuteXML(
        "toRest", rest_position);

    std::string par1 = manymove_cpp_trees::buildParallelPlanExecuteXML(
        "scanAround", scan_surroundings);

    // Combine them in a single <Sequence> for the entire "preparatory" logic
    std::vector<std::string> prep_parallels = {par0, par1};
    std::string prep_sequence_xml = manymove_cpp_trees::sequenceWrapperXML("ComposedPrepSequence", prep_parallels);

    // 4) Build parallel blocks for "pickAndHoming_seq"
    std::string par2 = manymove_cpp_trees::buildParallelPlanExecuteXML(
        "pick", pick_sequence);

    std::string par3 = manymove_cpp_trees::buildParallelPlanExecuteXML(
        "home", home_position);

    std::vector<std::string> pick_parallels = {par2, par3};
    std::string pick_sequence_xml = manymove_cpp_trees::sequenceWrapperXML("ComposedPickSequence", pick_parallels);

    // 5) Combine prep_sequence_xml and pick_sequence_xml in a single <Sequence>
    //    => MasterSequence with two children
    std::vector<std::string> master_branches = {prep_sequence_xml, pick_sequence_xml};
    std::string master_body = manymove_cpp_trees::sequenceWrapperXML("GlobalMasterSequence", master_branches);

    // 6) Wrap everything into a top-level <root> with <BehaviorTree ID="MasterTree">
    std::string final_tree_xml = manymove_cpp_trees::mainTreeWrapperXML("MasterTree", master_body);

    RCLCPP_INFO(node->get_logger(), "=== Programmatically Generated Tree XML ===\n%s", final_tree_xml.c_str());

    // 7) Preload blackboard with the Moves
    //    Keys => "move_{move_id}"
    //    Ensures each move has a unique move_id
    {
        // Preparatory Sequence: par0 (rest_position) - 1 move
        for (auto &m : rest_position)
        {
            std::string key = "move_" + std::to_string(move_id);
            blackboard->set(key, std::make_shared<Move>(m));
            RCLCPP_INFO(node->get_logger(), "BB set: %s", key.c_str());
            move_id++;
        }

        // Preparatory Sequence: par1 (scan_surroundings) - 2 moves
        for (auto &m : scan_surroundings)
        {
            std::string key = "move_" + std::to_string(move_id);
            blackboard->set(key, std::make_shared<Move>(m));
            RCLCPP_INFO(node->get_logger(), "BB set: %s", key.c_str());
            move_id++;
        }

        // PickAndHoming Sequence: par2 (pick_sequence) - 3 moves
        for (auto &m : pick_sequence)
        {
            std::string key = "move_" + std::to_string(move_id);
            blackboard->set(key, std::make_shared<Move>(m));
            RCLCPP_INFO(node->get_logger(), "BB set: %s", key.c_str());
            move_id++;
        }

        // PickAndHoming Sequence: par3 (home_position) - 1 move
        for (auto &m : home_position)
        {
            std::string key = "move_" + std::to_string(move_id);
            blackboard->set(key, std::make_shared<Move>(m));
            RCLCPP_INFO(node->get_logger(), "BB set: %s", key.c_str());
            move_id++;
        }
    }

    // 8) Register node types
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<manymove_cpp_trees::PlanningAction>("PlanningAction");
    factory.registerNodeType<manymove_cpp_trees::ExecuteTrajectory>("ExecuteTrajectory");

    // 9) Create the tree from final_tree_xml
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

    // 10) ZMQ publisher (optional)
    BT::PublisherZMQ publisher(tree);

    // 11) Tick the tree
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
