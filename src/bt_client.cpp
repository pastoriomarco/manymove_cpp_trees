#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <geometry_msgs/msg/pose.hpp>
#include "manymove_cpp_trees/planning_action.hpp"
#include "manymove_cpp_trees/execute_trajectory.hpp"
#include "manymove_cpp_trees/behavior_tree_xml_generator.hpp"
#include "manymove_cpp_trees/move.hpp"
#include "manymove_cpp_trees/serialization_helper.hpp"
#include "manymove_planner/msg/movement_config.hpp"
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <fstream>

using geometry_msgs::msg::Pose;
using manymove_cpp_trees::Move;
using manymove_planner::msg::MovementConfig;

// Helper to create movement configurations
std::unordered_map<std::string, MovementConfig> defineMovementConfigs()
{
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

// Helper to create a Pose
Pose createPose(double x, double y, double z,
                double qx, double qy, double qz, double qw)
{
    Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
    return pose;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_client_node");
    RCLCPP_INFO(node->get_logger(), "BT Client Node started");

    // 1. Define Movement Configurations
    auto move_configs = defineMovementConfigs();

    // 2. Define Move Sequences

    // Define specific joint configurations
    std::vector<double> joint_rest = {0.0, -0.785, 0.785, 0.0, 1.57, 0.0};
    std::vector<double> joint_look_sx = {-0.175, -0.419, 1.378, 0.349, 1.535, -0.977};
    std::vector<double> joint_look_dx = {0.733, -0.297, 1.378, -0.576, 1.692, 1.291};

    std::string named_home = "home";

    // Define Poses
    Pose pick_target = createPose(0.2, -0.1, 0.15, 1.0, 0.0, 0.0, 0.0);
    Pose approach_target = pick_target;
    approach_target.position.z += 0.02; // Adjust z position for approach

    // Define move sequences
    std::vector<Move> rest_position = {
        {"joint", joint_rest, Pose(), "", move_configs["max_move"]},
    };

    std::vector<Move> scan_surroundings = {
        {"joint", joint_look_sx, Pose(), "", move_configs["max_move"]},
        {"joint", joint_look_dx, Pose(), "", move_configs["max_move"]},
    };

    std::vector<Move> pick_sequence = {
        {"pose", {}, approach_target, "", move_configs["mid_move"]},
        {"pose", {}, pick_target, "", move_configs["slow_move"]},
        {"pose", {}, approach_target, "", move_configs["max_move"]},
    };

    std::vector<Move> home_position = {
        {"named", {}, Pose(), named_home, move_configs["max_move"]},
    };

    // Aggregate all sequences
    std::vector<std::vector<Move>> list_of_sequences = {rest_position, scan_surroundings, pick_sequence, home_position};

    // 3. Initialize BehaviorTree Factory and Register Custom Nodes
    BT::BehaviorTreeFactory factory;

    // Register custom action nodes
    factory.registerNodeType<manymove_cpp_trees::PlanningAction>("PlanningAction");
    factory.registerNodeType<manymove_cpp_trees::ExecuteTrajectory>("ExecuteTrajectory");

    // 4. Generate XML and Create the Behavior Tree

    // Instantiate the XML generator
    manymove_cpp_trees::BehaviorTreeXMLGenerator xml_generator(list_of_sequences);

    // Generate the XML string
    std::string tree_xml = xml_generator.generateXML();

    RCLCPP_INFO(node->get_logger(), "Generated Behavior Tree XML:\n%s", tree_xml.c_str());

    // Save the XML to a file for visualization
    std::ofstream tree_file("bt_tree.xml");
    if (tree_file.is_open())
    {
        tree_file << tree_xml;
        tree_file.close();
        RCLCPP_INFO(node->get_logger(), "Behavior Tree XML saved to bt_tree.xml");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to save Behavior Tree XML to file.");
    }

    // 5. Set the ROS node on the blackboard for action nodes to access
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);

    // 6. Create the tree from the XML string and the blackboard
    BT::Tree tree = factory.createTreeFromText(tree_xml, blackboard);

    // 7. Attach a ZMQ Publisher for Groot
    BT::PublisherZMQ publisher_zmq(tree);

    // 8. Tick the Behavior Tree

    rclcpp::Rate rate(1000); // 1000 Hz

    while (rclcpp::ok())
    {
        tree.tickRoot();

        BT::NodeStatus status = tree.rootNode()->status();

        if (status == BT::NodeStatus::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "Behavior Tree completed successfully.");
            break;
        }
        else if (status == BT::NodeStatus::FAILURE)
        {
            RCLCPP_ERROR(node->get_logger(), "Behavior Tree failed.");
            break;
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    // 9. Shutdown
    tree.rootNode()->halt();
    rclcpp::shutdown();
    return 0;
}
