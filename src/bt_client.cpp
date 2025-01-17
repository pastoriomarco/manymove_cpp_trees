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

using geometry_msgs::msg::Pose;
using namespace manymove_cpp_trees;

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

    /*
     * defineMovementConfigs() creates the default configs to use to plan for moves as
     * defined in the helper function definition.
     * Here we created some default configs like "max_move", "mid_move" and "slow_move",
     * each with its own limits in scaling factors, max cartesian speed, step size and
     * so on, and we use it on any kind of move. We may also create specific configurations
     * for certain moves, for example with a finer step size on a short cartesian move, or
     * a lower plan number target for faster planning times in moves that are not time sensitive
     * during execution, but require planning times as short as possible.
     * Please note that composing the moves in sequences with parallel planning and execute usually
     * proves to be much more effective in reducing overall completion time than reducing planning
     * time, as usually the planning of a single move is much faster than its execution, unless
     * we need to plan for barely reachable poses or complex collision avoidance scenarios.
     */
    auto move_configs = defineMovementConfigs();

    // We define the joint targets we need for the joint moves as vectors of doubles.
    // Be careful that the number of values must match the number of DOF of the robot (here, 6 DOF)
    std::vector<double> joint_rest = {0.0, -0.785, 0.785, 0.0, 1.57, 0.0};
    std::vector<double> joint_look_sx = {-0.175, -0.419, 1.378, 0.349, 1.535, -0.977};
    std::vector<double> joint_look_dx = {0.733, -0.297, 1.378, -0.576, 1.692, 1.291};
    std::string named_home = "home";

    // Original test poses: they should be overwritten by the blackboard key that will be dynamically updated getting the grasp pose object
    Pose pick_target = createPose(0.2, -0.1, 0.15, 1.0, 0.0, 0.0, 0.0);
    Pose approach_target = pick_target;
    approach_target.position.z += 0.02;

    // Populate the blackboard with the poses, one unique key for each pose we want to use.
    // Be careful not to use names that may conflict with the keys automatically created for the moves. (Usually move_{move_id})
    blackboard->set("pick_target", pick_target);
    blackboard->set("approach_target", approach_target);
    RCLCPP_INFO(node->get_logger(), "Blackboard: set('pick_target', pick_target Pose)");
    RCLCPP_INFO(node->get_logger(), "Blackboard: set('approach_target', approach_target Pose)");

    /*
     * Here we compose the sequences of moves. Each of the following sequences represent a logic
     * sequence of moves that are somehow correlated, and not interrupted by operations on I/Os,
     * objects and so on. For example the pick_sequence is a short sequence of moves composed by
     * a "pose" move to get in a position to be ready to approach the object, and the "cartesian"
     * move to get the gripper to the grasp position moving linearly to minimize chances of collisions.
     * As we'll se later, we can then compose these sequences of moves together to build bigger blocks
     * of logically corralated moves.
     */
    std::vector<Move> rest_position = {
        {"joint", "", joint_rest, "", move_configs["max_move"]},
    };

    std::vector<Move> scan_surroundings = {
        {"joint", "", joint_look_sx, "", move_configs["max_move"]},
        {"joint", "", joint_look_dx, "", move_configs["max_move"]},
    };

    // Sequences for Pick/Homing
    std::vector<Move> pick_sequence = {
        {"pose", "approach_target", {}, "", move_configs["mid_move"]},
        {"cartesian", "pick_target", {}, "", move_configs["slow_move"]},
    };
    std::vector<Move> home_position = {
        {"cartesian", "approach_target", {}, "", move_configs["max_move"]},
        {"named", "", {}, named_home, move_configs["max_move"]},
    };

    /*
     * Build parallel move sequence blocks
     * The buildParallelPlanExecuteXML creates the xml tree branch that parallelizes completely the planning and
     * the execution of the sequence of moves. The moves will be planned in sequence until the last move is successfully
     * planned, and the trajectories will be stored in the blackboard and set as valid. The execution starts in parallel,
     * with the first move polling the blackboard until its trajectory is flagged as valid, then the execution begins.
     * This allows for the best performance in related move sequences since the robot needs to wait for just the first
     * trajectory to be available before starting to move.
     * Each move execution resets the validity of the respective trajectory in the blackboard but, to avoid the risk of
     * stale trajectories in scenarios where a branch could be restarted or repeated, you can set the flag reset_trajs to
     * true to add a leaf node that resets all the trajectories of that move sequence in the blackboard.
     */
    std::string to_rest = buildParallelPlanExecuteXML(
        "toRest", rest_position, blackboard, true);

    std::string scan_around = buildParallelPlanExecuteXML(
        "scanAround", scan_surroundings, blackboard, true);

    std::string pick_object = buildParallelPlanExecuteXML(
        "pick", pick_sequence, blackboard, true);

    std::string to_home = buildParallelPlanExecuteXML(
        "home", home_position, blackboard, true);

    /*
     * Combine the parallel move sequence blocks in logic sequences for the entire "preparatory" and "pick" logic.
     * Each subsequence will plan and execute in parallel, but the next subsequence will start planning all its moves
     * only after the last move of the previous sequence executes.
     * This can be useful for example if you have a camera mounted on the robot's arm and you want the next moves to be
     * planned only after the scene have been scanned. For example: the first move to_rest will take the robot in an
     * upright position with the camera pointing down to where the robot base is: I want the octomap to update before
     * continuing planning, but I don't need to wait for inputs or do any other action before planning.
     * Once the scan_around sequence is executed I will want to check for some inputs before continuing, so I terminate the
     * move sequence here: this will let me wrap this serie of sequences with other leaf nodes.
     * Here I create the std::vector<std::string> and name them before creating the xml with the sequenceWrapperXML, to give
     * a cue of what the sequence is about conceptually, but I could also insert the vector in the sequenceWrapperXML directly
     * like this:
     * std::string prep_sequence_xml = sequenceWrapperXML("ComposedPrepSequence", {to_rest, scan_around});
     */
    std::vector<std::string> prep_parallels = {to_rest, scan_around};
    std::vector<std::string> pick_parallels = {pick_object, to_home};

    // Translate it to xml tree leaf or branch
    std::string prep_sequence_xml = sequenceWrapperXML("ComposedPrepSequence", prep_parallels);
    std::string pick_sequence_xml = sequenceWrapperXML("ComposedPickSequence", pick_parallels);

    // Combine prep_sequence_xml and pick_sequence_xml in a <Repeat> node single <Sequence>
    //    => RepeatForever with two children
    std::vector<std::string> composite_move_sequences = {prep_sequence_xml, pick_sequence_xml};
    std::string repeat_wrapper_xml = repeatWrapperXML("RepeatForever", composite_move_sequences, 1); // num_cycles=-1 for infinite

    // ----------------------------------------------------------------------------
    // 3) Build blocks for objects handling
    // ----------------------------------------------------------------------------
    std::vector<double> ground_dimension = {0.8, 0.8, 0.1};
    auto ground_pose = poseBuilderRPY(0.0, 0.0, -0.05, 0.0, 0.0, 0.0);

    std::vector<double> wall_dimension = {0.8, 0.02, 0.8};
    auto wall_pose = poseBuilderRPY(0.0, 0.4, 0.3, 0.0, 0.0, 0.0);

    std::vector<double> cylinderdimension = {0.1, 0.005};
    auto cylinderpose = poseBuilderRPY(0.1, 0.2, 0.005, 0.0, 1.57, 0.0);

    std::string mesh_file = "package://manymove_object_manager/meshes/unit_tube.stl";
    std::vector<double> mesh_scale = {0.01, 0.01, 0.1};
    auto mesh_pose = poseBuilderRPY(0.1, -0.2, 0.005, 0.0, 1.57, 0.0);

    // Create object actions
    ObjectAction add_ground = createAddPrimitiveObject("obstacle_ground", "box", ground_dimension, ground_pose);
    ObjectAction add_wall = createAddPrimitiveObject("obstacle_wall", "box", wall_dimension, wall_pose);
    ObjectAction add_cylinder = createAddPrimitiveObject("graspable_cylinder", "cylinder", cylinderdimension, cylinderpose);
    ObjectAction add_mesh = createAddMeshObject("graspable_mesh", mesh_pose, mesh_file, mesh_scale[0], mesh_scale[1], mesh_scale[2]);

    // Translate objects to xml tree leaf or branch
    std::string add_ground_xml = buildObjectActionXML("add_ground", add_ground);
    std::string add_wall_xml = buildObjectActionXML("add_wall", add_wall);
    std::string add_cylinder_xml = buildObjectActionXML("add_cylinder", add_cylinder);
    std::string add_mesh_xml = buildObjectActionXML("add_mesh", add_mesh);

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
    ObjectAction get_pick_pose_action = createGetObjectPose(
        object_id_for_pose,
        pick_pose_key,
        pick_transform_xyz_rpy,
        reference_orientation_rpy);

    // Create the GetObjectPoseAction
    ObjectAction get_approach_pose_action = createGetObjectPose(
        object_id_for_pose,
        approach_pose_key,
        approach_transform_xyz_rpy,
        reference_orientation_rpy);

    // Translate get_pose_action to xml tree leaf
    std::string get_pick_pose_xml = buildObjectActionXML("get_pick_pose", get_pick_pose_action);
    std::string get_approach_pose_xml = buildObjectActionXML("get_approach_pose", get_approach_pose_action);

    // Add get_pose_xml to the object_then_moves vector
    // For example, after adding all objects and before the repeat_wrapper_xml
    std::vector<std::string> objects_then_moves = {add_ground_xml,
                                                   add_wall_xml,
                                                   add_cylinder_xml,
                                                   add_mesh_xml,
                                                   get_pick_pose_xml,
                                                   get_approach_pose_xml,
                                                   repeat_wrapper_xml};

    // ----------------------------------------------------------------------------
    // 5) Combine the objects and moves in a sequences:
    // ----------------------------------------------------------------------------
    std::string object_then_moves_xml = sequenceWrapperXML("ObjectMovesSequence", objects_then_moves);

    //    => MasterSequence with RepeatForever as child to set BehaviorTree ID and root main_tree_to_execute in the XML
    std::vector<std::string> master_branches = {object_then_moves_xml};
    std::string master_body = sequenceWrapperXML("GlobalMasterSequence", master_branches);

    // ----------------------------------------------------------------------------
    // 6) Wrap everything into a top-level <root> with <BehaviorTree ID="MasterTree">
    // ----------------------------------------------------------------------------
    std::string final_tree_xml = mainTreeWrapperXML("MasterTree", master_body);

    RCLCPP_INFO(node->get_logger(), "=== Programmatically Generated Tree XML ===\n%s", final_tree_xml.c_str());

    // 7) Register node types
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<PlanningAction>("PlanningAction");
    factory.registerNodeType<ExecuteTrajectory>("ExecuteTrajectory");
    factory.registerNodeType<ResetTrajectories>("ResetTrajectories");

    factory.registerNodeType<AddCollisionObjectAction>("AddCollisionObjectAction");
    factory.registerNodeType<RemoveCollisionObjectAction>("RemoveCollisionObjectAction");
    factory.registerNodeType<AttachDetachObjectAction>("AttachDetachObjectAction");
    factory.registerNodeType<CheckObjectExistsAction>("CheckObjectExistsAction");
    factory.registerNodeType<GetObjectPoseAction>("GetObjectPoseAction");

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
