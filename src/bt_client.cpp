#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "manymove_cpp_trees/action_nodes_objects.hpp"
#include "manymove_cpp_trees/action_nodes_planner.hpp"
#include "manymove_cpp_trees/action_nodes_signals.hpp"
#include "manymove_cpp_trees/move.hpp"
#include "manymove_cpp_trees/object.hpp"
#include "manymove_cpp_trees/tree_helper.hpp"
#include "manymove_cpp_trees/bt_converters.hpp"

#include "manymove_signals/action/set_output.hpp"
#include "manymove_signals/action/get_input.hpp"
#include "manymove_signals/action/check_robot_state.hpp"
#include "manymove_signals/action/reset_robot_state.hpp"

#include <string>
#include <vector>

using geometry_msgs::msg::Pose;
using namespace manymove_cpp_trees;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("bt_client_node");
    RCLCPP_INFO(node->get_logger(), "BT Client Node started (Purely Programmatic XML).");

    // This parameter is to be set true if we are connected to a real robot that exposes the necessary services for manymove_signals
    bool is_robot_real;
    node->declare_parameter<bool>("is_robot_real", false);
    node->get_parameter_or<bool>("is_robot_real", is_robot_real, false);

    // This parameter indicates the prefix to apply to the robot's action servers
    std::string robot_prefix;
    node->declare_parameter<std::string>("robot_prefix", "");
    node->get_parameter_or<std::string>("robot_prefix", robot_prefix, "");

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

    // Original pick test poses: they should be overwritten by the blackboard key that will be dynamically updated getting the grasp pose object
    Pose pick_target = createPose(0.2, -0.1, 0.15, 1.0, 0.0, 0.0, 0.0);
    Pose approach_pick_target = pick_target;
    approach_pick_target.position.z += 0.02;

    // Test poses to place the object, these are not overwritten later (for now)
    Pose drop_target = createPose(0.2, 0.0, 0.15, 1.0, 0.0, 0.0, 0.0);
    Pose approach_drop_target = drop_target;
    approach_drop_target.position.z += 0.02;

    // Populate the blackboard with the poses, one unique key for each pose we want to use.
    // Be careful not to use names that may conflict with the keys automatically created for the moves. (Usually move_{move_id})
    blackboard->set("pick_target", pick_target);
    blackboard->set("approach_pick_target", approach_pick_target);
    RCLCPP_INFO(node->get_logger(), "Blackboard: set('pick_target', pick_target Pose)");
    RCLCPP_INFO(node->get_logger(), "Blackboard: set('approach_pick_target', approach_pick_target Pose)");

    blackboard->set("drop_target", drop_target);
    blackboard->set("approach_drop_target", approach_drop_target);
    RCLCPP_INFO(node->get_logger(), "Blackboard: set('drop_target', drop_target Pose)");
    RCLCPP_INFO(node->get_logger(), "Blackboard: set('approach_drop_target', approach_drop_target Pose)");

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

    // Sequences for Pick/Drop/Homing
    std::vector<Move> pick_sequence = {
        {"pose", "approach_pick_target", {}, "", move_configs["mid_move"]},
        {"cartesian", "pick_target", {}, "", move_configs["slow_move"]},
    };

    std::vector<Move> drop_sequence = {
        {"pose", "approach_pick_target", {}, "", move_configs["mid_move"]},
        {"pose", "approach_drop_target", {}, "", move_configs["max_move"]},
        {"cartesian", "drop_target", {}, "", move_configs["slow_move"]},
    };

    std::vector<Move> home_position = {
        {"cartesian", "approach_drop_target", {}, "", move_configs["max_move"]},
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
     * Notice that on any string representing an XML snippet it's better to use _xml at the end of the name to give better
     * sense of what's in that variable.
     */
    std::string to_rest_xml = buildParallelPlanExecuteXML(
        robot_prefix + "toRest", rest_position, blackboard, robot_prefix, true);

    std::string scan_around_xml = buildParallelPlanExecuteXML(
        robot_prefix + "scanAround", scan_surroundings, blackboard, robot_prefix, true);

    std::string pick_object_xml = buildParallelPlanExecuteXML(
        robot_prefix + "pick", pick_sequence, blackboard, robot_prefix, true);

    std::string drop_object_xml = buildParallelPlanExecuteXML(
        robot_prefix + "drop", drop_sequence, blackboard, robot_prefix, true);

    std::string to_home_xml = buildParallelPlanExecuteXML(
        robot_prefix + "home", home_position, blackboard, robot_prefix, true);

    /*
     * Combine the parallel move sequence blocks in logic sequences for the entire logic.
     * Each subsequence will plan and execute in parallel, but the next subsequence will start planning all its moves
     * only after the last move of the previous sequence executes.
     * This can be useful for example if you have a camera mounted on the robot's arm and you want the next moves to be
     * planned only after the scene have been scanned. For example: the first move to_rest will take the robot in an
     * upright position with the camera pointing down to where the robot base is: I want the octomap to update before
     * continuing planning, but I don't need to wait for inputs or do any other action before planning.
     * Once the scan_around sequence is executed I will want to check for some inputs before continuing, so I terminate the
     * move sequence here: this will let me wrap this serie of sequences with other leaf nodes.
     * Other sequences like the ones to pick and to drop the objects will need to wait for inputs and/or give outputs, so
     * we sepearate them from the beginning.
     */

    // Translate it to xml tree leaf or branch
    std::string prep_sequence_xml = sequenceWrapperXML(
        robot_prefix + "ComposedPrepSequence", {to_rest_xml, scan_around_xml});
    std::string pick_sequence_xml = sequenceWrapperXML(
        robot_prefix + "ComposedPickSequence", {pick_object_xml});
    std::string drop_sequence_xml = sequenceWrapperXML(
        robot_prefix + "ComposedDropSequence", {drop_object_xml});
    std::string home_sequence_xml = sequenceWrapperXML(
        robot_prefix + "ComposedHomeSequence", {to_home_xml});

    // ----------------------------------------------------------------------------
    // 3) Build blocks for objects handling
    // ----------------------------------------------------------------------------
    std::vector<double> ground_dimension = {0.8, 0.8, 0.1};
    auto ground_pose = createPoseRPY(0.0, 0.0, -0.05, 0.0, 0.0, 0.0);

    std::vector<double> wall_dimension = {0.8, 0.02, 0.8};
    auto wall_pose = createPoseRPY(0.0, 0.4, 0.3, 0.0, 0.0, 0.0);

    std::vector<double> cylinderdimension = {0.1, 0.005};
    auto cylinderpose = createPoseRPY(0.1, 0.2, 0.005, 0.0, 1.57, 0.0);

    std::string mesh_file = "package://manymove_object_manager/meshes/unit_tube.stl";

    std::vector<double> mesh_scale = {0.01, 0.01, 0.1};                  //< The tube is vertical with dimension 1m x 1m x 1m. We scale it to 10x10x100 mm
    auto mesh_pose = createPoseRPY(0.1, -0.2, 0.2005, 0.785, 1.57, 0.0); //< We place it on the floor and lay it on its side, X+ facing down

    // Create object actions xml snippets (the object are created directly in the create*() functions relative to each type of object action)
    std::string check_ground_obj_xml = buildObjectActionXML("check_ground", createCheckObjectExists("obstacle_ground"));
    std::string check_wall_obj_xml = buildObjectActionXML("check_wall", createCheckObjectExists("obstacle_wall"));
    std::string check_cylinder_obj_xml = buildObjectActionXML("check_cylinder", createCheckObjectExists("graspable_cylinder"));
    std::string check_mesh_obj_xml = buildObjectActionXML("check_mesh", createCheckObjectExists("graspable_mesh"));

    std::string add_ground_obj_xml = buildObjectActionXML("add_ground", createAddPrimitiveObject("obstacle_ground", "box", ground_dimension, ground_pose));
    std::string add_wall_obj_xml = buildObjectActionXML("add_wall", createAddPrimitiveObject("obstacle_wall", "box", wall_dimension, wall_pose));
    std::string add_cylinder_obj_xml = buildObjectActionXML("add_cylinder", createAddPrimitiveObject("graspable_cylinder", "cylinder", cylinderdimension, cylinderpose));
    std::string add_mesh_obj_xml = buildObjectActionXML("add_mesh", createAddMeshObject("graspable_mesh", mesh_pose, mesh_file, mesh_scale[0], mesh_scale[1], mesh_scale[2]));

    // Compose the check and add sequence for objects
    std::string init_ground_obj_xml = fallbackWrapperXML("init_ground_obj", {check_ground_obj_xml, add_ground_obj_xml});
    std::string init_wall_obj_xml = fallbackWrapperXML("init_wall_obj", {check_wall_obj_xml, add_wall_obj_xml});
    std::string init_cylinder_obj_xml = fallbackWrapperXML("init_cylinder_obj", {check_cylinder_obj_xml, add_cylinder_obj_xml});
    std::string init_mesh_obj_xml = fallbackWrapperXML("init_mesh_obj", {check_mesh_obj_xml, add_mesh_obj_xml});

    // the name of the link to attach the object to
    std::string link_name = robot_prefix + "link_tcp";

    std::string attach_mesh_obj_xml = buildObjectActionXML("attach_mesh", createAttachObject("graspable_mesh", link_name));
    std::string detach_mesh_obj_xml = buildObjectActionXML("attach_mesh", createDetachObject("graspable_mesh", link_name));
    std::string remove_mesh_obj_xml = buildObjectActionXML("remove_mesh", createRemoveObject("graspable_mesh"));

    // ----------------------------------------------------------------------------
    // 4) Add GetObjectPoseAction Node and nodes to attach/detach objects
    // ----------------------------------------------------------------------------
    // Define the object ID and pose_key where the pose will be stored
    std::string object_id_for_pose = "graspable_mesh"; // Example object ID
    std::string pick_pose_key = "pick_target";
    std::string approach_pose_key = "approach_pick_target";

    // Define the transformation and reference orientation
    /*
     * The reference orientation determines how the tranform behaves: if we leave the reference orientation to all zeroes the
     * transform of the object will be referred to the world frame or, if it's attached, to the frame it is attached to.
     * Since we may want to grasp an object, we may need to move [TODO]...
     */
    std::vector<double> pick_pre_transform_xyz_rpy = {-0.002, 0.0, 0.0, 0.0, 1.57, 0.0};
    std::vector<double> approach_pre_transform_xyz_rpy = {-0.05, 0.0, 0.0, 0.0, 1.57, 0.0};
    std::vector<double> post_transform_xyz_rpy = {0.0, 0.0, -0.025, 3.14, 0.0, 0.0};

    // Translate get_pose_action to xml tree leaf
    std::string get_pick_pose_xml = buildObjectActionXML(
        "get_pick_pose", createGetObjectPose(
                             object_id_for_pose,
                             pick_pose_key,
                             pick_pre_transform_xyz_rpy,
                             post_transform_xyz_rpy));
    std::string get_approach_pose_xml = buildObjectActionXML(
        "get_approach_pose", createGetObjectPose(
                                 object_id_for_pose,
                                 approach_pose_key,
                                 approach_pre_transform_xyz_rpy,
                                 post_transform_xyz_rpy));

    // ----------------------------------------------------------------------------
    // 5) Define Signals calls:
    // ----------------------------------------------------------------------------

    // Let's send and receive signals only if the robot is real, and let's fake a 250ms on inputs otherwise

    std::string signal_gripper_close_xml = (is_robot_real ? buildSetOutputXML("GripperClose", "controller", 0, 1, robot_prefix) : "");
    std::string signal_gripper_open_xml = (is_robot_real ? buildSetOutputXML("GripperOpen", "controller", 0, 0, robot_prefix) : "");
    std::string check_gripper_close_xml = (is_robot_real ? buildCheckInputXML("WaitForSensor", "controller", 0, 1, robot_prefix, true, 0) : "<Delay delay_msec=\"250\">\n<AlwaysSuccess />\n</Delay>\n");
    std::string check_gripper_open_xml = (is_robot_real ? buildCheckInputXML("WaitForSensor", "controller", 0, 0, robot_prefix, true, 0) : "<Delay delay_msec=\"250\">\n  <AlwaysSuccess />\n</Delay>\n");
    std::string check_robot_state_xml = buildCheckRobotStateXML("CheckRobot", robot_prefix, "robot_ready", "error_code", "robot_mode", "robot_state", "robot_msg");
    std::string reset_robot_state_xml = buildResetRobotStateXML("ResetRobot", robot_prefix);

    std::string check_reset_robot_xml = (is_robot_real ? fallbackWrapperXML(robot_prefix + "CheckResetFallback", {check_robot_state_xml, reset_robot_state_xml}) : "<Delay delay_msec=\"250\">\n<AlwaysSuccess />\n</Delay>\n");

    // ----------------------------------------------------------------------------
    // 6) Combine the objects and moves in a sequences that can run a number of times:
    // ----------------------------------------------------------------------------

    // Let's build the full sequence in locically separated blocks:
    std::string spawn_objects_xml = sequenceWrapperXML("SpawnObjects", {init_ground_obj_xml, init_wall_obj_xml, init_cylinder_obj_xml, init_mesh_obj_xml});
    std::string get_grasp_object_poses_xml = sequenceWrapperXML("GetGraspPoses", {get_pick_pose_xml, get_approach_pose_xml});
    std::string go_to_pick_pose_xml = sequenceWrapperXML("GoToPickPose", {prep_sequence_xml, pick_sequence_xml});
    std::string close_gripper_xml = sequenceWrapperXML("CloseGripper", {signal_gripper_close_xml, check_gripper_close_xml, attach_mesh_obj_xml});
    std::string open_gripper_xml = sequenceWrapperXML("OpenGripper", {signal_gripper_open_xml, detach_mesh_obj_xml});

    // Repeat node must have only one children, so it also wrap a Sequence child that wraps the other children
    std::string repeat_forever_wrapper_xml = repeatWrapperXML(
        "RepeatForever",
        {check_reset_robot_xml,      //< We check if the robot is active, if not we try to reset it
         spawn_objects_xml,          //< We add all the objects to the scene
         get_grasp_object_poses_xml, //< We get the updated poses relative to the objects
         go_to_pick_pose_xml,        //< Prep sequence and pick sequence
         close_gripper_xml,          //< We attach the object
         drop_sequence_xml,          //< Drop sequence
         open_gripper_xml,           //< We detach the object
         home_sequence_xml,          //< Homing sequence
         remove_mesh_obj_xml},       //< We delete the object for it to be added on the next cycle in the original position
        -1);                         //< num_cycles=-1 for infinite

    // Combine prep_sequence_xml and pick_sequence_xml in a <Repeat> node single <Sequence>
    //    => Repeat node must have only one children, so it also wrap a Sequence child that wraps the other childs
    // std::string repeat_forever_wrapper_xml = repeatWrapperXML(
    //     "RepeatForever", {object_then_moves_xml}, -1); // num_cycles=-1 for infinite

    //    => MasterSequence with RepeatForever as child to set BehaviorTree ID and root main_tree_to_execute in the XML
    std::vector<std::string> master_branches_xml = {repeat_forever_wrapper_xml};
    std::string master_body = sequenceWrapperXML("GlobalMasterSequence", master_branches_xml);

    // ----------------------------------------------------------------------------
    // 7) Wrap everything into a top-level <root> with <BehaviorTree ID="MasterTree">
    // ----------------------------------------------------------------------------
    std::string final_tree_xml = mainTreeWrapperXML("MasterTree", master_body);

    RCLCPP_INFO(node->get_logger(), "=== Programmatically Generated Tree XML ===\n%s", final_tree_xml.c_str());

    // 8) Register node types
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<PlanningAction>("PlanningAction");
    factory.registerNodeType<ExecuteTrajectory>("ExecuteTrajectory");
    factory.registerNodeType<ResetTrajectories>("ResetTrajectories");

    factory.registerNodeType<AddCollisionObjectAction>("AddCollisionObjectAction");
    factory.registerNodeType<RemoveCollisionObjectAction>("RemoveCollisionObjectAction");
    factory.registerNodeType<AttachDetachObjectAction>("AttachDetachObjectAction");
    factory.registerNodeType<CheckObjectExistsAction>("CheckObjectExistsAction");
    factory.registerNodeType<GetObjectPoseAction>("GetObjectPoseAction");

    factory.registerNodeType<SetOutputAction>("SetOutputAction");
    factory.registerNodeType<GetInputAction>("GetInputAction");
    factory.registerNodeType<CheckRobotStateAction>("CheckRobotStateAction");
    factory.registerNodeType<ResetRobotStateAction>("ResetRobotStateAction");

    factory.registerNodeType<CheckBlackboardValue>("CheckBlackboardValue");

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

    // 10) ZMQ publisher (optional, to visualize in Groot)
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
