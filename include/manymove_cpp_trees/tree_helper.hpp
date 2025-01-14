#ifndef MANYMOVE_CPP_TREES_TREE_HELPER_HPP
#define MANYMOVE_CPP_TREES_TREE_HELPER_HPP

#include <string>
#include <vector>
#include <unordered_map>
#include <geometry_msgs/msg/pose.hpp>
#include "manymove_cpp_trees/move.hpp"
#include "manymove_planner/msg/movement_config.hpp"
#include <behaviortree_cpp_v3/blackboard.h>

namespace manymove_cpp_trees
{

    /**
     * @brief Return some standard MovementConfig presets (max_move, mid_move, slow_move).
     */
    std::unordered_map<std::string, manymove_planner::msg::MovementConfig>
    defineMovementConfigs();

    /**
     * @brief Create a geometry_msgs::msg::Pose easily.
     */
    geometry_msgs::msg::Pose createPose(double x, double y, double z,
                                        double qx, double qy, double qz, double qw);

    /**
     * @brief Build a single parallel "plan + execute" block with unique naming
     *        using a static global counter for each move.
     *
     * The signature is:
     *  buildParallelPlanExecuteXML(prefix, moves, blackboard, [optional] reset_trajs)
     *
     * Example output:
     *
     *  <ResetTrajectories move_ids="id1,id2,..."/>
     *  <Parallel name="ParallelPlanExecute_{prefix}_{blockStartID}" success_threshold="2" failure_threshold="1">
     *    <Sequence name="PlanningSequence_{prefix}_{blockStartID}">
     *      <PlanningAction name="PlanMove_{globalID}" ...
     *         move_id="{globalID}"
     *         planned_move_id="{planned_move_id_{globalID}}"
     *         trajectory="{trajectory_{globalID}}"
     *         planning_validity="{validity_{globalID}}"/>
     *      ...
     *    </Sequence>
     *    <Sequence name="ExecutionSequence_{prefix}_{blockStartID}">
     *      <ExecuteTrajectory name="ExecMove_{globalID}"
     *         planned_move_id="{planned_move_id_{globalID}}"
     *         trajectory="{trajectory_{globalID}}"
     *         planning_validity="{validity_{globalID}}"/>
     *      ...
     *    </Sequence>
     *  </Parallel>
     *
     * Each move in @p moves increments a global counter, thus guaranteeing each
     * move_id, planned_move_id, validity_, trajectory_ are unique across the entire tree.
     *
     * This function also populates the blackboard with move IDs.
     *
     * WARNING:This function will require the output XML to be wrapped in a Control leaf node,
     *  since ResetTrajectories is detached from its own ParallelPlanExecute leaf node. This is done inside this
     *  function to allow grouping several ParallelPlanExecute in a single sequence to reduce tree's complexity. Moreover,
     *  you can set @p reset_trajs to false to avoid generating ResetTrajectories if your control logic don't require it,
     *  further simplifying the tree's structure.
     *
     * @param prefix A label for the parallel block (e.g., "preparatory" or "pickAndHoming")
     * @param moves The vector of Move that we plan/execute in this parallel block
     * @param blackboard The blackboard to populate with move IDs
     * @param reset_trajs This condition generates the ResetTrajectories leaf node to reset all the sequence's trajs before planning and executing
     * @return A string with the generated XML snippet
     */
    std::string buildParallelPlanExecuteXML(const std::string &prefix,
                                            const std::vector<Move> &moves,
                                            BT::Blackboard::Ptr blackboard,
                                            bool reset_trajs = true);

    /**
     * @brief Wrap multiple snippets in a <Sequence> with a given name.
     */
    std::string sequenceWrapperXML(const std::string &sequence_name,
                                   const std::vector<std::string> &branches);

    /**
     * @brief Wrap multiple snippets in a <ReactiveSequence> with a given name.
     *
     * This wrapper is useful for creating tree branches that continuously monitor
     * the robot's state and can interrupt planning/execution branches if necessary.
     *
     * @param sequence_name A unique name for the ReactiveSequence.
     * @param branches A vector of XML snippets representing the child nodes.
     * @return A string containing the generated XML snippet.
     */
    std::string reactiveWrapperXML(const std::string &sequence_name,
                                   const std::vector<std::string> &branches);

    /**
     * @brief Wrap multiple snippets in a <RepeatNode> node with a given name.
     *
     * This wrapper allows repeating its child node multiple times based on the specified
     * number of attempts. To repeat indefinitely, set num_cycles to -1.
     *
     * @param sequence_name A unique name for the RepeatNode node.
     * @param branches A vector of XML snippets representing the child nodes.
     * @param num_cycles Number of repeat attempts (-1 for infinite retries).
     * @return A string containing the generated XML snippet.
     */
    std::string repeatWrapperXML(const std::string &sequence_name,
                                const std::vector<std::string> &branches,
                                const int num_cycles = -1);

    /**
     * @brief Wrap a snippet in a top-level <root> with <BehaviorTree ID="...">
     *        so it can be loaded by BehaviorTreeFactory.
     */
    std::string mainTreeWrapperXML(const std::string &tree_id,
                                   const std::string &content);

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_TREE_HELPER_HPP
