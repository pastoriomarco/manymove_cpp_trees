#ifndef MANYMOVE_CPP_TREES_TREE_HELPER_HPP
#define MANYMOVE_CPP_TREES_TREE_HELPER_HPP

#include <string>
#include <vector>
#include <unordered_map>
#include <geometry_msgs/msg/pose.hpp>
#include "manymove_cpp_trees/move.hpp"
#include "manymove_planner/msg/movement_config.hpp"

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
 *  buildParallelPlanExecuteXML(prefix, moves)
 *
 * Example output:
 *
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
 * @param prefix A label for the parallel block (e.g. "preparatory" or "pickAndHoming")
 * @param moves The vector of Move that we plan/execute in this parallel block
 * @return A string with the generated XML snippet
 */
std::string buildParallelPlanExecuteXML(const std::string &prefix,
                                        const std::vector<Move> &moves);

/**
 * @brief Wrap multiple snippets in a <Sequence> with a given name.
 */
std::string sequenceWrapperXML(const std::string &sequence_name,
                               const std::vector<std::string> &branches);

/**
 * @brief Wrap a snippet in a top-level <root> with <BehaviorTree ID="...">
 *        so it can be loaded by BehaviorTreeFactory.
 */
std::string mainTreeWrapperXML(const std::string &tree_id,
                               const std::string &content);

} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_TREE_HELPER_HPP
