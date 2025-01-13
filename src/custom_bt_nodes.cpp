#include <behaviortree_cpp_v3/bt_factory.h>
#include "manymove_cpp_trees/planning_action.hpp"
#include "manymove_cpp_trees/execute_trajectory.hpp"
// #include "custom_nodes.hpp" // if needed

extern "C" void BT_RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<manymove_cpp_trees::PlanningAction>("PlanningAction");
    factory.registerNodeType<manymove_cpp_trees::ExecuteTrajectory>("ExecuteTrajectory");
    // factory.registerNodeType<ResetValiditiesNode>("ResetValiditiesNode"); ...
}
