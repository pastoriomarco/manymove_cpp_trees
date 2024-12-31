#include <behaviortree_cpp_v3/bt_factory.h>
#include "manymove_cpp_trees/planning_action.hpp"
#include "manymove_cpp_trees/execute_trajectory.hpp"

namespace manymove_cpp_trees
{
    void RegisterCustomNodes(BT::BehaviorTreeFactory& factory)
    {
        factory.registerNodeType<PlanningAction>("PlanningAction");
        factory.registerNodeType<ExecuteTrajectory>("ExecuteTrajectory");
    }
} // namespace manymove_cpp_trees

// Export a function that Groot can use to load the plugin
extern "C" void BT_RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    manymove_cpp_trees::RegisterCustomNodes(factory);
}
