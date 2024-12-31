#ifndef MANYMOVE_CPP_TREES_SERIALIZATION_HELPER_HPP
#define MANYMOVE_CPP_TREES_SERIALIZATION_HELPER_HPP

#include <string>
#include "manymove_planner/msg/move_manipulator_goal.hpp"

namespace manymove_cpp_trees
{
    std::string serializeMoveManipulatorGoal(const manymove_planner::msg::MoveManipulatorGoal& goal);
    manymove_planner::msg::MoveManipulatorGoal deserializeMoveManipulatorGoal(const std::string& serialized_goal);
}

#endif // MANYMOVE_CPP_TREES_SERIALIZATION_HELPER_HPP
