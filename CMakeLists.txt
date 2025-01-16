cmake_minimum_required(VERSION 3.5)
project(manymove_cpp_trees)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(manymove_planner REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(manymove_object_manager REQUIRED)
find_package(tf2 REQUIRED)

# Include directories
include_directories(include)

# Add executable for BT client
add_executable(bt_client
    src/bt_client.cpp
    src/action_nodes.cpp
    src/tree_helper.cpp
)

ament_target_dependencies(bt_client
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  manymove_planner
  manymove_object_manager
  moveit_msgs
  tf2
)

# Install executable
install(TARGETS
  bt_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers (tree_helper.hpp, planning_action.hpp, etc.)
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
