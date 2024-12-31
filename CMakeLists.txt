cmake_minimum_required(VERSION 3.5)
project(manymove_cpp_trees)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(manymove_planner REQUIRED)

# Include directories
include_directories(
  include
)

# Declare the shared library for custom BT nodes
add_library(custom_bt_nodes SHARED
  src/custom_bt_nodes.cpp
  src/planning_action.cpp
  src/execute_trajectory.cpp
)

# Link dependencies to the shared library
ament_target_dependencies(custom_bt_nodes
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  geometry_msgs
  moveit_msgs
  manymove_planner
)

# Set RPATH for the shared library to look in the same directory as the executable
set_target_properties(custom_bt_nodes PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# Install the shared library to lib/manymove_cpp_trees
install(TARGETS
  custom_bt_nodes
  DESTINATION lib/manymove_cpp_trees
)

# Declare the executable
add_executable(bt_client
  src/bt_client.cpp
  src/behavior_tree_xml_generator.cpp
  src/serialization_helper.cpp
)

# Link dependencies and the shared library to the executable
ament_target_dependencies(bt_client
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  geometry_msgs
  moveit_msgs
  manymove_planner
)

target_link_libraries(bt_client custom_bt_nodes)

# Set RPATH for the executable to look in the same directory for shared libraries
set_target_properties(bt_client PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# Install the executable to lib/manymove_cpp_trees
install(TARGETS
  bt_client
  DESTINATION lib/manymove_cpp_trees
)

# Install headers (optional, if other packages need them)
install(DIRECTORY include/
  DESTINATION include/
)

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
