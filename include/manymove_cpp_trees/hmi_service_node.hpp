#ifndef MANYMOVE_CPP_TREES_HMI_SERVICE_NODE_HPP
#define MANYMOVE_CPP_TREES_HMI_SERVICE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <behaviortree_cpp_v3/blackboard.h>

namespace manymove_cpp_trees
{
    /**
     * @brief A simple node that provides three services to modify BT blackboard keys.
     *
     * The services are:
     *   - start_execution: sets "stop_execution" to false.
     *   - stop_execution: sets "stop_execution" to true.
     *   - reset_program: sets both "stop_execution" and "abort_mission" to true.
     */
    class HMIServiceNode : public rclcpp::Node
    {
    public:
        explicit HMIServiceNode(const std::string &node_name, BT::Blackboard::Ptr blackboard);

    private:
        BT::Blackboard::Ptr blackboard_;

        // Service servers
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_execution_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_execution_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_program_srv_;

        // Service callbacks
        void handle_start_execution(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                    std::shared_ptr<std_srvs::srv::Empty::Response> response);

        void handle_stop_execution(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                   std::shared_ptr<std_srvs::srv::Empty::Response> response);

        void handle_reset_program(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                  std::shared_ptr<std_srvs::srv::Empty::Response> response);
    };
}

#endif // MANYMOVE_CPP_TREES_HMI_SERVICE_NODE_HPP
