#ifndef BT_ROSTOPIC_LOGGER_HPP
#define BT_ROSTOPIC_LOGGER_HPP

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/loggers/abstract_logger.h>
#include "behaviortree_forest_interfaces/msg/transition.hpp"
#include "behavior_tree_eut_plugins/eut_debug.h"

using Transition = behaviortree_forest_interfaces::msg::Transition;

namespace BT_SERVER
{
    class RosTopicTransitionLogger : public BT::StatusChangeLogger
    {
        public:
            RosTopicTransitionLogger(const BT::Tree& tree, rclcpp::Publisher<Transition>::SharedPtr pub);
            ~RosTopicTransitionLogger() override;
            virtual void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status, BT::NodeStatus status) override;
            virtual void flush() override;
        private:
            std::string ConvertStatusToString(BT::NodeStatus status);
            rclcpp::Publisher<Transition>::SharedPtr bt_transition_publisher_;
    };
}

#endif