#include "behaviortree_forest/bt_transition_logger.hpp"


#include "behaviortree_eut_plugins/eut_utils.h"

namespace BT_SERVER
{
    RosTopicTransitionLogger::RosTopicTransitionLogger(const BT::Tree& tree, rclcpp::Publisher<Transition>::SharedPtr pub) : BT::StatusChangeLogger(tree.rootNode()), bt_transition_publisher_(pub)
    {}
    RosTopicTransitionLogger::~RosTopicTransitionLogger()
    {}
    void RosTopicTransitionLogger::flush()
    {}
    void RosTopicTransitionLogger::callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status, BT::NodeStatus status)
    {
        Transition msg;
        msg.uid = node.UID();
        msg.name = node.name();
        msg.model = node.registrationName();
        msg.status = ConvertStatusToString(status);
        msg.prev_status = ConvertStatusToString(prev_status);
        bt_transition_publisher_->publish(msg);
    }
    std::string RosTopicTransitionLogger::ConvertStatusToString(BT::NodeStatus status)
    {
        switch(status)
        {
            case BT::NodeStatus::IDLE:
                return "IDLE";
                break;
            case BT::NodeStatus::RUNNING:
                return "RUNNING";
                break;
            case BT::NodeStatus::SUCCESS:
                return "SUCCESS";
                break;
            case BT::NodeStatus::FAILURE:
                return "FAILURE";
                break;
            case BT::NodeStatus::SKIPPED:
                return "SKIPPED";
                break;
            default:
                return "IDLE";
                break;
        }
    }
}   // end namespace