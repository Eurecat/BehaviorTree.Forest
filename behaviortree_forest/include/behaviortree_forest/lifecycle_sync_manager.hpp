#ifndef LIFECYCLE_SYNC_MANAGER_HPP
#define LIFECYCLE_SYNC_MANAGER_HPP


#include "behaviortree_forest/sync_manager_base.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace BT_SERVER
{
    class LifecycleSyncManager : public SyncManagerBase
    {

    public:
        LifecycleSyncManager(const rclcpp_lifecycle::LifecycleNode::SharedPtr& lifecycle_node, BT::Blackboard::Ptr blackboard, const std::string& bt_uid, const BT::EutBehaviorTreeFactory& eut_bt_factory);
        ~LifecycleSyncManager();
        

        void syncBBUpdateCB(const BBEntries::SharedPtr sync_entries_upd_msg) final;
        void publishUpdatedSyncEntries() final;

        // Lifecycle node management
        bool config();
        bool cleanup();
        bool activate();
        bool deactivate();

    private:
        // LifecycleNode ptr
        rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;

        rclcpp::CallbackGroup::SharedPtr bb_upd_cb_group_;

        //Publishers
        rclcpp_lifecycle::LifecyclePublisher<BBEntries>::SharedPtr sync_bb_pub_;

        //Subscribers
        rclcpp::Subscription<BBEntries>::SharedPtr sync_bb_sub_;

        bool active_;
    };
} // namespace BT_SERVER
#endif // LIFECYCLE_SYNC_MANAGER_HPP