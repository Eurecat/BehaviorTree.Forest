#ifndef SYNC_MANAGER_HPP
#define SYNC_MANAGER_HPP

#include "behaviortree_forest/sync_manager_base.hpp"

namespace BT_SERVER
{
    class SyncManager : public SyncManagerBase
    {

    public:
        SyncManager(const rclcpp::Node::SharedPtr& node, BT::Blackboard::Ptr blackboard, const std::string& bt_uid, const BT::EutBehaviorTreeFactory& eut_bt_factory);
        ~SyncManager();

        void syncBBUpdateCB(const BBEntries::SharedPtr sync_entries_upd_msg) final;
        /// @brief Publish sync entries that have been updated and are in TO_SYNC or SYNCING and have not ack yet
        /// @return 
        void publishUpdatedSyncEntries() final;

    private:
        // Node ptr
        rclcpp::Node::SharedPtr node_;

        rclcpp::CallbackGroup::SharedPtr bb_upd_cb_group_;

        // Publishers
        rclcpp::Publisher<BBEntries>::SharedPtr sync_bb_pub_;

        // Subscribers
        rclcpp::Subscription<BBEntries>::SharedPtr sync_bb_sub_;
    };
} // namespace BT_SERVER
#endif // SYNC_MANAGER_HPP