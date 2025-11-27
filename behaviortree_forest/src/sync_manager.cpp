#include <behaviortree_forest/sync_manager.hpp>


using std::placeholders::_1;

namespace BT_SERVER
{
    SyncManager::SyncManager(const rclcpp::Node::SharedPtr& node, BT::Blackboard::Ptr blackboard, const std::string& bt_uid, const BT::EutBehaviorTreeFactory& eut_bt_factory)
    : 
      SyncManagerBase(blackboard, bt_uid, eut_bt_factory),
      node_(node)
    {
      setLogger(node_->get_logger());

      bb_upd_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      rclcpp::SubscriptionOptions sync_bb_sub_options;
      sync_bb_sub_options.callback_group = bb_upd_cb_group_;  // Attach the subscription to the callback group
      
      // Updates republisher for all trees (put latch to true atm, because seems a good option that you receive last update from the server)
      sync_bb_pub_ = node->create_publisher<BBEntries>("/behavior_tree_forest/local_update", 10);

      // Updates subscriber server side
      sync_bb_sub_ = node->create_subscription<BBEntries>("behavior_tree_forest/broadcast_update", 10, std::bind(&SyncManager::syncBBUpdateCB, this, _1), sync_bb_sub_options);
    }
    
    SyncManager::~SyncManager() {}

    void SyncManager::publishUpdatedSyncEntries()
    {
      BBEntries bb_entries_msg = getSyncEntriesToPublish();
      // Publish iff there is at least one item
      if (!bb_entries_msg.entries.empty())
        sync_bb_pub_->publish(bb_entries_msg);
    }

    void SyncManager::syncBBUpdateCB(const BBEntries::SharedPtr sync_entries_upd_msg)
    {
      for(const auto& sync_entry_upd : sync_entries_upd_msg->entries)
        processSyncEntryUpdate(sync_entry_upd);
    }
} // namespace BT_SERVER
