#include <behaviortree_forest/lifecycle_sync_manager.hpp>


using std::placeholders::_1;

namespace BT_SERVER
{
    LifecycleSyncManager::LifecycleSyncManager(const rclcpp_lifecycle::LifecycleNode::SharedPtr& lifecycle_node, BT::Blackboard::Ptr blackboard, const std::string& bt_uid, const BT::EutBehaviorTreeFactory& eut_bt_factory)
    :       
      SyncManagerBase(blackboard, bt_uid, eut_bt_factory),
      lifecycle_node_(lifecycle_node),
      active_(false)
    {}
    
    LifecycleSyncManager::~LifecycleSyncManager() {}

    bool LifecycleSyncManager::config()
    {
      if(!lifecycle_node_) return false;

      setLogger(lifecycle_node_->get_logger());

      bb_upd_cb_group_ = lifecycle_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      rclcpp::SubscriptionOptions sync_bb_sub_options;
      sync_bb_sub_options.callback_group = bb_upd_cb_group_;  // Attach the subscription to the callback group
      
      // Updates republisher for all trees (put latch to true atm, because seems a good option that you receive last update from the server)
      sync_bb_pub_ = lifecycle_node_->create_publisher<BBEntries>("/behavior_tree_forest/local_update", 10);

      // Updates subscriber server side
      sync_bb_sub_ = lifecycle_node_->create_subscription<BBEntries>("behavior_tree_forest/broadcast_update", 10, std::bind(&LifecycleSyncManager::syncBBUpdateCB, this, _1), sync_bb_sub_options);
      return true;
    }

    bool LifecycleSyncManager::cleanup()
    {
      active_ = false;
      resetLogger();
      if(sync_bb_pub_)
      {
        if(sync_bb_pub_->is_activated())
        {
          sync_bb_pub_->on_deactivate();
        }
        sync_bb_pub_.reset();
      }

      if(sync_bb_sub_)
      {
        sync_bb_sub_.reset();
      }
      return true;
    }

    bool LifecycleSyncManager::activate()
    {
      if(sync_bb_pub_)
      {
        sync_bb_pub_->on_activate();
      }
      active_ = true;
      return true;
    }

    bool LifecycleSyncManager::deactivate()
    {
      if(sync_bb_pub_)
      {
        sync_bb_pub_->on_deactivate();
      }
      active_ = false;
      return true;
    }


    void LifecycleSyncManager::publishUpdatedSyncEntries()
    {
      if(sync_bb_pub_ && sync_bb_pub_->is_activated())
      {
        BBEntries bb_entries_msg = getSyncEntriesToPublish();
        // Publish iff there is at least one item
        if (!bb_entries_msg.entries.empty())
          sync_bb_pub_->publish(bb_entries_msg);
      }
    }

    void LifecycleSyncManager::syncBBUpdateCB(const BBEntries::SharedPtr sync_entries_upd_msg)
    {
      if(!active_) return;
      
      for(const auto& sync_entry_upd : sync_entries_upd_msg->entries)
        processSyncEntryUpdate(sync_entry_upd);
    }
  
  } // namespace BT_SERVER
