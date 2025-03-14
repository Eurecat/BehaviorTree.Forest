#include <behaviortree_forest/sync_manager.hpp>
namespace BT_SERVER
{
    SyncManager::SyncManager(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    {
    }
    SyncManager::~SyncManager() {}

    std::vector<std::string> SyncManager::getSyncKeysList()
    {
      std::vector<std::string> keys;
  
      {
        std::scoped_lock lock{syncMap_lock_};  // protect this block
        keys.reserve(syncMap_.size()); // Reserve space to avoid unnecessary allocations
        for (const auto& element : syncMap_)
        {
          keys.push_back(element.first);
        }
      }
      return keys;
    }
    SyncMap SyncManager::getKeysValueToSync ()
    {
      std::scoped_lock lock{syncMap_lock_};  // protect this block  
      SyncMap ports_to_be_sync_cpy;
      for(auto& element : syncMap_)
      {
        BT_SERVER::SyncEntry sync_entry_cp = element.second.getSafeCopy();
        if(sync_entry_cp.entry && 
            !sync_entry_cp.entry->value.empty() &&
            // (!sync_entry_cp.entry->value.empty() || !BT::missingTypeInfo(sync_entry_cp.entry->info.type())) && // shall be not empty or shall have at least a type to be shared 
            sync_entry_cp.status == SyncStatus::TO_SYNC || sync_entry_cp.status == SyncStatus::SYNCING)
        {
          ports_to_be_sync_cpy.emplace(element.first, std::move(sync_entry_cp));
        }
      }
      return ports_to_be_sync_cpy;
    }

    bool SyncManager::updateSyncMapEntrySyncStatus(const std::string& key, SyncStatus sync_status) 
    { 
      std::scoped_lock lock{syncMap_lock_};  // protect this block
      auto syncmapentry_it = syncMap_.find(key);
      if(syncmapentry_it != syncMap_.end())
      {
        return syncmapentry_it->second.updateSyncStatus(sync_status);
        return true;
      }
      return false;
    }
  
    bool SyncManager::updateSyncMapEntrySyncStatus(const std::string& key, SyncStatus expected_sync_status, SyncStatus new_sync_status) 
    { 
      std::scoped_lock lock{syncMap_lock_};  // protect this block
      auto syncmapentry_it = syncMap_.find(key);
      if(syncmapentry_it != syncMap_.end())
      {
        return syncmapentry_it->second.updateSyncStatus(expected_sync_status, new_sync_status);
        return true;
      }
      return false;
    }

    bool SyncManager::addToSyncMap(const std::string& bb_key, SyncStatus sync_status, BT::Blackboard::Ptr blackboard_) 
    { 
      if(const auto entry_ptr = blackboard_->getEntry(bb_key))
      {
        std::scoped_lock lock{syncMap_lock_};  // protect this block
        if(syncMap_.find(bb_key) == syncMap_.end())
        {
          syncMap_.emplace(bb_key, std::move(SyncEntry(entry_ptr, sync_status)));
          return true;
        }
      }
      return false;
    }

    std::pair<bool, SyncEntry> SyncManager::checkForSyncKey(const std::string& key)
    {
      std::scoped_lock lock{syncMap_lock_};  // protect this block
      const auto it = syncMap_.find(key);
      if(it != syncMap_.end())
        return std::make_pair(true, it->second.getSafeCopy());
      else
        return std::make_pair(false, SyncEntry());
    }

    bool SyncManager::checkSyncStatus(const std::string& key, SyncStatus sync_status)
    {
      std::scoped_lock lock{syncMap_lock_};  // protect this block
      const auto sync_item_it = syncMap_.find(key);
      if(sync_item_it != syncMap_.end())
      {
        return sync_item_it->second.syncStatus() == sync_status;
      }
      return false;
    }

    void SyncManager::checkForToSyncEntries(BT::Blackboard::Ptr blackboard_)
    {
      std::vector<std::string> erased_entries;
      std::scoped_lock lock{syncMap_lock_};  // protect this block
      //SEARCH ON THE BLACKBOARD FOR NEW SYNC PORTS UPDATED
      for (auto& sync_entry_item : syncMap_)
      {
        const SyncEntry& sync_entry = sync_entry_item.second.getSafeCopy();
        if(blackboard_->entryInfo(sync_entry_item.first) == nullptr)
        {
          erased_entries.push_back(sync_entry_item.first);// erased entry
          continue;
        }
  
        if (sync_entry.entry && sync_entry.entry->stamp > sync_entry.last_synced)
        {
          sync_entry_item.second.updateSyncStatus(SyncStatus::TO_SYNC);
        }
      }
  
      // removed erased entries from SyncMap
      for(const auto& erased_entry : erased_entries)
        syncMap_.erase(syncMap_.find(erased_entry));
    }
} // namespace BT_SERVER
