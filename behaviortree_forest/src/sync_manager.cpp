#include <behaviortree_forest/sync_manager.hpp>
namespace BT_SERVER
{
    SyncManager::SyncManager(const rclcpp::Node::SharedPtr& node, BT::Blackboard::Ptr blackboard, const std::string& bt_uid)
    : node_(node), blackboard_(blackboard), bt_uid_(bt_uid)
    {
    }
    
    SyncManager::~SyncManager() {}

    std::vector<std::string> SyncManager::getSyncKeysList()
    {
      // Always make sure you are treating a consistent sync map
      {
        refreshSyncMap();
      }

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
      // Always make sure you are treating a consistent sync map
      {
        refreshSyncMap();
      }

      std::scoped_lock lock{syncMap_lock_};  // protect this block  
      SyncMap ports_to_be_sync_cpy;
      for(auto& element : syncMap_)
      {
        // std::cout << element.first << "to be synced???\n" << std::flush;
        
        // std::cout << element.first << "to be synced??? exist = " << std::to_string(element.second.entry != nullptr) << 
        //   " not empty " << std::to_string(!(element.second.entry->value.empty())) << 
        //   " sync_status " << std::to_string(element.second.status == SyncStatus::TO_SYNC) << "\n" << std::flush;

        BT_SERVER::SyncEntry sync_entry_cp = element.second.getSafeCopy();
        // std::cout << element.first << "to be synced??? exist = " << std::to_string(sync_entry_cp.entry != nullptr) << 
        //   " not empty " << std::to_string(!sync_entry_cp.entry->value.empty()) << 
        //   " sync_status " << std::to_string(element.second.status == SyncStatus::TO_SYNC) << "\n" << std::flush;
        if(sync_entry_cp.entry && 
            !sync_entry_cp.entry->value.empty() &&
            // (!sync_entry_cp.entry->value.empty() || !BT::missingTypeInfo(sync_entry_cp.entry->info.type())) && // shall be not empty or shall have at least a type to be shared 
            sync_entry_cp.status == SyncStatus::TO_SYNC || sync_entry_cp.status == SyncStatus::SYNCING)
        {
          // std::cout << element.first << "to be synced\n" << std::flush;
          ports_to_be_sync_cpy.emplace(element.first, std::move(sync_entry_cp));
        }
      }
      return ports_to_be_sync_cpy;
    }

    bool SyncManager::updateSyncMapEntrySyncStatus(const std::string& key, SyncStatus sync_status) 
    { 
      // Always make sure you are treating a consistent sync map
      {
        refreshSyncMap();
      }


      // std::cout << "updateSyncMapEntrySyncStatus " << key << " to " << ((sync_status == SyncStatus::TO_SYNC)? "TO_SYNC" : "BB") << std::endl;
      std::scoped_lock lock{syncMap_lock_};  // protect this block
      auto syncmapentry_it = syncMap_.find(key);
      if(syncmapentry_it != syncMap_.end())
      {
        return syncmapentry_it->second.updateSyncStatus(sync_status);
      }
      return false;
    }
  
    bool SyncManager::updateSyncMapEntrySyncStatus(const std::string& key, SyncStatus expected_sync_status, SyncStatus new_sync_status) 
    { 
      // Always make sure you are treating a consistent sync map
      {
        refreshSyncMap();
      }


      std::scoped_lock lock{syncMap_lock_};  // protect this block
      auto syncmapentry_it = syncMap_.find(key);
      if(syncmapentry_it != syncMap_.end())
      {
        return syncmapentry_it->second.updateSyncStatus(expected_sync_status, new_sync_status);
      }
      return false;
    }

    bool SyncManager::addToSyncMap(const std::string& bb_key, SyncStatus sync_status) 
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


    // bool SyncManager::rmFromSyncMap(const std::string& bb_key)
    // {
    //   if(const auto entry_ptr = blackboard_->getEntry(bb_key))
    //   {
    //     std::scoped_lock lock{syncMap_lock_};  // protect this block
    //     if(syncMap_.find(bb_key) != syncMap_.end())
    //     {
    //       syncMap_.erase(bb_key);
    //       return true;
    //     }
    //   }
    //   return false;
    // }

    BT::Expected<SyncEntry> SyncManager::getSyncEntry(const std::string& key)
    {
      // Always make sure you are treating a consistent sync map
      {
        refreshSyncMap();
      }

      std::scoped_lock lock{syncMap_lock_};  // protect this block
      const auto it = syncMap_.find(key);
      if(it != syncMap_.end())
        return it->second.getSafeCopy();
      else
        return nonstd::make_unexpected(BT::StrCat("Cannot found sync entry ", key));
    }

    bool SyncManager::hasSyncStatus(const std::string& key, SyncStatus sync_status)
    {
      // Always make sure you are treating a consistent sync map
      {
        refreshSyncMap();
      }

            
      std::scoped_lock lock{syncMap_lock_};  // protect this block
      const auto sync_item_it = syncMap_.find(key);
      if(sync_item_it != syncMap_.end())
      {
        return sync_item_it->second.syncStatus() == sync_status;
      }
      return false;
    }

    void SyncManager::refreshSyncMap()
    {
      std::vector<std::string> erased_entries;
      std::scoped_lock lock{syncMap_lock_};  // protect this block
      //SEARCH ON THE BLACKBOARD FOR NEW SYNC PORTS UPDATED
      for (auto& [bb_key, sync_entry] : syncMap_)
      {
        const auto entry_ptr = blackboard_->getEntry(bb_key);

        if(entry_ptr == nullptr)
        {
          erased_entries.push_back(bb_key);// erased entry
          continue;
        }

        sync_entry.updateBBEntry(entry_ptr);// might have changed
  
        if (entry_ptr->stamp > sync_entry.lastSynced())
        {
          sync_entry.updateSyncStatus(SyncStatus::TO_SYNC);
        }
      }
  
      // removed erased entries from SyncMap
      for(const auto& erased_entry : erased_entries)
        syncMap_.erase(syncMap_.find(erased_entry));
    }
} // namespace BT_SERVER
