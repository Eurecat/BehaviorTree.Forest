#include <behaviortree_forest/sync_manager.hpp>


using std::placeholders::_1;

namespace BT_SERVER
{
    SyncManager::SyncManager(const rclcpp::Node::SharedPtr& node, BT::Blackboard::Ptr blackboard, const std::string& bt_uid)
    : node_(node), blackboard_(blackboard), bt_uid_(bt_uid)
    {
      bb_upd_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      rclcpp::SubscriptionOptions sync_bb_sub_options;
      sync_bb_sub_options.callback_group = bb_upd_cb_group_;  // Attach the subscription to the callback group
      // Updates subscriber server side
      sync_bb_sub_ = node->create_subscription<BBEntries>("behavior_tree_forest/broadcast_update", 10, std::bind(&SyncManager::syncBBUpdateCB, this, _1), sync_bb_sub_options);
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


    void SyncManager::syncBBUpdateCB(const BBEntries::SharedPtr sync_entries_upd_msg)
    {
      for(const auto& sync_entry_upd : sync_entries_upd_msg->entries)
        processSyncEntryUpdate(sync_entry_upd);
    }

    void SyncManager::processSyncEntryUpdate(const BBEntry& sync_entry_upd)
    {
      RCLCPP_DEBUG(this->node_->get_logger(), "[SyncManager %s]::syncBBUpdateCB\tkey=%s,\tvalue=%s,\ttype=%s", 
          bt_uid_.c_str(), sync_entry_upd.key.c_str(), sync_entry_upd.value.c_str(), sync_entry_upd.type.c_str());

      //Check that the Entry received is Sync for this BT
      const auto& checked_sync_entry = getSyncEntry(sync_entry_upd.key);
      if (!checked_sync_entry.has_value() || !checked_sync_entry.value().entry) return;

      RCLCPP_INFO(this->node_->get_logger(), "[SyncManager %s]::syncBBUpdateCB processing \tkey=%s,\tvalue=%s,\ttype=%s,\tbt_id=%s", 
          bt_uid_.c_str(), sync_entry_upd.key.c_str(), sync_entry_upd.value.c_str(), sync_entry_upd.type.c_str(), sync_entry_upd.bt_id.c_str());
      //Updates are republished from bt_server, check that the update comes from another BT before processing the value and update the BB
      if (sync_entry_upd.bt_id != bt_uid_)
      {
        bool update_successful = false;
        
        // const bool void_type = (sync_entry_upd.type == BT::demangle(typeid(void))); // source tree does not know the type of the value //TODO Check how bt.cpp v4 handle void: directly with Any??

        //Get the Entry
        std::shared_ptr<BT::Blackboard::Entry> entry_ptr = blackboard_->getEntry(sync_entry_upd.key); // in principle the same as checked_sync_entry.second.entry, in practice could have been already deleted from the BB

        // //retrieve string converter functor
        // const BT::StringConverter* string_converter_ptr = (void_type || !entry_ptr)? nullptr : &entry_ptr->string_converter;
        
        // //check string converter functor
        // if(!void_type && string_converter_ptr == nullptr)
        // {
        //     RCLCPP_ERROR(node_->get_logger(),"[SyncManager %s] Entry in Sync. BB for key [%s] has type [%s], but no string converter can be found for this type", 
        //         bt_uid_.c_str(), sync_entry_upd.key.c_str(), sync_entry_upd.type.c_str());
        //     return;
        // }

        //retrieve current entry in bt server bb
        if(entry_ptr)
        {
            // if(entry_ptr->port_info.missingTypeInfo()) is it necessary??? I would not update type info if received from another tree (i.e. from void to type T, with T != void)
            // {
            //     BT::Optional<BT::PortInfo> port_info_opt = bt_factory_ptr->getPortInfo(sync_entry_upd.type);
            //     if(!port_info_opt.has_value())
            //     {
            //         ROS_ERROR("[SyncManager %s] Entry in Sync. BB for key [%s] has type [%s], but it is an unknown type and therefore cannot be treated", tree_identifier_.c_str(), sync_entry_upd.key.c_str(), sync_entry_upd.type.c_str());
            //         return; // type unknown
            //     }
            //     tree_->rootBlackboard()->setPortInfo(sync_entry_upd.key, port_info_opt.value());
            // }            
            //Get the TypeInfo
            // auto type_info = blackboard_->entryInfo(sync_entry_upd.key);
            // if( BT::missingTypeInfo(type_info->type())  && !void_type && sync_entry_upd.type != type_info->typeName()) //TODO evaluate strictness and checks to be made here
            // {
            //     RCLCPP_ERROR(node_->get_logger(),"[SyncManager %s]. Entry in Sync. BB for key [%s] has type [%s], but receiving requests for update with type [%s]",
            //         bt_uid_.c_str(), sync_entry_upd.key.c_str(), type_info->typeName().c_str(), sync_entry_upd.type.c_str());
            //     return; // type inconsistencies, don't update
            // }
             RCLCPP_DEBUG(node_->get_logger(),"[SyncManager %s]::syncBBUpdateCB Trying to update sync port from SERVER for key [%s] with value [%s] : Accepted type %s, received %s", 
                  bt_uid_.c_str(),
                  sync_entry_upd.key.c_str(), sync_entry_upd.value.c_str(),
                 entry_ptr->info.typeName().c_str(), sync_entry_upd.type.c_str());
            if(BT::isStronglyTyped(sync_entry_upd.type) && entry_ptr->info.isStronglyTyped() && sync_entry_upd.type != entry_ptr->info.typeName())
            {
              if(entry_ptr->value.isNumber() && BT::isNumberType(sync_entry_upd.type))
              {
                // will be treated later within the library in the blackboard->set(...) call and might be still acceptable
              }
              else
              {
                // unacceptable inconsistency
                RCLCPP_WARN(node_->get_logger(),"[SyncManager %s]::syncBBUpdateCB Failed to update sync port from SERVER for key [%s] with value [%s]; Type inconsistency: Accepted type %s, but received %s", 
                  bt_uid_.c_str(),
                  sync_entry_upd.key.c_str(), sync_entry_upd.value.c_str(),
                  entry_ptr->info.typeName().c_str(), sync_entry_upd.type.c_str());
                return;
              }
            }

            try
            {
              BT::JsonExporter::ExpectedEntry expected_entry = {};
              if(!BT::isStronglyTyped(sync_entry_upd.type) && entry_ptr->info.isStronglyTyped())
              {
                BT::Any any = (entry_ptr->info.converter())(sync_entry_upd.value);
                blackboard_->set(sync_entry_upd.key, std::move(any));
                // if(sync_entry_upd.value.empty()) return;
              }
              else if(!BT::isStronglyTyped(sync_entry_upd.type) && !entry_ptr->info.isStronglyTyped())
              {
                if(!sync_entry_upd.value.empty()) blackboard_->set(sync_entry_upd.key, sync_entry_upd.value); // no type and stringified value both side (maintain unknown type)
                else return; // no type, no value
              }
              else
              {
                RCLCPP_INFO(node_->get_logger(),"[SyncManager %s]::syncBBUpdateCB about to parse as a json sync port from SERVER for key [%s] with value [%s]", 
                  bt_uid_.c_str(),
                  sync_entry_upd.key.c_str(), sync_entry_upd.value.c_str());
                const nlohmann::json json_value = nlohmann::json::parse(sync_entry_upd.value);
                expected_entry = BT::EutUtils::eutFromJson(json_value, entry_ptr->info.type());
                if(expected_entry.has_value()) blackboard_->set(sync_entry_upd.key, std::move(expected_entry.value().first));
                else 
                  RCLCPP_WARN(node_->get_logger(),"[SyncManager %s]::syncBBUpdateCB Failed to update sync port from SERVER for key [%s] with value [%s] not expected %s", 
                  bt_uid_.c_str(),
                  sync_entry_upd.key.c_str(), sync_entry_upd.value.c_str(),
                  expected_entry.error().c_str());
              }

              // if(expected_entry.has_value())
              // {
              //   blackboard_->set(sync_entry_upd.key, std::move(expected_entry.value().first));
                // if(entry_ptr->info.isStronglyTyped())
                // {
                //     // convert from string new value
                //     // BT::Any new_any_value = type_info->parseString(sync_entry_upd.value);
                    
                    
                //     // std::cout << "[BTWrapper "<<tree_identifier_<<"]::syncBBUpdateCB built new_any_value with type " << BT::demangle(new_any_value.type()) << " \n" << std::flush;
                //     // update it into the sync BB
                //     blackboard_->set(sync_entry_upd.key, std::move(new_any_value));
                // }
                // else
                //     blackboard_->set(sync_entry_upd.key, sync_entry_upd.value);
              // }
            
            }
            catch(const std::exception& e)
            {
              RCLCPP_WARN(this->node_->get_logger(), "[SyncManager %s]::syncBBUpdateCB fail to update value in BB for \tkey=%s,\tvalue=%s,\ttype=%s, error %s", 
                bt_uid_.c_str(), sync_entry_upd.key.c_str(), sync_entry_upd.value.c_str(), sync_entry_upd.type.c_str(), e.what());
              return;
            }

            // std::cout << "[BTWrapper "<<tree_identifier_<<"]::syncBBUpdateCB updated value in BB for key [" << sync_entry_upd.key << "] \n" << std::flush;
            // update_successful = true;
        }
      }

      if(sync_entry_upd.bt_id == bt_uid_)
      {
        std::scoped_lock entry_lock(checked_sync_entry.value().entry->entry_mutex);
        if(sync_entry_upd.sender_sequence_id < checked_sync_entry.value().entry->sequence_id)  // This is supposed to be the ACK for the sync sent by this tree and it's an old one, so the variable cannot be considered synced (yet)
          return;
      }
      
      if (!updateSyncMapEntrySyncStatus(sync_entry_upd.key, SyncStatus::SYNCED))  // being it yours or from another tree, mark it as synced
        RCLCPP_ERROR(this->node_->get_logger(),"[SyncManager %s]::syncBBUpdateCB Key [%s] failed to set status value [SYNCED]", 
        bt_uid_.c_str(), sync_entry_upd.key.c_str());
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
