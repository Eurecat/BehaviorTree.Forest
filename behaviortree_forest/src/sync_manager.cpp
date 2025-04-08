#include <behaviortree_forest/sync_manager.hpp>


using std::placeholders::_1;

namespace BT_SERVER
{
    SyncManager::SyncManager(const rclcpp::Node::SharedPtr& node, BT::Blackboard::Ptr blackboard, const std::string& bt_uid, const BT::EutBehaviorTreeFactory& eut_bt_factory)
    : node_(node), blackboard_(blackboard), bt_uid_(bt_uid), eut_bt_factory_(eut_bt_factory)
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

    BBEntries SyncManager::getSyncEntriesToPublish(const std::string bt_id)
    {
      BT_SERVER::SyncMap entries_map = getKeysValueToSync();

      BBEntries entries;
      for(const auto& ser_entry : entries_map)
      {
          const std::shared_ptr<BT::Blackboard::Entry> entry_check_ptr = blackboard_->getEntry(ser_entry.first);  
          if(entry_check_ptr)
          {
              BBEntry message;
              message.key = ser_entry.first;
              message.type = BT::demangle(entry_check_ptr->info.type());
              auto val = BT::EutUtils::eutToJsonString(ser_entry.first, blackboard_);
              if (val.has_value())
                  message.value = val.value();
              else
                  message.value = "";
              message.bt_id = bt_id;
              message.sender_sequence_id = entry_check_ptr->sequence_id;
              //RCLCPP_INFO(nh_->get_logger(),"KnowledgeManager Adding ToPubEntry -> KEY: %s TYPE: %s VAL: %s", message.key.c_str(), message.type.c_str(), message.value.c_str());
                   
              if(updateSyncMapEntrySyncStatus(ser_entry.first, SyncStatus::TO_SYNC, SyncStatus::SYNCING) || hasSyncStatus(ser_entry.first,SyncStatus::SYNCING))
              {
                  entries.entries.push_back(message);
              }
              else
              {
                std::cerr << "[SyncManager]::getSyncEntriesToPublish Error getting entry [" << ser_entry.first << "] To publish!" << std::endl;
              }
          }
          else
          {
            std::cerr << "[SyncManager]::getSyncEntriesToPublish Error getting entry [" << ser_entry.first << "] To publish!" << std::endl;
          }
      }
      return entries;
    }
    void SyncManager::syncBBUpdateCB(const BBEntries::SharedPtr sync_entries_upd_msg)
    {
      for(const auto& sync_entry_upd : sync_entries_upd_msg->entries)
        processSyncEntryUpdate(sync_entry_upd);
    }

    bool SyncManager::processSyncEntryUpdate(const BBEntry& sync_entry_upd)
    {
      std::string remote_entry_type_str = sync_entry_upd.type.rfind("nlohmann::json", 0) == std::string::npos ? sync_entry_upd.type : "json" ;
      RCLCPP_DEBUG(this->node_->get_logger(), "[SyncManager]::syncBBUpdateCB\tkey=%s,\tvalue=%s,\ttype=%s\n", 
           sync_entry_upd.key.c_str(), sync_entry_upd.value.c_str(), remote_entry_type_str.c_str());

      //Check that the Entry received is Sync for this BT
      const auto& checked_sync_entry = getSyncEntry(sync_entry_upd.key);
      if (!checked_sync_entry.has_value() || !checked_sync_entry.value().entry) return false;

      RCLCPP_INFO(this->node_->get_logger(), "[SyncManager %s]::syncBBUpdateCB processing \tkey=%s,\tvalue=%s,\ttype=%s,\tbt_id=%s", 
          bt_uid_.c_str(), sync_entry_upd.key.c_str(), sync_entry_upd.value.c_str(), remote_entry_type_str.c_str(), sync_entry_upd.bt_id.c_str());
      
      bool update_successful = false;
      //Get the Entry
      std::shared_ptr<BT::Blackboard::Entry> entry_ptr = blackboard_->getEntry(sync_entry_upd.key); // in principle the same as checked_sync_entry.second.entry, in practice could have been already deleted from the BB
      
      if(entry_ptr)
      {
        //Updates are republished from bt_server, check that the update comes from another BT before processing the value and update the BB 
        // or if the type has been changed to a strongly typed one from the server
        if (sync_entry_upd.bt_id != bt_uid_ || sync_entry_upd.type != BT::demangle(entry_ptr->info.type()))
        {
          // const bool void_type = (sync_entry_upd.type == BT::demangle(typeid(void))); // source tree does not know the type of the value //TODO Check how bt.cpp v4 handle void: directly with Any??
          //retrieve string converter functor
          // const BT::StringConverter* string_converter_ptr = (void_type || !entry_ptr)? nullptr : &entry_ptr->string_converter;
          
          //check string converter functor
          // if(!void_type && string_converter_ptr == nullptr)
          // {
          //     RCLCPP_ERROR(node_->get_logger(),"[SyncManager %s] Entry in Sync. BB for key [%s] has type [%s], but no string converter can be found for this type", 
          //         bt_uid_.c_str(), sync_entry_upd.key.c_str(), sync_entry_upd.type.c_str());
          //     return;
          // }
        
        // const bool void_type = (sync_entry_upd.type == BT::demangle(typeid(void))); // source tree does not know the type of the value //TODO Check how bt.cpp v4 handle void: directly with Any??


        // //retrieve string converter functor
        // const BT::StringConverter* string_converter_ptr = (void_type || !entry_ptr)? nullptr : &entry_ptr->string_converter;
        
        // //check string converter functor
        // if(!void_type && string_converter_ptr == nullptr)
        // {
        //     RCLCPP_ERROR(node_->get_logger(),"[SyncManager %s] Entry in Sync. BB for key [%s] has type [%s], but no string converter can be found for this type", 
        //         bt_uid_.c_str(), sync_entry_upd.key.c_str(), remote_entry_type_str.c_str());
        //     return;
        // }

            // if(entry_ptr->port_info.missingTypeInfo()) is it necessary??? I would not update type info if received from another tree (i.e. from void to type T, with T != void)
            // {
            //     BT::Optional<BT::PortInfo> port_info_opt = bt_factory_ptr->getPortInfo(sync_entry_upd.type);
            //     if(!port_info_opt.has_value())
            //     {
            //         ROS_ERROR("[SyncManager %s] Entry in Sync. BB for key [%s] has type [%s], but it is an unknown type and therefore cannot be treated", tree_identifier_.c_str(), sync_entry_upd.key.c_str(), remote_entry_type_str.c_str());
            //         return; // type unknown
            //     }
            //     tree_->rootBlackboard()->setPortInfo(sync_entry_upd.key, port_info_opt.value());
            // }            
            //Get the TypeInfo
            // auto type_info = blackboard_->entryInfo(sync_entry_upd.key);
            // if( BT::missingTypeInfo(type_info->type())  && !void_type && sync_entry_upd.type != type_info->typeName()) //TODO evaluate strictness and checks to be made here
            // {
            //     RCLCPP_ERROR(node_->get_logger(),"[SyncManager %s]. Entry in Sync. BB for key [%s] has type [%s], but receiving requests for update with type [%s]",
            //         bt_uid_.c_str(), sync_entry_upd.key.c_str(), type_info->typeName().c_str(), remote_entry_type_str.c_str());
            //     return; // type inconsistencies, don't update
            // }
            //  RCLCPP_DEBUG(node_->get_logger(),"[SyncManager]::syncBBUpdateCB Trying to update sync port from SERVER for key [%s] with value [%s] : Accepted type %s, received %s", 
            //       sync_entry_upd.key.c_str(), sync_entry_upd.value.c_str(),
            //      entry_ptr->info.typeName().c_str(), remote_entry_type_str.c_str());

            std::string current_entry_type_str = entry_ptr->info.typeName().rfind("nlohmann::json", 0) == std::string::npos ? entry_ptr->info.typeName() : "json" ;
            
            // CHECK TYPE INCONSISTENCIES
            if(BT::isStronglyTyped(sync_entry_upd.type) && entry_ptr->info.isStronglyTyped() && sync_entry_upd.type != entry_ptr->info.typeName())
            {
              if((entry_ptr->value.isNumber() || entry_ptr->value.isType<bool>()) && BT::isNumberType(sync_entry_upd.type))
              {
                // will be treated later within the library in the blackboard->set(...) call and might be still acceptable
              }
              else
              {
                // unacceptable inconsistency
                RCLCPP_WARN(node_->get_logger(),"[SyncManager]::syncBBUpdateCB Failed to update sync port from SERVER for key [%s] with value [%s]; Type inconsistency: Accepted type %s, but received %s", 
                  sync_entry_upd.key.c_str(), sync_entry_upd.value.c_str(),
                  entry_ptr->info.typeName().c_str(), remote_entry_type_str.c_str());
                return false;
              }
            }

            try
            {
              BT::JsonExporter::ExpectedEntry expected_entry = {};
              if(!BT::isStronglyTyped(sync_entry_upd.type) && entry_ptr->info.isStronglyTyped())
              {
                // CASE A: Remote Entry !StronglyTyped & Local Entry StronglyTyped
                // BT::Any any = (entry_ptr->info.converter())(sync_entry_upd.value);
                // blackboard_->set(sync_entry_upd.key, std::move(any));
                blackboard_->set(sync_entry_upd.key, sync_entry_upd.value);
                // if(sync_entry_upd.value.empty()) return;
              }
              else if(!BT::isStronglyTyped(sync_entry_upd.type) && !entry_ptr->info.isStronglyTyped())
              {
                // CASE B: Remote Entry !StronglyTyped & Local Entry !StronglyTyped
                if(!sync_entry_upd.value.empty()) 
                {
                  blackboard_->set(sync_entry_upd.key, sync_entry_upd.value); // no type and stringified value both side (maintain unknown type)
                  {
                    std::scoped_lock lock(entry_ptr->entry_mutex);
                    entry_ptr->info = BT::TypeInfo(); //BT::AnyTypeAllowed;
                  }
                }
                else return false; // no type, no value
              }
              else
              {
                // CASE C: Remote Entry StronglyTyped & Local Entry !StronglyTyped
                // CASE D: Remote Entry StronglyTyped & Local Entry StronglyTyped
                
                bool strongly_typing = BT::isStronglyTyped(sync_entry_upd.type) && !entry_ptr->info.isStronglyTyped(); // CASE C
                BT::TypeInfo type_info = entry_ptr->info;
                if(strongly_typing)
                {
                  BT::Expected<BT::TypeInfo> new_type_info = eut_bt_factory_.getTypeInfo(sync_entry_upd.type);
                  if(!new_type_info.has_value())
                  {
                    RCLCPP_WARN(node_->get_logger(),"[SyncManager %s]::syncBBUpdateCB Failed to update sync port from SERVER for key [%s] with value [%s] unknown type %s", 
                        bt_uid_.c_str(),
                        sync_entry_upd.key.c_str(), sync_entry_upd.value.c_str(),
                        remote_entry_type_str.c_str());
                    return false;
                  }
                  type_info = new_type_info.value();
                }


                RCLCPP_INFO(node_->get_logger(),"[SyncManager %s]::syncBBUpdateCB about to parse as a json sync port from SERVER for key [%s] with value [%s]", 
                  bt_uid_.c_str(),
                  sync_entry_upd.key.c_str(), sync_entry_upd.value.c_str());
                const nlohmann::json json_value = nlohmann::json::parse(sync_entry_upd.value);
                expected_entry = BT::EutUtils::eutFromJson(json_value, strongly_typing? type_info.type() : entry_ptr->info.type()); // TODO Fix bug of the type for Case C
                if(expected_entry.has_value()) 
                {
                  blackboard_->set(sync_entry_upd.key, std::move(expected_entry.value().first));
                  if(strongly_typing) // CASE C
                  {
                    std::scoped_lock lock(entry_ptr->entry_mutex);
                    entry_ptr->info = type_info;
                  }
                }
                else 
                  RCLCPP_WARN(node_->get_logger(),"[SyncManager]::syncBBUpdateCB Failed to update sync port from SERVER for key [%s] with value [%s] not expected %s", 
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
                bt_uid_.c_str(), sync_entry_upd.key.c_str(), sync_entry_upd.value.c_str(), remote_entry_type_str.c_str(), e.what());
              return false;
            }

            // std::cout << "[BTWrapper "<<tree_identifier_<<"]::syncBBUpdateCB updated value in BB for key [" << sync_entry_upd.key << "] \n" << std::flush;
            // update_successful = true;
        }
        
        
        if(sync_entry_upd.bt_id == bt_uid_)
        {
          std::scoped_lock entry_lock(checked_sync_entry.value().entry->entry_mutex);
          if(sync_entry_upd.sender_sequence_id < checked_sync_entry.value().entry->sequence_id)  // This is supposed to be the ACK for the sync sent by this tree and it's an old one, so the variable cannot be considered synced (yet)
            return false; // do not update the status, it is not yet synced
        }
        
        if (updateSyncMapEntrySyncStatus(sync_entry_upd.key, SyncStatus::SYNCED))  // being it yours or from another tree, mark it as synced
          update_successful = true;
        else 
          RCLCPP_ERROR(this->node_->get_logger(),"[SyncManager %s]::syncBBUpdateCB Key [%s] failed to set status value [SYNCED]", 
            bt_uid_.c_str(), sync_entry_upd.key.c_str());
      }
      return update_successful;

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
