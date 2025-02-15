#include "behaviortree_forest/tree_wrapper.hpp"
#include "behaviortree_eut_plugins/utils/eut_utils.h"

#include "yaml-cpp/yaml.h"

namespace BT_SERVER
{
  TreeWrapper::TreeWrapper(const rclcpp::Node::SharedPtr& node)
    : node_(node), tree_ptr_(nullptr)
  {
    root_blackboard_ = BT::Blackboard::create();
  }

  TreeWrapper::~TreeWrapper() {}

  bool TreeWrapper::resetTree()
  {
      if(isTreeLoaded() )
      {
          tree_ptr_->haltTree();
          return true;
      }
      return false;
  }

  void TreeWrapper::removeTree()
  {
     resetTree();
     tree_ptr_.reset();
  }

  BT::NodeAdvancedStatus TreeWrapper::tickTree()
  {
    auto status = isTreeLoaded()? tree_ptr_->tickExactlyOnce() : BT::NodeStatus::IDLE;
    return toAdvancedNodeStatus(status);
  }

  BT::NodeAdvancedStatus TreeWrapper::toAdvancedNodeStatus (BT::NodeStatus status)
  {
    BT::NodeAdvancedStatus adv_status;
      switch(status)
      {
          case BT::NodeStatus::FAILURE:
              adv_status = BT::NodeAdvancedStatus::FAILURE;
              break;
          case BT::NodeStatus::IDLE:
              adv_status = BT::NodeAdvancedStatus::IDLE;
              break;
          case BT::NodeStatus::RUNNING:
              adv_status = BT::NodeAdvancedStatus::RUNNING;
              break;
          case BT::NodeStatus::SUCCESS:
              adv_status = BT::NodeAdvancedStatus::SUCCESS;
              break;
          case BT::NodeStatus::SKIPPED:
              adv_status = BT::NodeAdvancedStatus::SKIPPED;
              break;
      }
      if (debug_tree_ptr->isPaused())
        adv_status = BT::NodeAdvancedStatus::PAUSED;

      return adv_status;
  }

  void TreeWrapper::initGrootV2Pub()
  {
    //TODO: NEW GROOT
    groot_publisher_.reset();
    groot_publisher_ = std::make_shared<BT::Groot2Publisher>(*tree_ptr_, tree_server_port_);
  }

  //TODO: Bug counting nodes!
  size_t TreeWrapper::treeNodesCount() 
  {
      std::vector<const BT::TreeNode*> nodes;
      size_t nodes_count = 0;
      for(auto const& subtree : tree_ptr_->subtrees)
      {
        nodes_count += subtree->nodes.size();
      }
      return nodes_count;
  }

  TreeStatus TreeWrapper::buildTreeExecutionStatus()
  {
      TreeStatus tree_status_msg;

      static const auto to_msg_status = [](const BT::NodeAdvancedStatus& bt_status)
      {
          auto status { TreeStatus::IDLE };

          switch(bt_status)
          {
              case BT::NodeAdvancedStatus::FAILURE:
                  status = TreeStatus::FAILURE;
                  break;
              case BT::NodeAdvancedStatus::IDLE:
                  status = TreeStatus::IDLE;
                  break;
              case BT::NodeAdvancedStatus::RUNNING:
                  status = TreeStatus::RUNNING;
                  break;
              case BT::NodeAdvancedStatus::SUCCESS:
                  status = TreeStatus::SUCCESS;
                  break;
              case BT::NodeAdvancedStatus::SKIPPED:
                  status = TreeStatus::SKIPPED;
                  break;
              case BT::NodeAdvancedStatus::PAUSED:
                  status = TreeStatus::PAUSED;
                  break;
          }

          return status;
      };

      BT::NodeAdvancedStatus bt_tree_status = getTreeExecutionStatus();

      //Fill Service Tree info
      tree_status_msg.status = to_msg_status(bt_tree_status);
      tree_status_msg.name = tree_name_;
      tree_status_msg.file = tree_filename_;
      tree_status_msg.time_start = start_execution_time_;
      tree_status_msg.time = node_->get_clock()->now();
      tree_status_msg.uid = tree_uid_;
      tree_status_msg.details = execution_tree_error_;

      return tree_status_msg;
  }

  // Update and publishes atomically and mutually exclusive the current execution status.
  void TreeWrapper::updatePublishTreeExecutionStatus(const BT::NodeAdvancedStatus status, const bool avoid_duplicate)
  {
      {
          std::lock_guard<std::mutex> lk(status_lock_);
          bool duplicate = status == status_;
          status_ = status;
          if(duplicate && avoid_duplicate) return; //already published
      }
      publishExecutionStatus();
  }

  void TreeWrapper::publishExecutionStatus(bool error, std::string error_data)
  {
    TreeStatus status_msg = buildTreeExecutionStatus();

      if (error)
      {
          status_msg.status    = TreeStatus::CRASHED;
          status_msg.details      = error_data;
      }
      bt_execution_status_publisher_->publish(status_msg);
  }

  void TreeWrapper::initStatusPublisher()
  {
      bt_execution_status_publisher_ = node_->create_publisher<TreeStatus>("/"+tree_name_+"/execution_status", 100);
  }

  bool TreeWrapper::updateSyncMapEntrySyncStatus(const std::string& key, SyncStatus sync_status) 
  { 
    std::cout << "Updating sync status of " << key << " to " << BT_SERVER::toStr(sync_status) << "\n" << std::endl;
    std::scoped_lock lock{syncMap_lock_};  // protect this block
    auto syncmapentry_it = syncMap_.find(key);
    if(syncmapentry_it != syncMap_.end())
    {
      return syncmapentry_it->second.updateSyncStatus(sync_status);
      return true;
    }
    return false;
  }

  bool TreeWrapper::updateSyncMapEntrySyncStatus(const std::string& key, SyncStatus expected_sync_status, SyncStatus new_sync_status) 
  { 
    std::cout << "Updating sync status of " << key << " to " << BT_SERVER::toStr(new_sync_status) << "\n" << std::endl;
    std::scoped_lock lock{syncMap_lock_};  // protect this block
    auto syncmapentry_it = syncMap_.find(key);
    if(syncmapentry_it != syncMap_.end())
    {
      return syncmapentry_it->second.updateSyncStatus(expected_sync_status, new_sync_status);
      return true;
    }
    return false;
  }

  bool TreeWrapper::addToSyncMap(const std::string& bb_key, SyncStatus sync_status) 
  { 
    if(const auto entry_ptr = root_blackboard_->getEntry(bb_key))
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
  std::pair<bool, SyncEntry> TreeWrapper::checkForSyncKey(const std::string& key)
  {
    std::scoped_lock lock{syncMap_lock_};  // protect this block
    const auto it = syncMap_.find(key);
    if(it != syncMap_.end())
      return std::make_pair(true, it->second.getSafeCopy());
    else
      return std::make_pair(false, SyncEntry());
  }

  bool TreeWrapper::checkSyncStatus(const std::string& key, SyncStatus sync_status)
  {
    std::scoped_lock lock{syncMap_lock_};  // protect this block
    const auto sync_item_it = syncMap_.find(key);
    if(sync_item_it != syncMap_.end())
    {
      return sync_item_it->second.syncStatus() == sync_status;
    }
    return false;
  }

  void TreeWrapper::syncBBUpdateCB(const std::vector<BBEntry>& _bulk_upd)
  {
      for(const auto& upd : _bulk_upd)
          syncBBUpdateCB(upd);
  }

  void TreeWrapper::syncBBUpdateCB(const BBEntry& _single_upd)
  {
      //Check that the tree is loaded
      if(!isTreeLoaded()) return;

      std::cout << "[BTWrapper "<<tree_name_<<"]::syncBBUpdateCB " << 
          "\tkey=" << _single_upd.key << 
          "\ttype=" << _single_upd.type << 
          "\tvalue=" << _single_upd.value << "\n" << std::flush;
      //Check that the Entry received is Sync for this BT
      const auto& checked_sync_entry = checkForSyncKey(_single_upd.key);
      if (!checked_sync_entry.first || !checked_sync_entry.second.entry) return;

      //Updates are republished from bt_server, check that the update comes from another BT before processing the value and update the BB
      if (_single_upd.bt_id != tree_name_)
      {
        bool update_successful = false;
        
        // const bool void_type = (_single_upd.type == BT::demangle(typeid(void))); // source tree does not know the type of the value //TODO Check how bt.cpp v4 handle void: directly with Any??

        //Get the Entry
        std::shared_ptr<BT::Blackboard::Entry> entry_ptr = root_blackboard_->getEntry(_single_upd.key); // in principle the same as checked_sync_entry.second.entry, in practice could have been already deleted from the BB

        // //retrieve string converter functor
        // const BT::StringConverter* string_converter_ptr = (void_type || !entry_ptr)? nullptr : &entry_ptr->string_converter;
        
        // //check string converter functor
        // if(!void_type && string_converter_ptr == nullptr)
        // {
        //     RCLCPP_ERROR(node_->get_logger(),"[BTWrapper %s] Entry in Sync. BB for key [%s] has type [%s], but no string converter can be found for this type", 
        //         tree_name_.c_str(), _single_upd.key.c_str(), _single_upd.type.c_str());
        //     return;
        // }

        //retrieve current entry in bt server bb
        if(entry_ptr)
        {
            // if(entry_ptr->port_info.missingTypeInfo()) is it necessary??? I would not update type info if received from another tree (i.e. from void to type T, with T != void)
            // {
            //     BT::Optional<BT::PortInfo> port_info_opt = bt_factory_ptr->getPortInfo(_single_upd.type);
            //     if(!port_info_opt.has_value())
            //     {
            //         ROS_ERROR("[BTWrapper %s] Entry in Sync. BB for key [%s] has type [%s], but it is an unknown type and therefore cannot be treated", tree_identifier_.c_str(), _single_upd.key.c_str(), _single_upd.type.c_str());
            //         return; // type unknown
            //     }
            //     tree_->rootBlackboard()->setPortInfo(_single_upd.key, port_info_opt.value());
            // }            
            //Get the TypeInfo
            // auto type_info = root_blackboard_->entryInfo(_single_upd.key);
            // if( BT::missingTypeInfo(type_info->type())  && !void_type && _single_upd.type != type_info->typeName()) //TODO evaluate strictness and checks to be made here
            // {
            //     RCLCPP_ERROR(node_->get_logger(),"[BTWrapper %s]. Entry in Sync. BB for key [%s] has type [%s], but receiving requests for update with type [%s]",
            //         tree_name_.c_str(), _single_upd.key.c_str(), type_info->typeName().c_str(), _single_upd.type.c_str());
            //     return; // type inconsistencies, don't update
            // }
             RCLCPP_INFO(node_->get_logger(),"[BTWrapper %s]::syncBBUpdateCB Trying to update sync port from SERVER for key [%s] with value [%s] : Accepted type %s, received %s", 
                  tree_name_.c_str(),
                  _single_upd.key.c_str(), _single_upd.value.c_str(),
                 entry_ptr->info.typeName().c_str(), _single_upd.type.c_str());
            if(isStronglyTyped(_single_upd.type) && entry_ptr->info.isStronglyTyped() && _single_upd.type != entry_ptr->info.typeName())
            {
              if(entry_ptr->value.isNumber() && isNumberType(_single_upd.type))
              {
                // will be treated later within the library in the blackboard->set(...) call and might be still acceptable
              }
              else
              {
                // unacceptable inconsistency
                RCLCPP_ERROR(node_->get_logger(),"[BTWrapper %s]::syncBBUpdateCB Failed to update sync port from SERVER for key [%s] with value [%s]; Type inconsistency: Accepted type %s, but received %s", 
                  tree_name_.c_str(),
                  _single_upd.key.c_str(), _single_upd.value.c_str(),
                  entry_ptr->info.typeName().c_str(), _single_upd.type.c_str());
                return;
              }
            }

            try
            {
              
              const nlohmann::json json_value = nlohmann::json::parse(_single_upd.value);
              const BT::JsonExporter::ExpectedEntry expected_entry = BT::EutUtils::eutFromJson(json_value);
              if(expected_entry.has_value())
              {
                root_blackboard_->set(_single_upd.key, std::move(expected_entry.value().first));
                // if(entry_ptr->info.isStronglyTyped())
                // {
                //     // convert from string new value
                //     // BT::Any new_any_value = type_info->parseString(_single_upd.value);
                    
                    
                //     // std::cout << "[BTWrapper "<<tree_identifier_<<"]::syncBBUpdateCB built new_any_value with type " << BT::demangle(new_any_value.type()) << " \n" << std::flush;
                //     // update it into the sync BB
                //     root_blackboard_->set(_single_upd.key, std::move(new_any_value));
                // }
                // else
                //     root_blackboard_->set(_single_upd.key, _single_upd.value);
              }
            
            }
            catch(const std::exception& e)
            {
                std::cerr << "[BTWrapper "<<tree_name_<<"]::syncBBUpdateCB fail to update value in BB for key [" << _single_upd.key << "]: " << e.what() << " \n" << std::flush;
                return;
            }

            // std::cout << "[BTWrapper "<<tree_identifier_<<"]::syncBBUpdateCB updated value in BB for key [" << _single_upd.key << "] \n" << std::flush;
            // update_successful = true;
        }
      }

      if(_single_upd.bt_id == tree_name_ && _single_upd.sender_sequence_id < checked_sync_entry.second.entry->sequence_id)
      {
        // This is supposed to be the ACK for the sync sent by this tree and it's an old one, so the variable cannot be considered synced (yet)
        return;
      }
      
      
      updateSyncMapEntrySyncStatus(_single_upd.key, SyncStatus::SYNCED); // being it yours or from another tree, mark it as synced
  }

  void TreeWrapper::resetLoggers()
  {
    loggers_init_ = false;
    bt_logger_cout_.reset();
    bt_logger_trace_.reset();
    bt_logger_file_.reset();
    bt_logger_transition_rostopic_.reset();
    bt_logger_zmq_.reset();
  }

  void TreeWrapper::initLoggers()
  {
    //Create Loggers
    const char* home = getenv("HOME");
    log_folder_ = log_folder_.front() == '~' ? std::string(home) + log_folder_.substr(1, log_folder_.size() - 1) : log_folder_;
    log_folder_ = log_folder_.back() == '/' ? log_folder_ : log_folder_ + "/";
    std::stringstream file_base;
    const auto& current_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    file_base << log_folder_ << "behavior_tree_node-" << tree_name_ << "_tree-" << std::put_time(std::localtime(&current_time), "%F-%R");

    if (enable_file_log_)
    {
      const auto& log_file       = file_base.str() + ".fbl";
      bt_logger_file_ = std::make_unique<BT::FileLogger>(*tree_ptr_, log_file.c_str(), 20, true);
    }
    if (enable_minitrace_log_)
    {
      const auto& minitrace_file = file_base.str() + ".json";
      try
      {
          bt_logger_trace_ = std::make_unique<BT::MinitraceLogger>(*tree_ptr_, minitrace_file.c_str());
      }
      catch(const BT::LogicError& ex)
      {
          RCLCPP_WARN(node_->get_logger(),"Error initializing Minitrace logger for %s: %s", tree_name_.c_str(), ex.what());
      }
    }
    if (enable_cout_log_)
    {
      try
      {
          bt_logger_cout_ = std::make_unique<BT::StdCoutLogger>(*tree_ptr_);
      }
      catch(const BT::LogicError& ex)
      {
          RCLCPP_WARN(node_->get_logger(),"Error initializing Cout logger for %s: %s", tree_name_.c_str(), ex.what());
      }
    }
    if(enable_rostopic_log_)
    {
      auto bt_transition_publisher_ = node_->create_publisher<Transition>("/"+tree_name_+"/transition_status", 1);
      bt_logger_transition_rostopic_ = std::make_unique<RosTopicTransitionLogger>(*tree_ptr_, bt_transition_publisher_);
    }

    if (enable_zmq_log_) 
    {
      try
      {
        bt_logger_zmq_ = std::make_unique<BT::PublisherZMQ>(*debug_tree_ptr, 25, tree_publisher_port_,tree_server_port_);
      }
      catch (const std::exception& ex)
      {
        RCLCPP_WARN(node_->get_logger(),"Error initializing ZMQPublisher logger for %s: %s", tree_name_.c_str(), ex.what());
        bt_logger_zmq_.reset();
      }
      //TODO:
      //initGrootV2Pub();
    }

    loggers_init_ = true;
  }

  void TreeWrapper::loadAllPlugins()
  {
      RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS");
      loadPluginsFromROS(/*ros_plugin_directories_*/);
      // loadPluginsFromFolder();
      RCLCPP_INFO(node_->get_logger(),"LOADED PLUGINS");

  }

  // void TreeWrapper::loadPluginsFromFolder()
  // {
  //   bool import_from_folder = false;
  //   node_->get_parameter_or("import_from_folder",import_from_folder,false);

  //   if(import_from_folder)
  //   {
  //       RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS FROM FOLDER");
  //       std::string plugins_folder;
  //       if (!node_->get_parameter("plugins_folder",plugins_folder))
  //       {
  //           RCLCPP_WARN(node_->get_logger(),"Import from folder option is set, but folder param is missing");
  //       }
  //       else
  //       {
  //           using namespace boost::filesystem;

  //           if(!exists(plugins_folder))
  //           {
  //             RCLCPP_ERROR(node_->get_logger(),"Plugin folder %s does not exist.", plugins_folder.c_str());
  //             return;
  //           }

  //           auto directory_list = [&] { return boost::make_iterator_range(directory_iterator(plugins_folder), {}); };

  //           for(const auto& entry : directory_list())
  //           {
  //               if((!is_regular_file(entry) && !is_symlink(entry)) || entry.path().extension() != ".so") { continue; }

  //               try
  //               {
  //                 const auto& plugin_path = canonical(entry.path());

  //                 factory_.registerFromPlugin(plugin_path.string());
  //                 loaded_plugins_.emplace(plugin_path.string());
  //                 RCLCPP_INFO(node_->get_logger(),"Loaded plugin %s from folder %s", plugin_path.filename().string().c_str(), plugins_folder.c_str());
  //               }
  //               catch(const std::runtime_error& ex)
  //               {
  //                 RCLCPP_ERROR(node_->get_logger(),"Cannot load plugin %s from folder %s. Error: %s", entry.path().filename().string().c_str(), plugins_folder.c_str(), ex.what());
  //               }
  //           }
  //       }
  //       RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS FROM FOLDER OK");
  //   }
  // }

  void TreeWrapper::loadPluginsFromROS(/*std::vector<std::string> ros_plugins_folders*/)
  {
      RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS FROM ROS");

      // if empty, load all the findable ones
      if(ros_plugin_directories_.empty())
      {
        ros_plugin_directories_ = getBTPluginsFolders(); // pkgname/bt_plugins
      }

      bt_server::Params bt_params;
      bt_params.ros_plugins_timeout = 1000;
      bt_params.plugins = ros_plugin_directories_;
      RegisterPlugins(bt_params, factory_, node_);
      for(const auto& plugin : bt_params.plugins)
      {
        //RCLCPP_INFO(node_->get_logger(),"Added directory %s",plugin.c_str());
        loaded_plugins_.emplace(plugin);
      }
      RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS FROM ROS OK");
  }
  
  void TreeWrapper::initBB()
  {
    RCLCPP_INFO(node_->get_logger(),"CREATING BB");
    if (tree_bb_init_.size() > 0)
    {
      for(const auto& bb_init_abs_filepath: tree_bb_init_)
      {
          if(bb_init_abs_filepath.length() < 3) continue;
          initBBFromFile(bb_init_abs_filepath);
      }
    }
    RCLCPP_INFO(node_->get_logger(),"CREATING BB OK");
  }
  
  void TreeWrapper::initBBFromFile(const std::string& abs_file_path)
  {
    try 
    {
        // ROS_INFO("Initializing BB from YAML file %s", abs_file_path.c_str());
        YAML::Node config = YAML::LoadFile(abs_file_path);
        for(YAML::const_iterator it=config.begin();it!=config.end();++it)
        {
            std::string bb_key = it->first.as<std::string>();
            std::string bb_val = it->second.as<std::string>();
            
            //Check if is a SyncKey --> ${key}
            bool sync_entry = isSharedBlackboardPointer(bb_key);

            //Remove First '$' if is a SyncKey
            if (sync_entry) {bb_key.erase(0, 1);}

            //TODO: Inferred Values
            /*const BT::Expected<std::string> bbentry_value_inferred_keyvalues = replaceKeysWithStringValues(bb_val, root_blackboard_, true); // no effect if it has no key
            if(!bbentry_value_inferred_keyvalues)
            {
                // but will complain if it has a reference to a wrong key
                RCLCPP_ERROR(node_->get_logger(),"Init. of BB key %s for value %s, value inference did not succeed: %s", 
                    bb_key.c_str(), 
                    bb_val.c_str(),
                    bbentry_value_inferred_keyvalues.error().c_str());
                continue; // and skip this init
            }
            else
                bb_val = bbentry_value_inferred_keyvalues.value();*/

            RCLCPP_INFO(node_->get_logger(),"Init. BB key [\"%s\"] with value \"%s\"", bb_key.c_str(), bb_val.c_str());
            // use the string here and blackboard_ptr->set(...)
            root_blackboard_->set(bb_key, bb_val);
            
            //ADD VALUE TO SYNC MAP
            if (sync_entry)
            {
              addToSyncMap(bb_key, SyncStatus::TO_SYNC);
            }
        }
        RCLCPP_INFO(node_->get_logger(),"Initialized BB with %ld entries from YAML file %s", root_blackboard_->getKeys().size(), abs_file_path.c_str());
    }
    catch(const YAML::Exception& ex) 
    { 
        RCLCPP_ERROR(node_->get_logger(),"Initializing. BB key from file '%s' did not succeed: %s", abs_file_path.c_str(), ex.what());
    }
  }
  
  void TreeWrapper::checkForToSyncEntries()
  {
    std::vector<std::string> erased_entries;
    std::scoped_lock lock{syncMap_lock_};  // protect this block
    //SEARCH ON THE BLACKBOARD FOR NEW SYNC PORTS UPDATED
    for (auto& sync_entry_item : syncMap_)
    {
      const SyncEntry& sync_entry = sync_entry_item.second.getSafeCopy();
      if(root_blackboard_->entryInfo(sync_entry_item.first) == nullptr)
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

  void TreeWrapper::createTree(const std::string& full_path, bool tree_debug)
  {
    std::string tree_xml;
    std::vector<std::string> sync_keys;
    if (loadXMLToString(full_path, tree_xml))
    {
      //Extract SyncKeys from the XML Tree and Replace '$${key}' --> '${key}'
      sync_keys = extractSyncKeys(tree_xml);
      //Create the tree
      tree_ptr_ = std::make_shared<BT::Tree> (factory_.createTreeFromText(tree_xml,root_blackboard_));

      start_execution_time_ = node_->get_clock()->now();

      std::cout << "TREE INSTANTIATED WITH THIS BB INIT (BEFORE SYNC EXCHANGE)\n" << std::flush;
      root_blackboard_->debugMessage();
      std::cout << "\n\n" << std::flush;
    }
    else
    {
        throw BT::RuntimeError("Failed to open the file: "+full_path );
    }

    //Add the syncKeys to the syncMap
    for (auto key: sync_keys)
    {
      addToSyncMap(key, SyncStatus::TO_SYNC); // will not be added if not in the root_blackboard_
    }

    //Create debug_tree_ptr
    RCLCPP_INFO(node_->get_logger(),"Init debug_tree_ptr");
    debug_tree_ptr = std::make_shared<BT::DebuggableTree>(tree_ptr_,tree_debug);

  }

  SyncMap TreeWrapper::getKeysValueToSync ()
  {
    std::scoped_lock lock{syncMap_lock_};  // protect this block  
    RCLCPP_INFO(node_->get_logger()," Getting Keys Value to Sync"); 
    SyncMap ports_to_be_sync_cpy;
    for(auto& element : syncMap_)
    {
      BT_SERVER::SyncEntry sync_entry_cp = element.second.getSafeCopy();
      if(sync_entry_cp.entry && 
          !sync_entry_cp.entry->value.empty() &&
          // (!sync_entry_cp.entry->value.empty() || !BT::missingTypeInfo(sync_entry_cp.entry->info.type())) && // shall be not empty or shall have at least a type to be shared 
          sync_entry_cp.status == SyncStatus::TO_SYNC || sync_entry_cp.status == SyncStatus::SYNCING)
      {
        std::cout << "getKeysValueToSync select " << element.first << std::endl;
        ports_to_be_sync_cpy.emplace(element.first, std::move(sync_entry_cp));
      }
    }
    //RCLCPP_INFO(node_->get_logger()," Getting Keys Value to Sync OK"); 
    return ports_to_be_sync_cpy;
  }

  std::vector<std::string> TreeWrapper::getSyncKeysList()
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
}
