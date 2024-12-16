#include "tree_wrapper.hpp"

#include "yaml-cpp/yaml.h"

namespace BT_SERVER
{
  TreeWrapper::TreeWrapper(const rclcpp::Node::SharedPtr& node)
    : node_(node)
  {
    global_blackboard_ = BT::Blackboard::create();

    RCLCPP_INFO(node_->get_logger(),"Init tree_ptr");
    // Init Tree_ptr and debugTree
    tree_ptr_ = std::make_shared<BT::Tree> ();
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
     //is_tree_loaded_ = true;
  }

  BT::NodeAdvancedStatus TreeWrapper::tickTree()
  {
    auto status = tree_ptr_->tickOnce();
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

  void TreeWrapper::syncBBUpdateCB(const std::vector<BBEntry>& _bulk_upd)
  {
      for(const auto& upd : _bulk_upd)
          syncBBUpdateCB(upd);
  }

  void TreeWrapper::syncBBUpdateCB(const BBEntry& _single_upd)
  {
      //Check that the tree is loaded
      if(!is_tree_loaded_) return;

      //Updates are republished from bt_server, check that the update comes from another BT
      if (_single_upd.bt_id == tree_name_) return;

      //Check that the Entry received is Sync for this BT
      if (!hasSyncKey(_single_upd.key)) return;

      // std::cout << "[BTWrapper "<<tree_identifier_<<"]::syncBBUpdateCB " << 
      //     "\tkey=" << _single_upd.key << 
      //     "\ttype=" << _single_upd.type << 
      //     "\tvalue=" << _single_upd.value << "\n" << std::flush;
      // bool update_successful = false;
      
      const bool void_type = (_single_upd.type == BT::demangle(typeid(void))); // source tree does not know the type of the value

      //Get the Entry
      const BT::Blackboard::Entry* entry_ptr = global_blackboard_->getEntry(_single_upd.key).get();

     //retrieve string converter functor
      const BT::StringConverter* string_converter_ptr = (void_type || !entry_ptr)? nullptr : &entry_ptr->string_converter;
      
      //check string converter functor
      if(!void_type && string_converter_ptr == nullptr)
      {
          RCLCPP_ERROR(node_->get_logger(),"[BTWrapper %s] Entry in Sync. BB for key [%s] has type [%s], but no string converter can be found for this type", 
              tree_name_.c_str(), _single_upd.key.c_str(), _single_upd.type.c_str());
          return;
      }

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
          auto type_info = global_blackboard_->entryInfo(_single_upd.key);
          if( BT::missingTypeInfo(type_info->type())  && !void_type && _single_upd.type != type_info->typeName()) //TODO evaluate strictness and checks to be made here
          {
              RCLCPP_ERROR(node_->get_logger(),"[BTWrapper %s]. Entry in Sync. BB for key [%s] has type [%s], but receiving requests for update with type [%s]",
                  tree_name_.c_str(), _single_upd.key.c_str(), type_info->typeName().c_str(), _single_upd.type.c_str());
              return; // type inconsistencies, don't update
          }
          
          try
          {
              if(!void_type)
              {
                  // convert from string new value
                  BT::Any new_any_value = type_info->parseString(_single_upd.value);
                  
                  // std::cout << "[BTWrapper "<<tree_identifier_<<"]::syncBBUpdateCB built new_any_value with type " << BT::demangle(new_any_value.type()) << " \n" << std::flush;
                  // update it into the sync BB
                  global_blackboard_->set(_single_upd.key, std::move(new_any_value));
              }
              else
                  global_blackboard_->set(_single_upd.key, _single_upd.value);
          
          }
          catch(const std::exception& e)
          {
              std::cerr << "[BTWrapper "<<tree_name_<<"]::syncBBUpdateCB fail to update value in BB for key [" << _single_upd.key << "]: " << e.what() << " \n" << std::flush;
              return;
          }

          // std::cout << "[BTWrapper "<<tree_identifier_<<"]::syncBBUpdateCB updated value in BB for key [" << _single_upd.key << "] \n" << std::flush;
          // update_successful = true;
      }
    //Update sync timestamp
    last_sync_update_ = std::chrono::steady_clock::now().time_since_epoch();
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
      bt_logger_zmq_ = std::make_unique<BT::PublisherZMQ>(*debug_tree_ptr, 25, tree_publisher_port_,tree_server_port_);
      //TODO:
      //initGrootV2Pub();
    }

    loggers_init_ = true;
  }

  void TreeWrapper::loadAllPlugins()
  {
      RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS");
      loadPluginsFromROS(ros_plugin_directories_);
      loadPluginsFromFolder();
      RCLCPP_INFO(node_->get_logger(),"LOADED PLUGINS");

  }

  void TreeWrapper::loadPluginsFromFolder()
  {
    bool import_from_folder = false;
    node_->get_parameter_or("import_from_folder",import_from_folder,false);

    if(import_from_folder)
    {
        RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS FROM FOLDER");
        std::string plugins_folder;
        if (!node_->get_parameter("plugins_folder",plugins_folder))
        {
            RCLCPP_WARN(node_->get_logger(),"Import from folder option is set, but folder param is missing");
        }
        else
        {
            using namespace boost::filesystem;

            if(!exists(plugins_folder))
            {
              RCLCPP_ERROR(node_->get_logger(),"Plugin folder %s does not exist.", plugins_folder.c_str());
              return;
            }

            auto directory_list = [&] { return boost::make_iterator_range(directory_iterator(plugins_folder), {}); };

            for(const auto& entry : directory_list())
            {
                if((!is_regular_file(entry) && !is_symlink(entry)) || entry.path().extension() != ".so") { continue; }

                try
                {
                  const auto& plugin_path = canonical(entry.path());

                  factory_.registerFromPlugin(plugin_path.string());
                  loaded_plugins_.emplace(plugin_path.string());
                  RCLCPP_INFO(node_->get_logger(),"Loaded plugin %s from folder %s", plugin_path.filename().string().c_str(), plugins_folder.c_str());
                }
                catch(const std::runtime_error& ex)
                {
                  RCLCPP_ERROR(node_->get_logger(),"Cannot load plugin %s from folder %s. Error: %s", entry.path().filename().string().c_str(), plugins_folder.c_str(), ex.what());
                }
            }
        }
        RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS FROM FOLDER OK");
    }
  }

  void TreeWrapper::loadPluginsFromROS(std::vector<std::string> ros_plugins_folders)
  {
      RCLCPP_INFO(node_->get_logger(),"LOADING PLUGINS FROM ROS");

      bt_server::Params bt_params;
      bt_params.ros_plugins_timeout = 1000;
      bt_params.plugins = ros_plugins_folders;
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
            /*const BT::Expected<std::string> bbentry_value_inferred_keyvalues = replaceKeysWithStringValues(bb_val, global_blackboard_, true); // no effect if it has no key
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
            global_blackboard_->set(bb_key, bb_val);
            
            //ADD VALUE TO SYNC MAP
            if (sync_entry)
            {
              syncMap_.emplace(bb_key,std::make_pair(SyncStatus::TO_SYNC,global_blackboard_->getEntry(bb_key)));
            }
        }
        RCLCPP_INFO(node_->get_logger(),"Initialized BB with %ld entries from YAML file %s", global_blackboard_->getKeys().size(), abs_file_path.c_str());
    }
    catch(const YAML::Exception& ex) 
    { 
        RCLCPP_ERROR(node_->get_logger(),"Initializing. BB key from file '%s' did not succeed: %s", abs_file_path.c_str(), ex.what());
    }
  }
  void TreeWrapper::updateSyncStatus()
  {
    //SEARCH ON THE BLACKBOARD FOR NEW SYNC PORTS UPDATED
    std::unordered_set<std::string> syncKeys = getSyncKeysList();

    for (auto key : syncKeys)
    {
      auto entry = global_blackboard_->getEntry(key);
      if (entry->stamp > last_sync_update_)
      {
        syncMap_[key] = std::make_pair(SyncStatus::TO_SYNC,entry);
      }
    }

    //Update sync timestamp
    last_sync_update_ = std::chrono::steady_clock::now().time_since_epoch();
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
      tree_ptr_ = std::make_shared<BT::Tree> (factory_.createTreeFromText(tree_xml,global_blackboard_));
    }
    else
    {
        throw BT::RuntimeError("Failed to open the file: "+full_path );
    }

    //Add the syncKeys to the syncMap
    for (auto key: sync_keys)
    {
      syncMap_.emplace(key,std::make_pair(SyncStatus::TO_SYNC,global_blackboard_->getEntry(key)));
    }

    /*
    for (auto key: syncMap_)
    {
      RCLCPP_INFO(node_->get_logger(),"SyncMap_ : %s", key.first.c_str());
    }*/

    //Create debug_tree_ptr
    RCLCPP_INFO(node_->get_logger(),"Init debug_tree_ptr");
    debug_tree_ptr = std::make_shared<BT::DebuggableTree>(tree_ptr_,tree_debug);

  }

  void TreeWrapper::setTreeLoaded(bool loaded)
  { 
    is_tree_loaded_ = loaded;
    if (loaded)
    {
      start_execution_time_ = node_->get_clock()->now();
      last_sync_update_ = std::chrono::steady_clock::now().time_since_epoch();
    }
  }

  SyncMap TreeWrapper::getKeysValueToSync ()
  {
    //RCLCPP_INFO(node_->get_logger()," Getting Keys Value to Sync"); 
    SyncMap ports_to_be_sync_cpy;
    for(auto element : syncMap_)
    {
      if(element.second.first == SyncStatus::TO_SYNC)
      {
        element.second.first = SyncStatus::SYNCING;
        updateSyncMap(element.first,element.second);
        ports_to_be_sync_cpy.emplace(element);
      }
    }
    //RCLCPP_INFO(node_->get_logger()," Getting Keys Value to Sync OK"); 
    return ports_to_be_sync_cpy;
  }

  std::unordered_set<std::string> TreeWrapper::getSyncKeysList()
  {
    std::unordered_set<std::string> keys;
    {
      for (auto element : syncMap_)
      {
        keys.insert(element.first);
      }
    }
    return keys;
  }
}
