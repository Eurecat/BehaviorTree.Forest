
#include "behaviortree_forest/behaviortree_server.hpp"

#include "behaviortree_ros2/bt_utils.hpp"
#include "behaviortree_forest/params/behaviortree_server_params.hpp"
#include "behaviortree_eut_plugins/utils/eut_utils.h"

#include "behaviortree_forest_interfaces/srv/tree_restart_advanced.hpp"

#include "behaviortree_forest/sync_manager.hpp"

namespace BT_SERVER
{

  using TreeRestartAdvancedSrv = behaviortree_forest_interfaces::srv::TreeRestartAdvanced;

  BehaviorTreeServer::BehaviorTreeServer(const rclcpp::Node::SharedPtr& node) : 
    node_(node), 
    eut_bt_factory_(BT::EutBehaviorTreeFactory(std::make_shared<BT::BehaviorTreeFactory>())),
    ros2_launch_manager_(node)
  {
    //Add nh to executor
    executor_.add_node(node_);


    const auto ros_plugin_directories = getBTPluginsFolders(); // pkgname/bt_plugins

    bt_server::Params bt_params;
    bt_params.ros_plugins_timeout = 16000; //TODO shall be decrease: for now will just help to understand what is happening
    bt_params.plugins = ros_plugin_directories;

    RegisterPlugins(bt_params, eut_bt_factory_.originalFactory(), node_);
    eut_bt_factory_.updateTypeInfoMap();
    
    //Create Services:
    load_tree_srv_ = node_->create_service<LoadTreeSrv>("behavior_tree_forest/load_tree",std::bind(&BehaviorTreeServer::loadTreeCB,this,_1,_2));
    stop_tree_srv_ = node_->create_service<TreeRequestSrv>("behavior_tree_forest/stop_tree",std::bind(&BehaviorTreeServer::stopTreeCB,this,_1,_2));
    kill_tree_srv_ = node_->create_service<TreeRequestSrv>("behavior_tree_forest/kill_tree",std::bind(&BehaviorTreeServer::killTreeCB,this,_1,_2));
    kill_all_trees_srv_ = node_->create_service<EmptySrv>("behavior_tree_forest/kill_all_trees",std::bind(&BehaviorTreeServer::killAllTreesCB,this,_1,_2));
    restart_tree_srv_  = node_->create_service<TreeRequestSrv>("behavior_tree_forest/restart_tree",std::bind(&BehaviorTreeServer::restartTreeCB,this,_1,_2));
    restart_tree_by_capability_srv_  = node_->create_service<TreeRequestByCapabilitySrv>("behavior_tree_forest/restart_trees_by_capability",std::bind(&BehaviorTreeServer::restartTreeByCapabilityCB,this,_1,_2));
    pause_tree_srv_ = node_->create_service<TreeRequestSrv>("behavior_tree_forest/pause_tree",std::bind(&BehaviorTreeServer::pauseTreeCB,this,_1,_2));
    resume_tree_srv_ = node_->create_service<TreeRequestSrv>("behavior_tree_forest/resume_tree",std::bind(&BehaviorTreeServer::resumeTreeCB,this,_1,_2));    
    get_sync_bb_values_srv_ = node_->create_service<GetBBValuesSrv>("behavior_tree_forest/get_sync_bb_values",std::bind(&BehaviorTreeServer::getSyncBBValuesCB,this,_1,_2));
    get_tree_status_srv_ = node_->create_service<GetTreeStatusSrv>("behavior_tree_forest/get_tree_status",std::bind(&BehaviorTreeServer::getTreeStatusCB,this,_1,_2));
    get_all_trees_status_srv_ = node_->create_service<GetAllTreeStatusSrv>("behavior_tree_forest/get_all_trees_status",std::bind(&BehaviorTreeServer::getAllTreeStatusCB,this,_1,_2));
    
    trees_upd_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    //Create Sync_BB and Init
    node_->declare_parameter(PARAM_NAME_SYNC_BB_INIT, "");
    std::string sync_bb_init_file =node_->get_parameter(PARAM_NAME_SYNC_BB_INIT).as_string();
    sync_blackboard_ptr_ = BT::Blackboard::create();
    if(sync_bb_init_file.length() > 0) { initBB(sync_bb_init_file, sync_blackboard_ptr_); }

    //Updates subscriber server side
    sync_bb_sub_ = node_->create_subscription<BBEntries>("behavior_tree_forest/local_update", 10, std::bind(&BehaviorTreeServer::syncBBCB, this, _1)) ;
    
    //Updates republisher for all trees (put latch to true atm, because seems a good option that you receive last update from the server)
    sync_bb_pub_ = node_->create_publisher<BBEntries>("behavior_tree_forest/broadcast_update", 10);


    RCLCPP_INFO(node_->get_logger(), "BehaviorTreeServer with name %s up and running", this->node_->get_name());
  }
  
  BehaviorTreeServer::~BehaviorTreeServer() 
  {
    killAllTrees(true);//force kill them all
  }
  void BehaviorTreeServer::syncBBCB(const BBEntries::SharedPtr msg) const
  {
    BBEntries msg_republish_entries;
    for(const auto& entry : msg->entries)
    {
      // Return boolean and affect republish below
      if(syncBB(entry))
        msg_republish_entries.entries.push_back(entry);
    }
    republishUpdatedSyncEntries(msg_republish_entries);
  }
  bool BehaviorTreeServer::syncBB(const BBEntry& msg) const
  {
    // std::string remote_entry_type_str = msg.type.rfind("nlohmann::json", 0) == std::string::npos ? msg.type : "json" ;
    RCLCPP_DEBUG(node_->get_logger(), "RX Sync Update from [%s]. Key: '%s' Type: '%s' Value: '%s'", msg.bt_id.c_str(), msg.key.c_str(), BT::polishedTypeName(msg.type).c_str(), msg.value.c_str());

    if(sync_blackboard_ptr_)
    {
      // CASE A: Remote Entry !StronglyTyped & Local Entry StronglyTyped
      // CASE B: Remote Entry !StronglyTyped & Local Entry !StronglyTyped
      // CASE C: Remote Entry StronglyTyped & Local Entry !StronglyTyped
      // CASE D: Remote Entry StronglyTyped & Local Entry StronglyTyped
                
      auto entry_ptr = sync_blackboard_ptr_->getEntry(msg.key);

      if(!entry_ptr || !entry_ptr->info.isStronglyTyped() /*&& BT::isStronglyTyped(msg.type)*/) // entry does not exist or existed with no type and now I know type
      {
        // CASE B, C
        sync_blackboard_ptr_->unset(msg.key);
        BT::TypeInfo type_info = eut_bt_factory_.getTypeInfo(msg.type).value_or(BT::TypeInfo()); // fallback to AnyTypeAllowed
        if(type_info.isStronglyTyped())
        {
          // CASE C
          sync_blackboard_ptr_->createEntry(msg.key, type_info);
          entry_ptr = sync_blackboard_ptr_->getEntry(msg.key); // update entry ptr
        }
      }

      // type safety checks
      if(entry_ptr)
      { 
        if(BT::isStronglyTyped(msg.type) && entry_ptr->info.isStronglyTyped() && msg.type != entry_ptr->info.typeName())
        {
          if((entry_ptr->value.isNumber() || entry_ptr->value.isType<bool>()) && BT::isNumberType(msg.type))
          {
            // will be treated later within the library in the blackboard->set(...) call and might be still acceptable
          }
          else
          {
            // std::string current_entry_type_str = entry_ptr->info.typeName().rfind("nlohmann::json", 0) == std::string::npos ? entry_ptr->info.typeName() : "json" ;
      
            // unacceptable inconsistency
            RCLCPP_WARN(node_->get_logger(),"Failed to update sync port in SERVER BB for key [%s] with value [%s] : Type inconsistency type %s, but received %s", 
              msg.key.c_str(), msg.value.c_str(),
              BT::polishedTypeName(entry_ptr->info.typeName()).c_str(), BT::polishedTypeName(msg.type).c_str());
            return false;
          }
        }
      }

      try
      {

        BT::JsonExporter::ExpectedEntry expected_entry = {};
        if(!entry_ptr || !BT::isStronglyTyped(msg.type))
        {
          // CASE A, B
          if(!msg.value.empty()) sync_blackboard_ptr_->set(msg.key, msg.value); // no type and stringified value both side
          else return false; // no type, no value
        }
        else
        {
          // CASE D (case C directly treated above)
          const nlohmann::json json_value = nlohmann::json::parse(msg.value);
          RCLCPP_DEBUG(node_->get_logger(),"Sync port in SERVER BB for key [%s] with value parsed to json [%s] : type of parsed json [%s]", msg.key.c_str(), json_value.dump().c_str(), json_value.type_name());
          
          const BT::JsonExporter::ExpectedEntry expected_entry = BT::EutUtils::eutFromJson(json_value, entry_ptr->info.type());
          if(expected_entry.has_value())
          {
            sync_blackboard_ptr_->set(msg.key, std::move(expected_entry.value().first));
            RCLCPP_DEBUG(node_->get_logger(),"Synced port in SERVER BB for key [%s] with entry parsed from json [%s] : type [%s][%s]", msg.key.c_str(), json_value.dump().c_str(), 
              BT::demangle(expected_entry.value().first.type()).c_str(), expected_entry.value().second.typeName().c_str());
          }
        }
        
      }
      catch(const std::exception& e)
      {
        RCLCPP_ERROR(node_->get_logger(),"Failed to update sync port in SERVER BB for key [%s] with value [%s] : %s", msg.key.c_str(), msg.value.c_str(), e.what());
        return false;
      }
    }

    // //Check if input msg type is Void
    // const bool void_type = (msg->type == BT::demangle(typeid(void)));
    // if (void_type)
    // {
    //   //Skip Sync Entry as it still has no type declared
    //   return;
    // }

    // //Get the Entry & TypeInfo from the server BB 
    // const BT::Blackboard::Entry* entry_bb_server = sync_blackboard_ptr_->getEntry(msg->key).get();
    // const BT::TypeInfo* type_info_bb_server = sync_blackboard_ptr_->entryInfo(msg->key);

    // //Check if the Entry or Type info exist on the BB
    // if(entry_bb_server == nullptr || type_info_bb_server == nullptr)
    // {
    //   // Entry not present in the BB -> Create & insert
    //   BT::TypeInfo new_type_info = createTypeInfoFromType(msg->type);
    //   sync_blackboard_ptr_->createEntry(msg->key, new_type_info);
    //   RCLCPP_INFO(node_->get_logger(),"Entry in Sync BB for key [%s] Initialized with type [%s]", msg->key.c_str(), new_type_info.typeName().c_str());
    // }

    // //Retrieve current entry in bt server bb
    // auto entry_ptr = sync_blackboard_ptr_->getEntry(msg->key);
    // auto type_info = sync_blackboard_ptr_->entryInfo(msg->key);
    // if(!entry_ptr || !type_info)
    // {
    //   // Error on BB Entry!
    //   RCLCPP_ERROR(node_->get_logger(),"Entry/TypeInfo should exist on the Server BlackBoard!");
    //   return;
    // }

    // if(msg->type != type_info->typeName())
    // {
    //   // Type inconsistencies, don't update
    //   RCLCPP_ERROR(node_->get_logger(),"Sync Entry Type Msmatch for key [%s] - BB_type:[%s] RX_type:[%s]", msg->key.c_str(), type_info->typeName().c_str(), msg->type.c_str());
    //   return; 
    // }

    // try
    // {
    //   // convert from string new value and insert on the BB
    //   BT::Any new_any_value = type_info->parseString(msg->value); //If No Value set yet, it will not update the Entry and prompt de Warning
    //   sync_blackboard_ptr_->set(msg->key, std::move(new_any_value));
    // }
    // catch(const std::exception& e)
    // {
    //     RCLCPP_WARN(node_->get_logger(),"Failed to update sync port in SERVER BB for key [%s] with value [%s] : %s", msg->key.c_str(), msg->value.c_str(), e.what());
    //     return;
    // }

    //Republish message to all BT_NODES
    /*BBEntry upd_msg;
    upd_msg.key = msg.key;
    upd_msg.type = sync_blackboard_ptr_->getEntry(msg.key)->info.typeName() ;
    upd_msg.value = msg.value;
    upd_msg.bt_id = msg.bt_id;
    upd_msg.sender_sequence_id = msg.sender_sequence_id;
    sync_bb_pub_->publish(upd_msg);*/
    RCLCPP_DEBUG(node_->get_logger(), "Sync Port %s Updated on BT_Server BB succesfully",msg.key.c_str());
    return true;
  }

  void BehaviorTreeServer::republishUpdatedSyncEntries(const BBEntries& msg) const
  {
    if(msg.entries.empty())
    {
      RCLCPP_DEBUG(node_->get_logger(), "No Sync Entries to republish");
      return;
    }
    
    BBEntries entries;
    for (const auto& entry : msg.entries)
    {
      BT::Expected<BBEntry> entry_opt = SyncManager::buildBBEntryMsg(entry.key, sync_blackboard_ptr_);
      if(entry_opt)
      {
        BBEntry& upd_msg = entry_opt.value();
        upd_msg.bt_id = entry.bt_id;
        entries.entries.push_back(upd_msg);
      }
    }
    sync_bb_pub_->publish(entries);
  }

  bool BehaviorTreeServer::loadTreeCB(const std::shared_ptr<LoadTreeSrv::Request> req, std::shared_ptr<LoadTreeSrv::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(), "Loading new Tree file Request: '%s' ", req->tree_file.c_str());

    //1. Extract Tree Name
    std::string tree_name;
    std::string tree_filename_tmp = req->tree_file;
    std::size_t found1 = tree_filename_tmp.find_last_of("/");
    std::size_t found2 = tree_filename_tmp.find(".xml");
    if((found1 != std::string::npos) && (found2 != std::string::npos))
    {
      tree_name = tree_filename_tmp.substr ( (found1+1) , (tree_filename_tmp.size()-5-found1) );
    }
    else if(found2 != std::string::npos)
    {
      tree_name = tree_filename_tmp.substr (0,(tree_filename_tmp.size()-5));
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(),"Tree file name not valid %s",tree_filename_tmp.c_str());
      return false;
    }

    //2. Remove invalid characters on the TreeName
    const std::string characters_allowed = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890/_";
    auto new_end = std::remove_if(tree_name.begin(), tree_name.end(),
                                    [characters_allowed](std::string::value_type c)
                                    { return characters_allowed.find(c) == std::string::npos; });
    tree_name.erase(new_end, tree_name.end());

    //3. Force Unique Tree Name
    std::string tree_name_tmp = tree_name;
    int cnt = 1;
    bool tree_name_found = true;
    while (tree_name_found)
    {
        tree_name_found = false;
        for (const auto& tree_info : uids_to_tree_info_)
        {
            if (tree_info.second.tree_name == tree_name_tmp)
            {
                //Add number to differentiate names
                tree_name_found = true;
                tree_name_tmp = tree_name + std::to_string(cnt);
                cnt++;
            }
        }
    }
    tree_name = tree_name_tmp;

    RCLCPP_INFO(node_->get_logger(), "Loading Tree: %s",tree_name.c_str());

    trees_UID_++;
    std::string param_node_name = "__node:="+tree_name;
    std::string param_name = "tree_name:="+tree_name;
    std::string param_file = "tree_file:="+req->tree_file;
    std::string param_uid = "tree_uid:="+std::to_string(trees_UID_);
    std::string param_auto_restart = "tree_auto_restart:="+BT::toStr(req->auto_restart);
    std::string param_debug ="tree_debug:="+BT::toStr(req->debug);

    std::string param_bb_init = "";
    if (req->bb_init_files.size()> 0)
    {
        param_bb_init += "bb_init:=[";
        for (long unsigned int i = 0; i < req->bb_init_files.size(); i++)
        {
            param_bb_init += "\'" + req->bb_init_files[i] + "\'";
            if (i != (req->bb_init_files.size()-1))
                  param_bb_init += ",";
        }
        param_bb_init += "]";
    }

    fprintf(stderr, "bb_init c_str %s\n", param_bb_init.c_str());

    //Set default port IDs
    int server_port = 1667;
    int publisher_port = 1666;

    //Get port parameters
    if (req->server_port > 0)
        server_port = req->server_port;
    if (req->publisher_port > 0)
        publisher_port = req->publisher_port;

    std::string param_server_port = "server_port:="+std::to_string(server_port);  
    std::string param_pub_port ="publisher_port:="+std::to_string(publisher_port);
      
    pid_t pid;

    //CREATE ROS2 RUN MANAGER
    const char* package_name = "behaviortree_forest"; 
    const char* executable_name = "behaviortree_node";

    res->tree_uid = trees_UID_;
    res->started = false;
    res->instantiated = false;
    try {
      if(param_bb_init.empty())
        pid = ros2_launch_manager_.start(
          package_name,
          executable_name,
          "--ros-args",
          "-r", param_node_name.c_str(),
          "-p", param_name.c_str(),
          "-p", param_file.c_str(),
          "-p", param_uid.c_str(),
          "-p", param_debug.c_str(),
          "-p", param_auto_restart.c_str(),
          // "-p", param_bb_init.c_str(), // bug to be understand in how ros2 parse this empty vector of string when it's passed such as []
          "-p", param_server_port.c_str(),
          "-p", param_pub_port.c_str()
          );
      else
        pid = ros2_launch_manager_.start(
              package_name,
              executable_name,
              "--ros-args",
              "-r", param_node_name.c_str(),
              "-p", param_name.c_str(),
              "-p", param_file.c_str(),
              "-p", param_uid.c_str(),
              "-p", param_debug.c_str(),
              "-p", param_auto_restart.c_str(),
              "-p", param_bb_init.c_str(),
              "-p", param_server_port.c_str(),
              "-p", param_pub_port.c_str()
              );
    }
    catch (std::exception const &exception) {
      RCLCPP_ERROR(node_->get_logger(),"Error using ros2 run manager: %s", exception.what());
      return false;
    }
    
    res->started = true;

    // monitor the tree process status
    auto [it, inserted] = uids_to_tree_info_.try_emplace(res->tree_uid, tree_name, pid, TreeStatus::LOADING);
    TreeProcessInfo& new_process_info = it->second;

    std::string topic_name = "/"+tree_name+"/execution_status";
    rclcpp::SubscriptionOptions trees_upd_sub_options;
    trees_upd_sub_options.callback_group = trees_upd_cb_group_;
    // register sub with latch last msg
    new_process_info.status_subscriber =  node_->create_subscription<TreeStatus>(topic_name, 10, std::bind(&BehaviorTreeServer::treeStatusTopicCB, this, _1), trees_upd_sub_options);
    
    // Wait for tree to be loaded (check first execution status msg at  uids_to_tree_info_.at(msg->uid).tree_status) 
    // Check with a condition variable if a certain max_time is set greater than 0, otherwise just say is started without waiting...

    // Wait for tree to be loaded (checking first execution status message)
    const int max_wait_time_ms = (req->max_load_timeout_ms > 0) ? req->max_load_timeout_ms : 5000;  // 5 seconds maximum wait time as default one

    if (max_wait_time_ms > 0) 
    {
      RCLCPP_INFO(node_->get_logger(), "Waiting up to %d ms for tree %s to load...", 
                max_wait_time_ms, tree_name.c_str());
    
      std::unique_lock<std::mutex> lock(new_process_info.loading_mutex);
      bool success = new_process_info.loading_cv.wait_for(lock, std::chrono::milliseconds(max_wait_time_ms),
                                                            [&new_process_info]() { 
                                                              return new_process_info.tree_status.status != TreeStatus::LOADING; 
                                                            });
    
      if (success) 
      {
        RCLCPP_INFO(node_->get_logger(), "Tree %s loaded successfully with UID %u", 
                    tree_name.c_str(), res->tree_uid);
        res->instantiated = true;
      } 
      else 
      {
        RCLCPP_WARN(node_->get_logger(), "Timeout waiting for tree %s to load, but process started with UID %u", 
                    tree_name.c_str(), res->tree_uid);
      }
    } 
    else 
    {
      RCLCPP_INFO(node_->get_logger(), "Tree %s process started with UID %u (not waiting for load confirmation)", 
                  tree_name.c_str(), res->tree_uid);
    }

    if(!req->deployed_capability.empty()) // if capability is not empty, add to the map the capability -> uid map (remember that a capability can be deployed through different trees running in parallel)
      capability_to_uids_map_[req->deployed_capability].insert(res->tree_uid);

    return true;
  }

  bool BehaviorTreeServer::handleCallEmptySrv(const std::string& service_name)
  {
    const auto response = handleSyncSrvCall<EmptySrv>(service_name);

    return response.second == rclcpp::FutureReturnCode::SUCCESS; // Indicate that the service was received
  }

  bool BehaviorTreeServer::handleCallTriggerSrv(const std::string& service_name)
  {
    const auto response = handleSyncSrvCall<TriggerSrv>(service_name);
    bool success = response.second == rclcpp::FutureReturnCode::SUCCESS && response.first->success; // Indicate that the service was received and processed successfully
    return success; 
  }

  bool BehaviorTreeServer::stopTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Stopping tree with UID: '%u' ", req->tree_uid);
    if(uids_to_tree_info_.find(req->tree_uid) != uids_to_tree_info_.end())
    {
        const TreeProcessInfo& tree_info = uids_to_tree_info_.at(req->tree_uid);
        bool result = handleCallEmptySrv("/"+tree_info.tree_name+ "/stop_tree");
        RCLCPP_INFO(node_->get_logger(), "Stopping tree with UID: '%u' done ", req->tree_uid);
        res->success = result;
        res->details = "Stopping tree with UID: "+std::to_string(req->tree_uid)+" done";
        return result;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to Stop tree: UID %u does not exist", req->tree_uid);
      res->details = "Stopping tree with UID: " + std::to_string(req->tree_uid) + " failed";
    }
    return true;
  }

  bool BehaviorTreeServer::killTree(const uint32_t tree_uid, const bool force_kill)
  {
    if(uids_to_tree_info_.find(tree_uid) != uids_to_tree_info_.end())
    {
        TreeProcessInfo& tree_info = uids_to_tree_info_.at(tree_uid);
        if (!tree_info.killed && handleCallEmptySrv("/"+tree_info.tree_name+ "/kill_tree") || force_kill)
        {
          if(force_kill)
            RCLCPP_INFO(node_->get_logger(), "Force killing tree with uid %d", tree_uid);

          //Extract extra PIDs created when executing "ros2 run ..." with fork()
          pid_t bt_node_pid = ros2_launch_manager_.extract_bt_node_pid_from_python_pid(tree_info.pid);
          std::string command;
          if (bt_node_pid!=0) 
            command = "kill -9 "+ std::to_string(bt_node_pid) + " ; " + "kill -9 "+ std::to_string(tree_info.pid);
          else
            command = "kill -9 "+ std::to_string(tree_info.pid);

          //RCLCPP_INFO(node_->get_logger(), "EXECUTING KILL COMMAND: %s", command.c_str());
          int resultcmd = system(command.c_str());
          tree_info.killed = true;
          // if (resultcmd != -1)
          //   RCLCPP_INFO(node_->get_logger(), "Succesfully killed processes with PIDs: %s & %s",std::to_string(tree_info.pid).c_str(), std::to_string(bt_node_pid).c_str());
          // else
          //   RCLCPP_ERROR(node_->get_logger(), "Failed to kill process with PIDs %s & %s",std::to_string(tree_info.pid).c_str(), std::to_string(bt_node_pid).c_str());

          // remove it from the capability_to_uids_map_
          for(auto& it : capability_to_uids_map_)
          {
            auto& uids = it.second;
            uids.erase(tree_uid);
          }
          
          return resultcmd != -1;
        }
    }
    RCLCPP_ERROR(node_->get_logger(), "Failed to kill process: UID %u does not exist", tree_uid);
    return false;
  }

  bool BehaviorTreeServer::killAllTrees(const bool force_kill)
  {
    for (auto& tree_info : uids_to_tree_info_)
    {
      if(!tree_info.second.killed)
        killTree(tree_info.first, force_kill);
    }
    return true;
  }
  
  bool BehaviorTreeServer::killTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(), "Killing tree with UID: '%u' ", req->tree_uid);
    bool success = killTree(req->tree_uid, false);
    res->success = success;
    if(success)
      res->details = "Killing tree with UID: "+std::to_string(req->tree_uid)+" done";
    else
      res->details = "Killing tree with UID: "+std::to_string(req->tree_uid)+" failed";
    return true;
  }

  bool BehaviorTreeServer::killAllTreesCB(const std::shared_ptr<EmptySrv::Request> req, std::shared_ptr<EmptySrv::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(), "Killing all trees");
    killAllTrees();
    return true;
  }


  bool BehaviorTreeServer::restartTreeByCapabilityCB(const std::shared_ptr<TreeRequestByCapabilitySrv::Request> req, std::shared_ptr<TreeRequestByCapabilitySrv::Response> res)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Restarting Trees with capability: '%s' ", req->capability.c_str());
    if(capability_to_uids_map_.find(req->capability) != capability_to_uids_map_.end())
    {
      const auto& uids = capability_to_uids_map_.at(req->capability);
      for (const auto& uid : uids)
      {
        const TreeProcessInfo& tree_info = uids_to_tree_info_.at(uid);
        // req->reset_entries;
        TreeRestartAdvancedSrv::Request restart_advanced_req;
        restart_advanced_req.reset_entries = req->reset_entries;
        const auto response = handleSyncSrvCall<TreeRestartAdvancedSrv>("/"+tree_info.tree_name+ "/restart_tree_advanced", restart_advanced_req);
        res->success = response.second == rclcpp::FutureReturnCode::SUCCESS && response.first && response.first->success; // Indicate that the service was received and successfully processed;
        if(res->success)
        {
          RCLCPP_INFO(node_->get_logger(), "Restarted tree with UID: '%u' done ", uid);
          res->details += "Restarted tree with UID: "+std::to_string(uid)+" done, ";
        }
        else
        {
          // std::cout << "Failed to Restart with UID: '%u'. Service could not process the request nullptr? " << (response.first == nullptr) << " success? " << std::to_string(response.first && response.first->success) << uid;
          RCLCPP_ERROR(node_->get_logger(), "Failed to Restart with UID: '%u'. Service could not process the request", uid);
          res->details += "Failed to Restart with UID: "+std::to_string(uid)+". Service could not process the request, ";
        }
      }
      return true;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to restart tree: Capability %s does not exist", req->capability.c_str());
      res->success = false;
      res->details = "Failed to Restart tree: Capability "+req->capability+" does not exist";
    }
    return true;
  }

  bool BehaviorTreeServer::restartTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Restarting Tree with UID: '%u' ", req->tree_uid);
    if(uids_to_tree_info_.find(req->tree_uid) != uids_to_tree_info_.end())
    {
      const TreeProcessInfo& tree_info = uids_to_tree_info_.at(req->tree_uid);
      bool success = handleCallEmptySrv("/"+tree_info.tree_name+ "/restart_tree");
      res->success = success;
      if(success)
      {
        RCLCPP_INFO(node_->get_logger(), "Restarted tree with UID: '%u' done ", req->tree_uid);
        res->details = "Restarted tree with UID: "+std::to_string(req->tree_uid)+" done";
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to Restart with UID: '%u'. Service could not process the request", req->tree_uid);
        res->details = "Failed to Restart with UID: "+std::to_string(req->tree_uid)+". Service could not process the request";
      }
      return success;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to restart tree: UID %u does not exist", req->tree_uid);
      res->success = false;
      res->details = "Failed to Restart tree: UID "+std::to_string(req->tree_uid)+" does not exist";
    }
    return true;
  }

  bool BehaviorTreeServer::pauseTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Pausing tree with UID: '%u' ", req->tree_uid);
    if(uids_to_tree_info_.find(req->tree_uid) != uids_to_tree_info_.end())
    {
      const TreeProcessInfo& tree_info = uids_to_tree_info_.at(req->tree_uid);
      if(handleCallTriggerSrv("/"+tree_info.tree_name+ "/pause_tree"))
      {
        RCLCPP_INFO(node_->get_logger(), "Paused tree with UID: '%u' done ", req->tree_uid);
        res->success = true;
        res->details = "Paused tree with UID: "+std::to_string(req->tree_uid)+" done";
        return true;
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to Pause with UID: '%u'. Service could not process the request", req->tree_uid);
        res->success = false;
        res->details = "Failed to Pause with UID: "+std::to_string(req->tree_uid)+". Service could not process the request";
      }
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to Pause tree: UID %u does not exist", req->tree_uid);
      res->success = false;
      res->details = "Failed to Pause tree: UID "+std::to_string(req->tree_uid)+" does not exist";
    }
    return true;
  }

  bool BehaviorTreeServer::resumeTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Resuming tree with UID: '%u' ", req->tree_uid);
    if(uids_to_tree_info_.find(req->tree_uid) != uids_to_tree_info_.end())
    {
      const TreeProcessInfo& tree_info = uids_to_tree_info_.at(req->tree_uid);
      if(handleCallTriggerSrv("/"+tree_info.tree_name+ "/resume_tree"))
      {
        RCLCPP_INFO(node_->get_logger(), "Resumed tree with UID: '%u' done ", req->tree_uid);
        res->success = true;
        res->details = "Resumed tree with UID: "+std::to_string(req->tree_uid)+" done";
        return true;
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to Resumed with UID: '%u'. Service could not process the request", req->tree_uid);
        res->success = false;
        res->details = "Failed to Resume with UID: "+std::to_string(req->tree_uid)+". Service could not process the request";
      }
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to Resume tree: UID %u does not exist", req->tree_uid);
      res->success = false;
      res->details = "Failed to Resume with UID: "+std::to_string(req->tree_uid)+". UID does not exist";
    }
    return true;
  }

  bool BehaviorTreeServer::getSyncBBValuesCB (const std::shared_ptr<GetBBValuesSrv::Request> req, std::shared_ptr<GetBBValuesSrv::Response> res)
  {
    for (auto key : req->keys)
    {
      BT::Expected<BBEntry> bb_entry_opt = SyncManager::buildBBEntryMsg(key, sync_blackboard_ptr_);
      if(bb_entry_opt)
      {
          BBEntry& bb_entry = bb_entry_opt.value();
          bb_entry.bt_id = ""; //Empty BT_ID for BTServer requests
          res->entries.push_back(bb_entry);
      }
      else
      {
        RCLCPP_WARN(node_->get_logger(), "Error fetching PortValue: %s  for port %s ", bb_entry_opt.error().c_str(), key.c_str());
      }
    }
    return true;
  }
  
  bool BehaviorTreeServer::getTreeStatusCB(const std::shared_ptr<GetTreeStatusSrv::Request> req, std::shared_ptr<GetTreeStatusSrv::Response> res)
  {
    // Get Saved Status updated by ROS topic
    if (uids_to_tree_info_.find(req->tree_uid) != uids_to_tree_info_.end())
    {
      res->status = uids_to_tree_info_.at(req->tree_uid).tree_status;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get status tree: UID %u does not exist", req->tree_uid);
      return false;
    }
    return true;
  }
  
  bool BehaviorTreeServer::getAllTreeStatusCB(const std::shared_ptr<GetAllTreeStatusSrv::Request> req, std::shared_ptr<GetAllTreeStatusSrv::Response> res)
  {
    for (const auto& tree_info : uids_to_tree_info_)
    {
      res->status.push_back(tree_info.second.tree_status);
    }
    return true;
  }

  void BehaviorTreeServer::treeStatusTopicCB(const TreeStatus::SharedPtr msg)   
  {
    auto& tree_info = uids_to_tree_info_.at(msg->uid);

    // If the tree was loading and now has a different status, it means it's loaded
    bool tree_loaded_now = (tree_info.tree_status.status == TreeStatus::LOADING && msg->status != TreeStatus::LOADING);
    bool alert_tree_status_upd = tree_loaded_now; // can add others ucses later

    // Update tree info 
    uids_to_tree_info_.at(msg->uid).tree_status = *msg;
    
    if(alert_tree_status_upd)
    {
      RCLCPP_INFO(node_->get_logger(), "Tree %s with UID %u loaded", tree_info.tree_name.c_str(), msg->uid);
      // Notify the condition variable to wake up any waiting threads
      std::unique_lock<std::mutex> lock(tree_info.loading_mutex);
      tree_info.loading_cv.notify_all();
    }
  }

  void BehaviorTreeServer::initBB(const std::string& abs_file_path, BT::Blackboard::Ptr blackboard_ptr)
  {
    try 
    {
        RCLCPP_DEBUG(node_->get_logger(), "Initializing BB from YAML file %s", abs_file_path.c_str());
        YAML::Node config = YAML::LoadFile(abs_file_path);
        for(YAML::const_iterator it=config.begin();it!=config.end();++it)
        {
            const std::string& bb_key = it->first.as<std::string>();
            std::string bb_val = it->second.as<std::string>();
            
            // TODO Devis inferred key values within other values to be developed still... 
            // const BT::Expected<std::string> bbentry_value_inferred_keyvalues = replaceKeysWithStringValues(bb_val,blackboard_ptr); // no effect if it has no key
            // if(!bbentry_value_inferred_keyvalues)
            // {
            //     // but will complain if it has a reference to a wrong key
            //     RCLCPP_WARN(node_->get_logger(), "Init. of BB key %s for value %s, value inference did not succeed: %s", 
            //         bb_key.c_str(), 
            //         bb_val.c_str(),
            //         bbentry_value_inferred_keyvalues.error().c_str());
            //     continue; // and skip this init
            // }
            // else
            {
              // bb_val = bbentry_value_inferred_keyvalues.value();
              
              // use the string here and blackboard_ptr->set(...)
              blackboard_ptr->set(bb_key, bb_val);
              const auto entry_n = blackboard_ptr->getEntry(bb_key);
              // std::string entry_type_info_str = BT::demangle(entry_n->info.type()).rfind("nlohmann::json", 0) == std::string::npos ? BT::demangle(entry_n->info.type()) : "json";
              // std::string entry_type_valuetype_str = BT::demangle(entry_n->value.type()).rfind("nlohmann::json", 0) == std::string::npos ? BT::demangle(entry_n->value.type()) : "json";
              // std::string entry_type_casted_str = BT::demangle(entry_n->value.castedType()).rfind("nlohmann::json", 0) == std::string::npos ? BT::demangle(entry_n->value.castedType()) : "json";
              RCLCPP_DEBUG(node_->get_logger(), "Init. BB key [\"%s\"] with value \"%s\" et %s t %s, ct %s", 
              bb_key.c_str(), bb_val.c_str(),
              BT::polishedTypeName(BT::demangle(entry_n->info.type())).c_str(),
              BT::polishedTypeName(BT::demangle(entry_n->value.type())) .c_str(), 
              BT::polishedTypeName(BT::demangle(entry_n->value.castedType())).c_str());
        
            }
        }  
        RCLCPP_DEBUG(node_->get_logger(), "Initialized BB with %ld entries from YAML file %s", blackboard_ptr->getKeys().size(), abs_file_path.c_str());
    }
    catch(const YAML::Exception& ex) 
    { 
        RCLCPP_WARN(node_->get_logger(), "Initializing. BB key from file '%s' did not succeed: %s", abs_file_path.c_str(), ex.what());
    }
  }
  
  void BehaviorTreeServer::run()
  {
    executor_.spin();
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("behavior_tree_server");

  auto bt_server = std::make_shared<BT_SERVER::BehaviorTreeServer>(nh);

  bt_server->run();
  bt_server.reset();
  
  rclcpp::shutdown();

  return 0;
}