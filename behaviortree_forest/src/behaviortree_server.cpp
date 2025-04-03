
#include "behaviortree_forest/behaviortree_server.hpp"

#include "behaviortree_ros2/bt_utils.hpp"
#include "behaviortree_forest/params/behaviortree_server_params.hpp"
#include "behaviortree_eut_plugins/utils/eut_utils.h"

namespace BT_SERVER
{
  BehaviorTreeServer::BehaviorTreeServer(const rclcpp::Node::SharedPtr& node) : 
    node_(node), 
    eut_bt_factory_(BT::EutBehaviorTreeFactory(std::make_shared<BT::BehaviorTreeFactory>())),
    ros2_launch_manager_(node)
  {
    //Add nh to executor
    executor_.add_node(node_);


    const auto ros_plugin_directories = getBTPluginsFolders(); // pkgname/bt_plugins

    bt_server::Params bt_params;
    bt_params.ros_plugins_timeout = 5000;
    bt_params.plugins = ros_plugin_directories;

    RegisterPlugins(bt_params, eut_bt_factory_.originalFactory(), node_);
    eut_bt_factory_.updateTypeInfoMap();
    
    //Create Services:
    load_tree_srv_ = node_->create_service<LoadTreeSrv>("behavior_tree_forest/load_tree",std::bind(&BehaviorTreeServer::loadTreeCB,this,_1,_2));
    stop_tree_srv_ = node_->create_service<TreeRequestSrv>("behavior_tree_forest/stop_tree",std::bind(&BehaviorTreeServer::stopTreeCB,this,_1,_2));
    kill_tree_srv_ = node_->create_service<TreeRequestSrv>("behavior_tree_forest/kill_tree",std::bind(&BehaviorTreeServer::killTreeCB,this,_1,_2));
    kill_all_trees_srv_ = node_->create_service<EmptySrv>("behavior_tree_forest/kill_all_trees",std::bind(&BehaviorTreeServer::killAllTreesCB,this,_1,_2));
    restart_tree_srv_  = node_->create_service<TreeRequestSrv>("behavior_tree_forest/restart_tree",std::bind(&BehaviorTreeServer::restartTreeCB,this,_1,_2));
    pause_tree_srv_ = node_->create_service<TreeRequestSrv>("behavior_tree_forest/pause_tree",std::bind(&BehaviorTreeServer::pauseTreeCB,this,_1,_2));
    resume_tree_srv_ = node_->create_service<TreeRequestSrv>("behavior_tree_forest/resume_tree",std::bind(&BehaviorTreeServer::resumeTreeCB,this,_1,_2));    
    get_sync_bb_values_srv_ = node_->create_service<GetBBValuesSrv>("behavior_tree_forest/get_sync_bb_values",std::bind(&BehaviorTreeServer::getSyncBBValuesCB,this,_1,_2));
    get_tree_status_srv_ = node_->create_service<GetTreeStatusSrv>("behavior_tree_forest/get_tree_status",std::bind(&BehaviorTreeServer::getTreeStatusCB,this,_1,_2));
    get_all_trees_status_srv_ = node_->create_service<GetAllTreeStatusSrv>("behavior_tree_forest/get_all_trees_status",std::bind(&BehaviorTreeServer::getAllTreeStatusCB,this,_1,_2));
    
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
    RCLCPP_DEBUG(node_->get_logger(), "Received a Sync Update from %s BT. Key: '%s' Type: '%s' Value: '%s'", msg.bt_id.c_str(), msg.key.c_str(),msg.type.c_str(), msg.value.c_str());

    if(sync_blackboard_ptr_)
    {
      auto entry_ptr = sync_blackboard_ptr_->getEntry(msg.key);

      if(!entry_ptr || !entry_ptr->info.isStronglyTyped() && BT::isStronglyTyped(msg.type)) // entry does not exist or existed with no type and now I know type
      {
        sync_blackboard_ptr_->unset(msg.key);
        BT::TypeInfo type_info = eut_bt_factory_.getTypeInfo(msg.type).value_or(BT::TypeInfo()); // fallback to AnyTypeAllowed
        sync_blackboard_ptr_->createEntry(msg.key, type_info);
        entry_ptr = sync_blackboard_ptr_->getEntry(msg.key); // update entry ptr
      }

      // type safety checks
      if(entry_ptr)
      { 
        if(BT::isStronglyTyped(msg.type) && entry_ptr->info.isStronglyTyped() && msg.type != entry_ptr->info.typeName())
        {
          if(entry_ptr->value.isNumber() && BT::isNumberType(msg.type))
          {
            // will be treated later within the library in the blackboard->set(...) call and might be still acceptable
          }
          else
          {
            // unacceptable inconsistency
            RCLCPP_WARN(node_->get_logger(),"Failed to update sync port in SERVER BB for key [%s] with value [%s] : Type inconsistency type %s, but received %s", 
              msg.key.c_str(), msg.value.c_str(),
              msg.type.c_str(), entry_ptr->info.typeName().c_str());
            return false;
          }
        }
      }

      try
      {

        BT::JsonExporter::ExpectedEntry expected_entry = {};
        if(!entry_ptr || !BT::isStronglyTyped(msg.type))
        {
          if(!msg.value.empty()) sync_blackboard_ptr_->set(msg.key, msg.value); // no type and stringified value both side
          else return false; // no type, no value
        }
        else
        {
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
    BBEntries entries;
    for (const auto& entry : msg.entries)
    {
      BBEntry upd_msg;
      upd_msg.key = entry.key;
      upd_msg.type = sync_blackboard_ptr_->getEntry(entry.key)->info.typeName();
      upd_msg.value = entry.value;
      upd_msg.bt_id = entry.bt_id;
      upd_msg.sender_sequence_id = entry.sender_sequence_id;
      entries.entries.push_back(upd_msg);
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
        for (auto tree_info : uids_to_tree_info_)
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

    TreeProcessInfo new_process_info {tree_name,pid};
    std::string topic_name = "/"+tree_name+"/execution_status";
    new_process_info.status_subscriber =  node_->create_subscription<TreeStatus>(topic_name, 10, std::bind(&BehaviorTreeServer::treeStatusTopicCB, this, _1));
    uids_to_tree_info_.emplace(trees_UID_,new_process_info);
    res->tree_uid = trees_UID_;
  
    RCLCPP_INFO(node_->get_logger(),"Tree %s Loaded succesfully with UID %u", tree_name.c_str(), res->tree_uid);

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
        TreeProcessInfo tree_info = uids_to_tree_info_.at(req->tree_uid);
        bool result = handleCallEmptySrv("/"+tree_info.tree_name+ "/stop_tree");
        RCLCPP_INFO(node_->get_logger(), "Stopping tree with UID: '%u' done ", req->tree_uid);
        return result;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to Stop tree: UID %u does not exist", req->tree_uid);
    }
    return false;
  }

  bool BehaviorTreeServer::killTree(const uint8_t tree_uid, const bool force_kill)
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

          return resultcmd != -1;
        }
    }
    RCLCPP_ERROR(node_->get_logger(), "Failed to kill process: UID %u does not exist", tree_uid);
    return false;
  }

  bool BehaviorTreeServer::killAllTrees(const bool force_kill)
  {
    for (auto tree_info : uids_to_tree_info_)
    {
      if(!tree_info.second.killed)
        killTree(tree_info.first, force_kill);
    }
    return true;
  }
  
  bool BehaviorTreeServer::killTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(), "Killing tree with UID: '%u' ", req->tree_uid);
    return killTree(req->tree_uid, false);
  }

  bool BehaviorTreeServer::killAllTreesCB(const std::shared_ptr<EmptySrv::Request> req, std::shared_ptr<EmptySrv::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(), "Killing all trees");
    killAllTrees();
    return true;
  }

  bool BehaviorTreeServer::restartTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Restarting Tree with UID: '%u' ", req->tree_uid);
    if(uids_to_tree_info_.find(req->tree_uid) != uids_to_tree_info_.end())
    {
        TreeProcessInfo tree_info = uids_to_tree_info_.at(req->tree_uid);
        return handleCallEmptySrv("/"+tree_info.tree_name+ "/restart_tree");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to restart tree: UID %u does not exist", req->tree_uid);
    }
    return false;
  }

  bool BehaviorTreeServer::pauseTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Pausing tree with UID: '%u' ", req->tree_uid);
    if(uids_to_tree_info_.find(req->tree_uid) != uids_to_tree_info_.end())
    {
        TreeProcessInfo tree_info = uids_to_tree_info_.at(req->tree_uid);
        if(handleCallTriggerSrv("/"+tree_info.tree_name+ "/pause_tree"))
        {
          RCLCPP_INFO(node_->get_logger(), "Paused tree with UID: '%u' done ", req->tree_uid);
          return true;
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(), "Failed to Pause with UID: '%u'. Service could not process the request", req->tree_uid);
        }

    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to Pause tree: UID %u does not exist", req->tree_uid);
    }
    return false;
  }

  bool BehaviorTreeServer::resumeTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Resuming tree with UID: '%u' ", req->tree_uid);
    if(uids_to_tree_info_.find(req->tree_uid) != uids_to_tree_info_.end())
    {
        TreeProcessInfo tree_info = uids_to_tree_info_.at(req->tree_uid);
        if(handleCallTriggerSrv("/"+tree_info.tree_name+ "/resume_tree"))
        {
          RCLCPP_INFO(node_->get_logger(), "Resumed tree with UID: '%u' done ", req->tree_uid);
          return true;
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(), "Failed to Resumed with UID: '%u'. Service could not process the request", req->tree_uid);
        }
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to Resume tree: UID %u does not exist", req->tree_uid);
    }
    return false;
  }

  bool BehaviorTreeServer::getSyncBBValuesCB (const std::shared_ptr<GetBBValuesSrv::Request> req, std::shared_ptr<GetBBValuesSrv::Response> res)
  {
    for (auto key : req->keys)
    {
      auto bb_entry_str = BT::EutUtils::eutToJsonString(key, sync_blackboard_ptr_);
      if(bb_entry_str.has_value())
      {
          BBEntry bb_entry;
          bb_entry.key = key;
          bb_entry.type = BT::demangle(sync_blackboard_ptr_->getEntry(key)->info.type()); //sync_blackboard_ptr_->getEntry(key)->info.typeName();
          bb_entry.value = bb_entry_str.value();
          bb_entry.bt_id = ""; //Empty BT_ID for BTServer requests
          res->entries.push_back(bb_entry);
      }
      else
      {
        RCLCPP_WARN(node_->get_logger(), "Error fetching PortValue: %s  for port %s ", bb_entry_str.error().c_str(), key.c_str());
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
    for (auto tree_info : uids_to_tree_info_)
    {
      res->status.push_back(tree_info.second.tree_status);
    }
    return true;
  }

  void BehaviorTreeServer::treeStatusTopicCB(const TreeStatus::SharedPtr msg)   
  {
      uids_to_tree_info_.at(msg->uid).tree_status = *msg;
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
              RCLCPP_DEBUG(node_->get_logger(), "Init. BB key [\"%s\"] with value \"%s\" et %s t %s, ct %s", 
              bb_key.c_str(), bb_val.c_str(),
              BT::demangle(entry_n->info.type()).c_str(),
              BT::demangle(entry_n->value.type()).c_str(), BT::demangle(entry_n->value.castedType()).c_str());
        
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