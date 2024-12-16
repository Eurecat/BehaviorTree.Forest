
#include "behaviortree_server.hpp"

namespace BT_SERVER
{
  BehaviorTreeServer::BehaviorTreeServer(const rclcpp::Node::SharedPtr& node) : node_(node) 
  {
    //Add nh to executor
    executor_.add_node(node_);
    srv_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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
    sync_blackboard_ptr_ = BT::Blackboard::create();
    std::string sync_bb_init_file;
    if (!node_->get_parameter("sync_bb_init",sync_bb_init_file)){ sync_bb_init_file = ""; }
    if(sync_bb_init_file.length() > 0) { initBB(sync_bb_init_file, sync_blackboard_ptr_); }

    //Updates subscriber server side
    sync_bb_sub_ = node_->create_subscription<BBEntry>("behavior_tree_forest/local_update", 10, std::bind(&BehaviorTreeServer::syncBBCB, this, _1)) ;
    
    //Updates republisher for all trees (put latch to true atm, because seems a good option that you receive last update from the server)
    sync_bb_pub_ = node_->create_publisher<BBEntry>("behavior_tree_forest/broadcast_update", 10);
  }
  
  BehaviorTreeServer::~BehaviorTreeServer() 
  {
    RCLCPP_INFO(node_->get_logger(), "Calling destructor");
    killAllTreesCB(std::make_shared <EmptySrv::Request>(),std::make_shared <EmptySrv::Response>());
  }

  void BehaviorTreeServer::syncBBCB(const BBEntry::SharedPtr msg) const
  {
    RCLCPP_INFO(node_->get_logger(), "Received a Sync Update from %s BT. Key: '%s' Type: '%s' Value: '%s'", msg->bt_id.c_str(), msg->key.c_str(),msg->type.c_str(), msg->value.c_str());

    //Check if input msg type is Void
    const bool void_type = (msg->type == BT::demangle(typeid(void)));
    if (void_type)
    {
      //Skip Sync Entry as it still has no type declared
      return;
    }

    //Get the Entry & TypeInfo from the server BB 
    const BT::Blackboard::Entry* entry_bb_server = sync_blackboard_ptr_->getEntry(msg->key).get();
    const BT::TypeInfo* type_info_bb_server = sync_blackboard_ptr_->entryInfo(msg->key);

    //Check if the Entry or Type info exist on the BB
    if(entry_bb_server == nullptr || type_info_bb_server == nullptr)
    {
      // Entry not present in the BB -> Create & insert
      BT::TypeInfo new_type_info = createTypeInfoFromType(msg->type);
      sync_blackboard_ptr_->createEntry(msg->key, new_type_info);
      RCLCPP_INFO(node_->get_logger(),"Entry in Sync BB for key [%s] Initialized with type [%s]", msg->key.c_str(), new_type_info.typeName().c_str());
    }

    //Retrieve current entry in bt server bb
    auto entry_ptr = sync_blackboard_ptr_->getEntry(msg->key);
    auto type_info = sync_blackboard_ptr_->entryInfo(msg->key);
    if(!entry_ptr || !type_info)
    {
      // Error on BB Entry!
      RCLCPP_ERROR(node_->get_logger(),"Entry/TypeInfo should exist on the Server BlackBoard!");
      return;
    }

    if(msg->type != type_info->typeName())
    {
      // Type inconsistencies, don't update
      RCLCPP_ERROR(node_->get_logger(),"Sync Entry Type Missmatch for key [%s] - BB_type:[%s] RX_type:[%s]", msg->key.c_str(), type_info->typeName().c_str(), msg->type.c_str());
      return; 
    }

    try
    {
      // convert from string new value and insert on the BB
      BT::Any new_any_value = type_info->parseString(msg->value); //If No Value set yet, it will not update the Entry and prompt de Warning
      sync_blackboard_ptr_->set(msg->key, std::move(new_any_value));
    }
    catch(const std::exception& e)
    {
        RCLCPP_WARN(node_->get_logger(),"Failed to update sync port in SERVER BB for key [%s] with value [%s] : %s", msg->key.c_str(), msg->value.c_str(), e.what());
        return;
    }

    //Republish message to all BT_NODES
    BBEntry upd_msg;
    upd_msg.key = msg->key;
    upd_msg.type = sync_blackboard_ptr_->getEntry(msg->key)->info.type().name() ;
    upd_msg.value = msg->value;
    upd_msg.bt_id = msg->bt_id;
    sync_bb_pub_->publish(upd_msg);
    RCLCPP_INFO(node_->get_logger(), "Sync Port %s Updated on BT_Server BB and Republished succesfully",msg->key.c_str());
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
    std::string param_auto_restart = "tree_auto_restart:="+boolToString(req->auto_restart);
    std::string param_debug ="tree_debug:="+boolToString(req->debug);

    std::string param_bb_init = "tree_bb_init:=\\'";
    if (req->bb_init_files.size()> 0)
    {
        param_bb_init += "[";
        for (long unsigned int i = 0; i < req->bb_init_files.size(); i++)
        {
            param_bb_init += req->bb_init_files[i];
            if (i != (req->bb_init_files.size()-1))
                  param_bb_init += ",";
        }
        param_bb_init += "]\\'";
    }
    else
        param_bb_init += "[]\\'";

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

  bool BehaviorTreeServer::handleCallEmptySrv(rclcpp::Client<EmptySrv>::SharedPtr service_client)
  {
    RCLCPP_INFO(node_->get_logger(), "handleCallEmptySrv START");

    // Create the request for the Empty service you want to call
    auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();

    // Send the request asynchronously and get the FutureAndRequestId
    //auto future_response = service_client->async_send_request(empty_request);
    auto future = service_client->async_send_request(empty_request, std::bind(&BehaviorTreeServer::emptySrvCB, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "handleCallEmptySrv END");

    return true; // Indicate that the service was received
  }

  void BehaviorTreeServer::emptySrvCB(rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future)
  {
      RCLCPP_INFO(node_->get_logger(), "emptySrvCB");
      try {
          auto response = future.get();  // This will block until the response is received
          //RCLCPP_INFO(node_->get_logger(), "Empty Service Responded");
      } catch (const std::exception &e) {
          RCLCPP_ERROR(node_->get_logger(), "Empty Service call failed: %s", e.what());
      }
  }
  void BehaviorTreeServer::triggerSrvCB(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
  {
      RCLCPP_INFO(node_->get_logger(), "triggerSrvCB");
      try {
          auto response = future.get();  // This will block until the response is received
          //RCLCPP_INFO(node_->get_logger(), "Empty Service Responded");
      } catch (const std::exception &e) {
          RCLCPP_ERROR(node_->get_logger(), "Trigger Service call failed: %s", e.what());
      }
  }

  bool BehaviorTreeServer::rosServiceKillCall (std::string tree_name)
  {
    if (rclcpp::ok())
    {
      kill_service_client_ = node_->create_client<EmptySrv>("/"+tree_name+ "/kill_tree"); 
      if (!kill_service_client_->service_is_ready()) 
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to Kill tree: Service %s does not exist", kill_service_client_->get_service_name());
        return false;
      }
      return handleCallEmptySrv(kill_service_client_);
    }
    return true;
  }

  bool BehaviorTreeServer::rosServiceStopCall (std::string tree_name)
  {
    if (rclcpp::ok())
    {
      stop_service_client_ = node_->create_client<EmptySrv>("/"+tree_name+ "/stop_tree"); 
      if (!stop_service_client_->service_is_ready()) 
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to Stop tree: Service %s does not exist", stop_service_client_->get_service_name());
        return false;
      }
      return handleCallEmptySrv(stop_service_client_);
    }
    return true;
  }

  bool BehaviorTreeServer::rosServiceRestartCall (std::string tree_name)
  {
    restart_service_client_ = node_->create_client<EmptySrv>("/"+tree_name+ "/restart_tree");
    if (!restart_service_client_->service_is_ready()) 
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to Restart tree: Service %s does not exist", restart_service_client_->get_service_name());
      return false;
    }
    return handleCallEmptySrv(restart_service_client_);
  }

  bool BehaviorTreeServer::stopTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(), "Stopping tree with UID: '%u' ", req->tree_uid);
    if(uids_to_tree_info_.find(req->tree_uid) != uids_to_tree_info_.end())
    {
        TreeProcessInfo tree_info = uids_to_tree_info_.at(req->tree_uid);
        bool result = rosServiceStopCall(tree_info.tree_name);
        RCLCPP_INFO(node_->get_logger(), "Stopping tree with UID: '%u' done ", req->tree_uid);
        return result;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to Stop tree: UID %u does not exist", req->tree_uid);
    }
    return false;
  }
  
  bool BehaviorTreeServer::killTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(), "Killing tree with UID: '%u' ", req->tree_uid);
    if(uids_to_tree_info_.find(req->tree_uid) != uids_to_tree_info_.end())
    {
        TreeProcessInfo tree_info = uids_to_tree_info_.at(req->tree_uid);
        if (rosServiceKillCall(tree_info.tree_name))
        {
          //Extract extra PIDs created when executing "ros2 run ..." with fork()
          pid_t bt_node_pid = ros2_launch_manager_.extract_bt_node_pid_from_python_pid(tree_info.pid);
          std::string command;
          if (bt_node_pid!=0) 
            command = "kill -9 "+ std::to_string(bt_node_pid) + " ; " + "kill -9 "+ std::to_string(tree_info.pid);
          else
            command = "kill -9 "+ std::to_string(tree_info.pid);

          //RCLCPP_INFO(node_->get_logger(), "EXECUTING KILL COMMAND: %s", command.c_str());
          int resultcmd = system(command.c_str());
          /*if (resultcmd != -1)
            RCLCPP_INFO(node_->get_logger(), "Succesfully killed processes with PIDs: %s & %s",std::to_string(tree_info.pid).c_str(), std::to_string(bt_node_pid).c_str());
          else
            RCLCPP_ERROR(node_->get_logger(), "Failed to kill process with PIDs %s & %s",std::to_string(tree_info.pid).c_str(), std::to_string(bt_node_pid).c_str());
          */
          return true;
        }
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to kill process: UID %u does not exist", req->tree_uid);
    }

    return false;
  }

  bool BehaviorTreeServer::killAllTreesCB(const std::shared_ptr<EmptySrv::Request> req, std::shared_ptr<EmptySrv::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(), "Killing all trees");
    for (auto tree_info : uids_to_tree_info_)
    {
        std::shared_ptr<TreeRequestSrv::Request> request = std::make_shared<TreeRequestSrv::Request>();
        std::shared_ptr<TreeRequestSrv::Response> response;
        request->tree_uid  = tree_info.first;
        killTreeCB(request,response);
    }
    return true;
  }

  bool BehaviorTreeServer::restartTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(), "Restarting Tree with UID: '%u' ", req->tree_uid);
    if(uids_to_tree_info_.find(req->tree_uid) != uids_to_tree_info_.end())
    {
        TreeProcessInfo tree_info = uids_to_tree_info_.at(req->tree_uid);
        return rosServiceRestartCall(tree_info.tree_name);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to restart tree: UID %u does not exist", req->tree_uid);
    }
    return false;
  }

  bool BehaviorTreeServer::pauseTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(), "Pausing tree with UID: '%u' ", req->tree_uid);
    if(uids_to_tree_info_.find(req->tree_uid) != uids_to_tree_info_.end())
    {
        TreeProcessInfo tree_info = uids_to_tree_info_.at(req->tree_uid);
        pause_service_client_ = node_->create_client<TriggerSrv>("/"+tree_info.tree_name+ "/pause_tree"); 
        if (!pause_service_client_->service_is_ready()) 
        {
          RCLCPP_ERROR(node_->get_logger(), "Failed to Pause tree: Service %s does not exist", pause_service_client_->get_service_name());
          return false;
        }
        auto trigger_request = std::make_shared<TriggerSrv::Request>();
        //service_client->async_send_request(trigger_request);
        auto future = pause_service_client_->async_send_request(trigger_request, std::bind(&BehaviorTreeServer::triggerSrvCB, this, std::placeholders::_1));
    
        RCLCPP_INFO(node_->get_logger(), "Paused tree with UID: '%u' done ", req->tree_uid);
        return true;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to Pause tree: UID %u does not exist", req->tree_uid);
    }
    return false;
  }

  bool BehaviorTreeServer::resumeTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(), "Resuming tree with UID: '%u' ", req->tree_uid);
    if(uids_to_tree_info_.find(req->tree_uid) != uids_to_tree_info_.end())
    {
        TreeProcessInfo tree_info = uids_to_tree_info_.at(req->tree_uid);
        resume_service_client_ = node_->create_client<TriggerSrv>("/"+tree_info.tree_name+ "/resume_tree"); 
        if (!resume_service_client_->service_is_ready()) 
        {
          RCLCPP_ERROR(node_->get_logger(), "Failed to Pause tree: Service %s does not exist", resume_service_client_->get_service_name());
          return false;
        }
        auto trigger_request = std::make_shared<TriggerSrv::Request>();
        //service_client->async_send_request(trigger_request);
        auto future = resume_service_client_->async_send_request(trigger_request, std::bind(&BehaviorTreeServer::triggerSrvCB, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "Resumed tree with UID: '%u' done ", req->tree_uid);
        return true;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to Resume tree: UID %u does not exist", req->tree_uid);
    }
    return false;
  }

  bool BehaviorTreeServer::getSyncBBValuesCB (const std::shared_ptr<GetBBValuesSrv::Request> req, std::shared_ptr<GetBBValuesSrv::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(), "getSyncBBValuesCB");

    for (auto key : req->keys)
    {
      RCLCPP_INFO(node_->get_logger(), "Sync Key: '%s' ", key.c_str());

      auto bb_entry_str = BT::getEntryAsString(key,sync_blackboard_ptr_);
      if(bb_entry_str.has_value())
      {
          BBEntry bb_entry;
          bb_entry.key = key;
          bb_entry.type = sync_blackboard_ptr_->getEntry(key)->info.typeName();
          bb_entry.value = bb_entry_str.value();
          bb_entry.bt_id = ""; //Empty BT_ID for BTServer requests
          res->entries.push_back(bb_entry);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Error fetching PortValue: %s  for port %s ", bb_entry_str.error().c_str(), key.c_str());
      }
    }
    return true;
  }
  
  bool BehaviorTreeServer::getTreeStatusCB(const std::shared_ptr<GetTreeStatusSrv::Request> req, std::shared_ptr<GetTreeStatusSrv::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(), "Getting Tree status with UID: '%u' ", req->tree_uid);
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
    RCLCPP_INFO(node_->get_logger(), "Getting All trees status");
    for (auto tree_info : uids_to_tree_info_)
    {
      res->status.push_back(tree_info.second.tree_status);
    }
    return true;
  }

  void BehaviorTreeServer::treeStatusTopicCB(const TreeStatus::SharedPtr msg)   
  {
      uids_to_tree_info_.at(msg->uid).tree_status = *msg;
      RCLCPP_INFO(node_->get_logger(),"New Status topic RX: Tree_name:%s New status:%u", uids_to_tree_info_.at(msg->uid).tree_name.c_str() , uids_to_tree_info_.at(msg->uid).tree_status.status);
  }

  void BehaviorTreeServer::initBB(const std::string& abs_file_path, BT::Blackboard::Ptr blackboard_ptr)
  {
    try 
    {
        RCLCPP_INFO(node_->get_logger(), "Initializing BB from YAML file %s", abs_file_path.c_str());
        YAML::Node config = YAML::LoadFile(abs_file_path);
        for(YAML::const_iterator it=config.begin();it!=config.end();++it)
        {
            const std::string& bb_key = it->first.as<std::string>();
            std::string bb_val = it->second.as<std::string>();
            
            const BT::Expected<std::string> bbentry_value_inferred_keyvalues = replaceKeysWithStringValues(bb_val,blackboard_ptr,true); // no effect if it has no key
            if(!bbentry_value_inferred_keyvalues)
            {
                // but will complain if it has a reference to a wrong key
                RCLCPP_ERROR(node_->get_logger(), "Init. of BB key %s for value %s, value inference did not succeed: %s", 
                    bb_key.c_str(), 
                    bb_val.c_str(),
                    bbentry_value_inferred_keyvalues.error().c_str());
                continue; // and skip this init
            }
            else
                bb_val = bbentry_value_inferred_keyvalues.value();

            RCLCPP_INFO(node_->get_logger(), "Init. BB key [\"%s\"] with value \"%s\"", bb_key.c_str(), bb_val.c_str());
            // use the string here and blackboard_ptr->set(...)
            blackboard_ptr->set(bb_key, bb_val);
        }
        RCLCPP_INFO(node_->get_logger(), "Initialized BB with %ld entries from YAML file %s", blackboard_ptr->getKeys().size(), abs_file_path.c_str());
    }
    catch(const YAML::Exception& ex) 
    { 
        RCLCPP_ERROR(node_->get_logger(), "Initializing. BB key from file '%s' did not succeed: %s", abs_file_path.c_str(), ex.what());
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

 // Create a thread for running the multithreaded executor
  std::thread executor_thread([bt_server]() {
      bt_server->run();
  });

  executor_thread.join();
  rclcpp::shutdown();

  return 0;
}