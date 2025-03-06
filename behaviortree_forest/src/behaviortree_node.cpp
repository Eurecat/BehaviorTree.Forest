#include "behaviortree_forest/behaviortree_node.hpp"
#include "behaviortree_eut_plugins/utils/eut_utils.h"

#include "behaviortree_forest/params/behaviortree_node_params.hpp"

#include <chrono>

namespace BT_SERVER
{
  BehaviorTreeNode::BehaviorTreeNode(const rclcpp::Node::SharedPtr& node) : tree_wrapper_(node), 
    node_(node), 
    executor_(rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), 2))
  {
    std::cout << "BehaviorTreeNode " << "\n" << std::flush;
    srv_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    bb_upd_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    //Fetch Tree Parameters
    getParameters(node);

    //Create BT_NODE Tree ROS Services
    RCLCPP_INFO(node_->get_logger(),"Creating ROS2 Services, Subscribers and Publishers for behavior_tree_node %s", tree_wrapper_.tree_name_.c_str());

    get_loaded_plugins_srv_ =node->create_service<GetLoadedPluginsSrv>("/"+tree_name_+"/get_loaded_plugins",std::bind(&BehaviorTreeNode::getLoadedPluginsCB,this,_1,_2), rmw_qos_profile_services_default, srv_cb_group_);
    stop_tree_srv_ = node_->create_service<EmptySrv>("/"+tree_name_+"/stop_tree",std::bind(&BehaviorTreeNode::stopTreeCB,this,_1,_2), rmw_qos_profile_services_default, srv_cb_group_);
    kill_tree_srv_ = node_->create_service<EmptySrv>("/"+tree_name_+"/kill_tree",std::bind(&BehaviorTreeNode::killTreeCB,this,_1,_2), rmw_qos_profile_services_default, srv_cb_group_);
    restart_tree_srv_ = node_->create_service<EmptySrv>("/"+tree_name_+"/restart_tree",std::bind(&BehaviorTreeNode::restartTreeCB,this,_1,_2), rmw_qos_profile_services_default, srv_cb_group_);
    get_tree_status_srv_ = node_->create_service<GetTreeStatusSrv>("/"+tree_name_+"/status_tree",std::bind(&BehaviorTreeNode::statusTreeCB,this,_1,_2), rmw_qos_profile_services_default, srv_cb_group_);


        // Define the subscription options
    rclcpp::SubscriptionOptions sync_bb_sub_options;
    sync_bb_sub_options.callback_group = bb_upd_cb_group_;  // Attach the subscription to the callback group
    // Updates subscriber server side
    sync_bb_sub_ = node_->create_subscription<BBEntry>("behavior_tree_forest/broadcast_update", 10, std::bind(&BehaviorTreeNode::syncBBUpdateCB, this, _1), sync_bb_sub_options);
    
    // Updates republisher for all trees (put latch to true atm, because seems a good option that you receive last update from the server)
    sync_bb_pub_ = node_->create_publisher<BBEntry>("/behavior_tree_forest/local_update", 10);

    //Init Publishers
    tree_wrapper_.initStatusPublisher();

    if (tree_wrapper_.tree_uid_ >= 1) 
    {
      //Load Plugins
      tree_wrapper_.loadAllPlugins();

      //Create Blackboard
      tree_wrapper_.initBB();

      //Load Tree
      if (!loadTree()) {return;}
      tree_wrapper_.rootBlackboard()->debugMessage();
      
      RCLCPP_INFO(node_->get_logger(),"Tree loaded OK");

      executor_.add_node(node_);
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(),"Tree UID must be greater than 1");
        return;
    }

    // Publish the initial status (IDLE + no tree loaded).
    tree_wrapper_.updatePublishTreeExecutionStatus(BT::NodeAdvancedStatus::IDLE,false);

    // Publish Paused Status Timer
    if (tree_debug_)
    {
      pause_tree_srv_  = node->create_service<TriggerSrv>("/"+tree_name_+"/pause_tree", std::bind(&BehaviorTreeNode::pauseTreeCB,this,_1,_2),rmw_qos_profile_services_default, srv_cb_group_);
      resume_tree_srv_  = node->create_service<TriggerSrv>("/"+tree_name_+"/resume_tree", std::bind(&BehaviorTreeNode::resumeTreeCB,this,_1,_2),rmw_qos_profile_services_default, srv_cb_group_);
      check_paused_timer_ = node_->create_wall_timer(std::chrono::milliseconds(1000/loop_rate_), std::bind(&BehaviorTreeNode::checkPausedCB,this));
    }

    loop_timer_ = node_->create_wall_timer(std::chrono::milliseconds(1000/loop_rate_), std::bind(&BehaviorTreeNode::loopStep,this)); // Reentrant callback group y multithreaded executor (2 threads)
  }

  bool BehaviorTreeNode::loadTree()
  {
    const auto& full_path = getTreeFullPath(tree_wrapper_.tree_filename_, trees_folder_);

    //Init error in case of CRASH
    tree_wrapper_.execution_tree_error_ = "COULD NOT LOAD";

    // If there's a tree being executed, halt and destoy it to execute the new one
    if (tree_wrapper_.isTreeLoaded()) { tree_wrapper_.removeTree(); }

    try
    {
      RCLCPP_INFO(node_->get_logger(),"Creating tree from file %s", full_path.c_str());
      tree_wrapper_.createTree(full_path,tree_debug_);
      tree_wrapper_.rootBlackboard()->debugMessage();
    }
    catch(const std::runtime_error& ex)
    {
      RCLCPP_ERROR(node_->get_logger(),"Error loading tree %s: %s", full_path.c_str(), ex.what());

      std::string error_str = "Error loading tree " + full_path + " " + std::string(ex.what());;
      tree_wrapper_.execution_tree_error_ = ex.what() ;

      tree_wrapper_.publishExecutionStatus(true,error_str);
      return false;
    }

    // Send Sync Blackboard updates
    sendBlackboardUpdates(tree_wrapper_.getKeysValueToSync()); // send updates 
    // ASK DAVID we are assuming it can never run standalone!
    getBlackboardUpdates(); // blocking call to update bb with missing values that need to be retrieved from server

    //Init Loggers
    tree_wrapper_.initLoggers();

    tree_wrapper_.execution_tree_error_ = "";
    
    tree_wrapper_.publishExecutionStatus();

    RCLCPP_INFO(node_->get_logger(),"Loaded srv tree %s counting of %ld nodes", full_path.c_str(), tree_wrapper_.treeNodesCount());
    return true;
  }

  void BehaviorTreeNode::checkPausedCB()
  {
    //RCLCPP_INFO(node_->get_logger(), "checkPausedCB()");
    //Check if Tree is Paused
    if(tree_wrapper_.isTreeLoaded())
    {
        if(tree_wrapper_.debug_tree_ptr->isPaused() && tree_wrapper_.getTreeExecutionStatus() != BT::NodeAdvancedStatus::PAUSED)
        {
            tree_wrapper_.updatePublishTreeExecutionStatus(BT::NodeAdvancedStatus::PAUSED);
        } 
    }
  }

  void BehaviorTreeNode::syncBBUpdateCB(const BBEntry::SharedPtr _topic_msg)
  {   
    RCLCPP_INFO(node_->get_logger(), "syncBBUpdateCB on key: %s", _topic_msg->key.c_str());
    // Forward received update to tree
    if(tree_wrapper_.isTreeLoaded()) tree_wrapper_.syncBBUpdateCB(*_topic_msg);
  }

  void BehaviorTreeNode::sendBlackboardUpdates(const SyncMap& entries_map)
  {
    for(const auto& ser_entry : entries_map)
    {
      RCLCPP_INFO(node_->get_logger(), "sendBlackboardUpdate on key: %s", ser_entry.first.c_str());
      //Check Sync Value is initialized
      auto val = BT::EutUtils::eutToJsonString(ser_entry.first, tree_wrapper_.rootBlackboard());

      //if (val.has_value())
      {
        BBEntry bb_entry_msg;
        bb_entry_msg.key = ser_entry.first;
        bb_entry_msg.type = BT::demangle(ser_entry.second.entry->info.type()); //ser_entry.second.entry->info.typeName();
       
        if (val.has_value())
        {
          bb_entry_msg.value = val.value();
          // if(ser_entry.second.entry->info.type() == typeid(std::string))
          // {
          //   if (!bb_entry_msg.value.empty() && bb_entry_msg.value.front() == '"' && bb_entry_msg.value.back() == '"') 
          //   {
          //     bb_entry_msg.value = bb_entry_msg.value.substr(1, bb_entry_msg.value.size() - 2);
          //   }
          // }

          RCLCPP_INFO(node_->get_logger(), "sendBlackboardUpdate on key: %s, its value as a stringified json will be like %s and if I try to convert it back to json its type is now %s", 
            ser_entry.first.c_str(), bb_entry_msg.value.c_str(), nlohmann::json::parse(val.value()).type_name());
        }
        else
          bb_entry_msg.value = "";
        bb_entry_msg.bt_id = tree_name_;
        bb_entry_msg.sender_sequence_id = ser_entry.second.entry->sequence_id;
        
        if(tree_wrapper_.updateSyncMapEntrySyncStatus(bb_entry_msg.key, SyncStatus::TO_SYNC, SyncStatus::SYNCING) || tree_wrapper_.checkSyncStatus(bb_entry_msg.key,SyncStatus::SYNCING))
        {
          sync_bb_pub_->publish(bb_entry_msg);
        }
      }
      // else
      // {
      //   RCLCPP_ERROR(node_->get_logger(), "Sync Key: %s with type [%s] has not got a Value", ser_entry.first.c_str(), ser_entry.second.second->info.typeName().c_str() );
      // }
    }
    //std::cout << "send BB UPDATES OK" << std::flush;
  }

  void BehaviorTreeNode::getBlackboardUpdates()
  {
    RCLCPP_INFO(node_->get_logger(), "getBlackboardUpdates()");
    rclcpp::Client<GetBBValues>::SharedPtr client = node_->create_client<GetBBValues>("/behavior_tree_forest/get_sync_bb_values"); 
    auto request = std::make_shared<GetBBValues::Request>();

    request->keys = tree_wrapper_.getSyncKeysList();

    while (!client->wait_for_service()) 
    {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
    }
    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      tree_wrapper_.syncBBUpdateCB(result.get()->entries);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "ERROR: Failed to call service get_sync_bb_values");
    }
  }

  void BehaviorTreeNode::loopStep()
  {
    // while(rclcpp::ok())
    // {
      // Sleep if no tree running (main and remote)
      if(!tree_wrapper_.isTreeLoaded())
      {
        RCLCPP_INFO(node_->get_logger(),"TREE NOT LOADED -- Killing Node");
        rclcpp::shutdown();
        return;
      }

      if(tree_wrapper_.areLoggersInit() && !tree_wrapper_.hasExecutionTerminated()) 
      { 
          try
          {
              //RCLCPP_INFO(node_->get_logger(),"TICK ONCE");
              const auto tree_status = tree_wrapper_.tickTree(); 
              RCLCPP_INFO(node_->get_logger(),"TICK ONCE OK");

              RCLCPP_INFO(node_->get_logger(),"update sync status");  
              tree_wrapper_.checkForToSyncEntries(); 

              RCLCPP_INFO(node_->get_logger(),"sendBlackboardUpdates");
              sendBlackboardUpdates(tree_wrapper_.getKeysValueToSync());
              
              // Publish the updated status if there have been changes.
              if(tree_status != tree_wrapper_.getTreeExecutionStatus())
              {
                  tree_wrapper_.updatePublishTreeExecutionStatus(tree_status);
              }

              // Publish the updated status
              if(tree_status == BT::NodeAdvancedStatus::FAILURE)
              {
                  tree_wrapper_.resetTree();
                  tree_wrapper_.setExecuted(!tree_auto_restart_); // if auto restart is false, set executed to true to stop the tick, otherwise will restart the tick from the beginning
              }
              else if(tree_status == BT::NodeAdvancedStatus::SUCCESS)
              {
                  tree_wrapper_.resetTree();
                  tree_wrapper_.setExecuted(!tree_auto_restart_); // if auto restart is false, set executed to true to stop the tick, otherwise will restart the tick from the beginning
              }
          }
          catch(const BT::BehaviorTreeException& ex)
          {
              std::string error_str = "ERROR: Tree crashed with exception [ " + std::string(ex.what()) + " ]";
              RCLCPP_ERROR(node_->get_logger(),"Tree crashed with exception: %s", ex.what());
              tree_wrapper_.execution_tree_error_ = ex.what() ;
              tree_wrapper_.publishExecutionStatus(true, error_str);
              removeTree();
          }
      }
      else
      {
        //RCLCPP_INFO(node_->get_logger(),"EXEC TERMINATED");
        loop_timer_->cancel();
        loop_timer_->reset();  
      }
  }


  bool BehaviorTreeNode::getLoadedPluginsCB(const std::shared_ptr<GetLoadedPluginsSrv::Request> _request, std::shared_ptr<GetLoadedPluginsSrv::Response> _response)
  {
    _response->plugins.assign(tree_wrapper_.loaded_plugins_.cbegin(), tree_wrapper_.loaded_plugins_.cend());
    return true;
  }

  void BehaviorTreeNode::removeTree()
  {
    RCLCPP_INFO(node_->get_logger(),"removeTree()");
    tree_wrapper_.removeTree();

    // Update and publish the status here too
    // (neeed to cover the case where users manually
    // stop a tree by calling the stop_tree service).
    tree_wrapper_.updatePublishTreeExecutionStatus(BT::NodeAdvancedStatus::IDLE, false);
  }

  bool BehaviorTreeNode::stopTree()
  {
    RCLCPP_INFO(node_->get_logger(),"stopTree()");
    if(tree_wrapper_.isTreeLoaded())
    {
        removeTree();
    }
    tree_wrapper_.setExecuted(true);
    return true;
  }

  bool BehaviorTreeNode::stopTreeCB(const std::shared_ptr<EmptySrv::Request> _request, std::shared_ptr<EmptySrv::Response> _response)
  {
    RCLCPP_INFO(node_->get_logger(),"stopTreeCB");
    tree_wrapper_.execution_tree_error_ = "Canceled by stopTree Service";
    tree_wrapper_.updatePublishTreeExecutionStatus(BT::NodeAdvancedStatus::IDLE, false);

    bool res = stopTree();
    _response = std::make_shared <EmptySrv::Response>();
    return res;
  }
  bool BehaviorTreeNode::killTreeCB(const std::shared_ptr<EmptySrv::Request> _request, std::shared_ptr<EmptySrv::Response> _response)
  {
    RCLCPP_INFO(node_->get_logger(),"killTreeCB");
    bool res = stopTreeCB(_request,_response);
    tree_wrapper_.execution_tree_error_ = "Canceled by killTree Service";
    tree_wrapper_.updatePublishTreeExecutionStatus(BT::NodeAdvancedStatus::IDLE, false);
    return res;
  }

  bool BehaviorTreeNode::pauseTreeCB(const std::shared_ptr<TriggerSrv::Request> _request, std::shared_ptr<TriggerSrv::Response> _response)
  {
    RCLCPP_INFO(node_->get_logger(),"pauseTreeCB");
    //TODO: Allow to pause execution without specifying BT::NODE
    if(!tree_wrapper_.isTreePaused())
      tree_wrapper_.debug_tree_ptr->debugResume(BT::DebuggableTree::ExecMode::DEBUG_STEP);
    _response->success = tree_wrapper_.isTreePaused();
    
    RCLCPP_INFO(node_->get_logger(),"pauseTreeCB_OK");
    return _response->success;
  }
  bool BehaviorTreeNode::resumeTreeCB(const std::shared_ptr<TriggerSrv::Request> _request, std::shared_ptr<TriggerSrv::Response> _response)
  {
    RCLCPP_INFO(node_->get_logger(),"resumeTreeCB");
    _response->success = tree_wrapper_.debug_tree_ptr->debugResume();
    RCLCPP_INFO(node_->get_logger(),"resumeTreeCB_OK");
    return _response->success;
  }
  bool BehaviorTreeNode::restartTreeCB(const std::shared_ptr<EmptySrv::Request> _request, std::shared_ptr<EmptySrv::Response> _response)
  {
    RCLCPP_INFO(node_->get_logger(),"restartTreeCB");
    if (tree_wrapper_.isTreeLoaded())
    {
      tree_wrapper_.resetTree();

      tree_wrapper_.setExecuted(false);
      RCLCPP_INFO(node_->get_logger(),"SET EXECUTED = FALSE");
      return true;
    }
    return false;
  }
  bool BehaviorTreeNode::statusTreeCB(const std::shared_ptr<GetTreeStatusSrv::Request> _request, std::shared_ptr<GetTreeStatusSrv::Response> _response)
  {
    RCLCPP_INFO(node_->get_logger(),"statusTreeCB");
    if (tree_wrapper_.isTreeLoaded())
    {
      _response->status = tree_wrapper_.buildTreeExecutionStatus();
      return true;
    }
    _response->status.details = "NOT LOADED";
    return false;
  }

  void BehaviorTreeNode::getParameters (rclcpp::Node::SharedPtr nh)
  {
    std::cout << "getParameters " << "\n" << std::flush;
    // Declare parameters
    nh->declare_parameter(PARAM_NAME_TREES_FOLDER, "src/behaviortree_forest/behavior_trees");
    nh->declare_parameter(PARAM_NAME_ENABLE_COUT_LOG, true);
    nh->declare_parameter(PARAM_NAME_ENABLE_MINITRACE_LOG, true);
    nh->declare_parameter(PARAM_NAME_ENABLE_ROSTOPIC_LOG, true);
    nh->declare_parameter(PARAM_NAME_ENABLE_FILE_LOG, true);
    nh->declare_parameter(PARAM_NAME_ENABLE_GROOT_LOG, true);
    nh->declare_parameter(PARAM_NAME_TREE_NAME, "behaviortree");
    nh->declare_parameter(PARAM_NAME_TREE_FILE, "behaviortree.xml");
    nh->declare_parameter(PARAM_NAME_TREE_UID, 0);
    nh->declare_parameter(PARAM_NAME_TREE_DEBUG, false);
    nh->declare_parameter(PARAM_NAME_TREE_AUTORESTART, false);
    nh->declare_parameter(PARAM_NAME_TREE_SERVER_PORT, 1667);
    nh->declare_parameter(PARAM_NAME_TREE_PUBLISHER_PORT, 1666);
    nh->declare_parameter(PARAM_NAME_TREE_LOG_FOLDER, "/tmp/");
    nh->declare_parameter(PARAM_NAME_TREE_BB_INIT, std::vector<std::string>());
    nh->declare_parameter(PARAM_NAME_TREE_ROS_PLUGINS_DIR,std::vector<std::string>());

    nh->declare_parameter(PARAM_NAME_TREE_LOOP_RATE, 30);

    RCLCPP_INFO(nh->get_logger(),"Loading parameters");
      
    // Load Parameters
    trees_folder_ = nh->get_parameter(PARAM_NAME_TREES_FOLDER).as_string();
    tree_name_= nh->get_parameter(PARAM_NAME_TREE_NAME).as_string();
    tree_debug_ = nh->get_parameter(PARAM_NAME_TREE_DEBUG).as_bool();
    tree_auto_restart_ = nh->get_parameter(PARAM_NAME_TREE_AUTORESTART).as_bool();
    tree_wrapper_.tree_name_ = tree_name_;
    tree_wrapper_.tree_uid_ = nh->get_parameter(PARAM_NAME_TREE_UID).as_int();
    tree_wrapper_.tree_filename_ = nh->get_parameter(PARAM_NAME_TREE_FILE).as_string();
    tree_wrapper_.enable_cout_log_ = nh->get_parameter(PARAM_NAME_ENABLE_COUT_LOG).as_bool();
    tree_wrapper_.enable_minitrace_log_ = nh->get_parameter(PARAM_NAME_ENABLE_MINITRACE_LOG).as_bool();
    tree_wrapper_.enable_rostopic_log_ = nh->get_parameter(PARAM_NAME_ENABLE_ROSTOPIC_LOG).as_bool();
    tree_wrapper_.enable_file_log_ = nh->get_parameter(PARAM_NAME_ENABLE_FILE_LOG).as_bool();
    tree_wrapper_.enable_zmq_log_ = nh->get_parameter(PARAM_NAME_ENABLE_GROOT_LOG).as_bool();
    tree_wrapper_.log_folder_ = nh->get_parameter(PARAM_NAME_TREE_LOG_FOLDER).as_string();
    tree_wrapper_.tree_server_port_ = nh->get_parameter(PARAM_NAME_TREE_SERVER_PORT).as_int();
    tree_wrapper_.tree_publisher_port_ = nh->get_parameter(PARAM_NAME_TREE_PUBLISHER_PORT).as_int();
    tree_wrapper_.params_.groot2_port = tree_wrapper_.tree_server_port_;

    std::cout << "Fetching " << PARAM_NAME_TREE_ROS_PLUGINS_DIR << "\n" << std::flush;
    tree_wrapper_.ros_plugin_directories_ = node_->get_parameter(PARAM_NAME_TREE_ROS_PLUGINS_DIR).as_string_array();
    // tree_wrapper_.ros_plugin_directories_.push_back("behaviortree_ros2/bt_plugins");

    std::cout << "Fetching " << PARAM_NAME_TREE_LOOP_RATE << "\n" << std::flush;
    loop_rate_ = nh->get_parameter(PARAM_NAME_TREE_LOOP_RATE).as_int();
 
    //Build BB_init Vector (OLD WAY)
    /*std::string bb_init;
    bb_init = nh->get_parameter("bb_init").as_string();
    bb_init.erase(std::remove(bb_init.begin(), bb_init.end(),'['), bb_init.end());
    bb_init.erase(std::remove(bb_init.begin(), bb_init.end(),']'), bb_init.end());
    std::stringstream bb_init_stream(bb_init);
    std::string s;
    while (getline(bb_init_stream, s, ',')) {
      s.erase(std::remove(s.begin(), s.end(), '\''), s.end());
      tree_wrapper_.tree_bb_init_.push_back(s);
    }*/

    //Build BB_init Vector (New WAY TO TEST)
    std::cout << "Fetching " << PARAM_NAME_TREE_BB_INIT << "\n" << std::flush;
    tree_wrapper_.tree_bb_init_ = node_->get_parameter(PARAM_NAME_TREE_BB_INIT).as_string_array();

    RCLCPP_INFO(nh->get_logger(),"Parameters Loaded succesfully");
  }

  void BehaviorTreeNode::run()
  {
    if(tree_wrapper_.isTreeLoaded())
      executor_.spin();
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("behavior_tree_node");
  // RCLCPP_INFO(nh->get_logger(),"SPAWNING new behavior_tree_node");

  auto bt_node = std::make_shared<BT_SERVER::BehaviorTreeNode>(nh);
  bt_node->run();
  bt_node.reset();

  // RCLCPP_INFO(nh->get_logger(),"TERMINATING behavior_tree_node");
  rclcpp::shutdown();
  return 0;
}