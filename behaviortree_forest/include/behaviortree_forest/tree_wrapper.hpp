#ifndef TREE_WRAPPER_HPP
#define TREE_WRAPPER_HPP

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_eut_plugins/loggers/bt_file_logger.h>
#include <behaviortree_cpp/loggers/bt_minitrace_logger.h>
#include <behaviortree_eut_plugins/loggers/bt_zmq_publisher.h>

#include "behaviortree_forest/bt_transition_logger.hpp"
#include "behaviortree_ros2/bt_utils.hpp"

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include "behaviortree_forest_interfaces/srv/get_tree_status.hpp"
#include "behaviortree_forest_interfaces/msg/transition.hpp"
#include "behaviortree_forest_interfaces/msg/bb_entry.hpp"

#include "behaviortree_forest/sync_blackboard.hpp"
#include "behaviortree_forest/utils.hpp"

// generated file
#include "bt_executor_parameters.hpp"

using TreeStatus = behaviortree_forest_interfaces::msg::TreeExecutionStatus;
using Transition = behaviortree_forest_interfaces::msg::Transition;
using BBEntry = behaviortree_forest_interfaces::msg::BBEntry;

namespace BT_SERVER
{

  class TreeWrapper
  {

  public:
    TreeWrapper(const rclcpp::Node::SharedPtr& node);
    ~TreeWrapper();
    rclcpp::Node::SharedPtr node() { return node_; }
    const BT::Tree& tree() const { return *tree_ptr_; }
    BT::Blackboard::Ptr rootBlackboard() { return root_blackboard_; }
    BT::BehaviorTreeFactory& factory() { return factory_; }
    BT::NodeAdvancedStatus tickTree();

    void loadAllPlugins();
    void initBB();
    void initStatusPublisher();
    TreeStatus buildTreeExecutionStatus();
    void updatePublishTreeExecutionStatus(const BT::NodeAdvancedStatus status, const bool avoid_duplicate = true);
    void publishExecutionStatus(bool error=false, std::string error_data="");
    BT::NodeAdvancedStatus getTreeExecutionStatus() { 
      std::lock_guard<std::mutex> lk(status_lock_);
      return status_;
    }
    bool isTreeLoaded() const { return !!tree_ptr_; }

    void removeTree();
    bool resetTree();
    void createTree(const std::string& full_pat, bool tree_debug);

    void initLoggers();
    bool areLoggersInit() { return loggers_init_; }

    void setExecuted(bool executed) { executed_ = executed; }
    bool hasExecutionTerminated() { return executed_; }

    size_t treeNodesCount();

    // TODO: Check if tree is Paused
    bool isTreePaused() { return isTreeLoaded() && debug_tree_ptr->isPaused(); }

    void syncBBUpdateCB(const BBEntry& _single_upd);
    void syncBBUpdateCB(const std::vector<BBEntry>& _bulk_upd);
    SyncMap getKeysValueToSync ();
    
    bool updateSyncMapEntrySyncStatus(const std::string& key, SyncStatus sync_status);
    bool updateSyncMapEntrySyncStatus(const std::string& key, SyncStatus expected_sync_status, SyncStatus new_sync_status); 
    bool addToSyncMap(const std::string& bb_key, SyncStatus sync_status);
    std::pair<bool, SyncEntry> checkForSyncKey(const std::string& key);
    bool checkSyncStatus(const std::string& key, SyncStatus sync_status);


    void checkForToSyncEntries();
    std::vector<std::string> getSyncKeysList();

    //Tree Status
    //std::string execution_tree_status_ {};
    std::string execution_tree_error_ {};

    //parameters
    bt_server::Params params_;
    std::vector<std::string> ros_plugin_directories_;
    std::string log_folder_;
    bool enable_cout_log_;
    bool enable_minitrace_log_;
    bool enable_rostopic_log_;
    bool enable_file_log_;
    bool enable_zmq_log_;
    int tree_server_port_;
    int tree_publisher_port_;
    int tree_uid_;
    std::string tree_filename_;
    std::string tree_name_;
    std::vector<std::string> tree_bb_init_ {};
    std::set<std::string> loaded_plugins_;
    std::shared_ptr<BT::DebuggableTree> debug_tree_ptr;

  private:
    void loadPluginsFromROS(std::vector<std::string> ros_plugins_folders);
    void loadPluginsFromFolder();

    void initBBFromFile(const std::string& abs_file_path);
    void initGrootV2Pub();
    void resetLoggers();
    BT::NodeAdvancedStatus toAdvancedNodeStatus (BT::NodeStatus status);
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<BT::Tree> tree_ptr_;

    BT::BehaviorTreeFactory factory_;
    BT::Blackboard::Ptr root_blackboard_;

    bool executed_{false};
    bool factory_initialized_ = false;

    //Loggers
    std::unique_ptr<BT::StdCoutLogger>   bt_logger_cout_;
    std::unique_ptr<BT::FileLogger>      bt_logger_file_;
    std::unique_ptr<BT::MinitraceLogger> bt_logger_trace_;
    std::unique_ptr<BT::PublisherZMQ>    bt_logger_zmq_;
    std::shared_ptr<BT::Groot2Publisher> groot_publisher_;
    std::shared_ptr<RosTopicTransitionLogger> bt_logger_transition_rostopic_;

    //Publishers
    rclcpp::Publisher<TreeStatus>::SharedPtr bt_execution_status_publisher_;

    BT::NodeAdvancedStatus status_ { BT::NodeAdvancedStatus::IDLE };
    std::atomic_bool loggers_init_{false};
    rclcpp::Time start_execution_time_;
    std::mutex status_lock_;

    //Sync
    std::mutex syncMap_lock_;
    SyncMap syncMap_;
  };
}
#endif
