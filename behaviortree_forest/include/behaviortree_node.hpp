#ifndef BEHAVIORTREE_NODE_HPP
#define BEHAVIORTREE_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "utils.hpp"
#include "tree_wrapper.hpp"

#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "behaviortree_forest_interfaces/msg/bb_entry.hpp"
#include "behaviortree_forest_interfaces/srv/get_loaded_plugins.hpp"
#include "behaviortree_forest_interfaces/srv/get_bb_values.hpp"

#include "behaviortree_cpp/blackboard.h"

using std::placeholders::_1;
using std::placeholders::_2;

using BBEntry = behaviortree_forest_interfaces::msg::BBEntry;
using GetLoadedPluginsSrv = behaviortree_forest_interfaces::srv::GetLoadedPlugins;
using GetTreeStatusSrv = behaviortree_forest_interfaces::srv::GetTreeStatus;
using GetBBValues = behaviortree_forest_interfaces::srv::GetBBValues;
using TriggerSrv = std_srvs::srv::Trigger;
using EmptySrv = std_srvs::srv::Empty;

namespace BT_SERVER
{
  class BehaviorTreeNode
  {
    public:
      BehaviorTreeNode(const rclcpp::Node::SharedPtr& node);
      ~BehaviorTreeNode() = default;
      void loop();
      uint loop_rate_ = 30;

    private:
      void sendBlackboardUpdates(const SyncMap& entries_map);
      void getBlackboardUpdates();
      
      void syncBBUpdateCB(const BBEntry::SharedPtr _topic_msg);
      bool getLoadedPluginsCB(const std::shared_ptr<GetLoadedPluginsSrv::Request> _request, std::shared_ptr<GetLoadedPluginsSrv::Response> _response);
      bool stopTreeCB(const std::shared_ptr<EmptySrv::Request> _request, std::shared_ptr<EmptySrv::Response> _response);
      bool killTreeCB(const std::shared_ptr<EmptySrv::Request> _request, std::shared_ptr<EmptySrv::Response> _response);
      bool pauseTreeCB(const std::shared_ptr<TriggerSrv::Request> _request, std::shared_ptr<TriggerSrv::Response> _response);
      bool resumeTreeCB(const std::shared_ptr<TriggerSrv::Request> _request, std::shared_ptr<TriggerSrv::Response> _response);
      bool restartTreeCB(const std::shared_ptr<EmptySrv::Request> _request, std::shared_ptr<EmptySrv::Response> _response);
      bool statusTreeCB(const std::shared_ptr<GetTreeStatusSrv::Request> _request, std::shared_ptr<GetTreeStatusSrv::Response> _response);
      void checkPausedCB();

      void removeTree();
      bool stopTree();
      bool loadTree();
      void getParameters (rclcpp::Node::SharedPtr nh);

      rclcpp::Node::SharedPtr node_ ;
      TreeWrapper tree_wrapper_;
      std::string trees_folder_;
      std::string tree_name_;

      bool tree_debug_;
      bool tree_auto_restart_;

      //Services
      rclcpp::Service<GetLoadedPluginsSrv>::SharedPtr get_loaded_plugins_srv_;
      rclcpp::Service<EmptySrv>::SharedPtr stop_tree_srv_;
      rclcpp::Service<EmptySrv>::SharedPtr kill_tree_srv_;
      rclcpp::Service<EmptySrv>::SharedPtr restart_tree_srv_;
      rclcpp::Service<TriggerSrv>::SharedPtr pause_tree_srv_;
      rclcpp::Service<TriggerSrv>::SharedPtr resume_tree_srv_;
      rclcpp::Service<GetTreeStatusSrv>::SharedPtr get_tree_status_srv_;

      //Subscribers
      rclcpp::Subscription<BBEntry>::SharedPtr sync_bb_sub_;

      //Publishers
      rclcpp::Publisher<BBEntry>::SharedPtr sync_bb_pub_;

      //Timers
      rclcpp::TimerBase::SharedPtr check_paused_timer_; 
      rclcpp::Rate rate;
  };
}


#endif  // BEHAVIORTREE_NODE_H