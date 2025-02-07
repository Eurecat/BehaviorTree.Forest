#ifndef BEHAVIORTREE_SERVER_HPP
#define BEHAVIORTREE_SERVER_HPP

#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "behaviortree_forest_interfaces/msg/bb_entry.hpp"
#include "behaviortree_forest_interfaces/msg/tree_execution_status.hpp"

#include "behaviortree_forest_interfaces/srv/load_tree.hpp"
#include "behaviortree_forest_interfaces/srv/get_tree_status_by_id.hpp"
#include "behaviortree_forest_interfaces/srv/get_tree_status.hpp"
#include "behaviortree_forest_interfaces/srv/get_bb_values.hpp"
#include "behaviortree_forest_interfaces/srv/tree_request.hpp"
#include "behaviortree_forest_interfaces/srv/get_all_trees_status.hpp"

#include "behaviortree_cpp/blackboard.h"
#include <behaviortree_cpp/bt_factory.h>

#include "behaviortree_forest/sync_blackboard.hpp"
#include "behaviortree_forest/ros2_launch_manager.hpp"
#include "behaviortree_forest/utils.hpp"
#include "yaml-cpp/yaml.h"

#include <chrono>

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

using BBEntry = behaviortree_forest_interfaces::msg::BBEntry;
using TreeStatus = behaviortree_forest_interfaces::msg::TreeExecutionStatus;
using LoadTreeSrv = behaviortree_forest_interfaces::srv::LoadTree;
using TreeRequestSrv = behaviortree_forest_interfaces::srv::TreeRequest;
using GetBBValuesSrv = behaviortree_forest_interfaces::srv::GetBBValues;
using GetTreeStatusSrv = behaviortree_forest_interfaces::srv::GetTreeStatusByID;
using GetAllTreeStatusSrv = behaviortree_forest_interfaces::srv::GetAllTreesStatus;
using TriggerSrv = std_srvs::srv::Trigger;
using EmptySrv = std_srvs::srv::Empty;

namespace BT_SERVER
{
  class TreeProcessInfo
  {
    public:
      TreeProcessInfo(std::string in_tree_name, pid_t in_pid)
      {
          tree_name = in_tree_name;
          pid = in_pid;
          killed = false;
      };
      pid_t pid;
      std::string tree_name;
      TreeStatus tree_status;
      bool killed;
      rclcpp::Subscription<TreeStatus>::SharedPtr status_subscriber;
  };
  class BehaviorTreeServer
  {
    public:
      BehaviorTreeServer(const rclcpp::Node::SharedPtr& node) ;
      ~BehaviorTreeServer();
      void run();
      
    private:
      bool loadTreeCB(const std::shared_ptr<LoadTreeSrv::Request> req, std::shared_ptr<LoadTreeSrv::Response> res);
      bool stopTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res);
      bool killTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res);
      bool killTree(const uint8_t tree_uid, const bool force_kill = false);
      bool killAllTrees(const bool force_kill = false);
      bool killAllTreesCB(const std::shared_ptr<EmptySrv::Request> req, std::shared_ptr<EmptySrv::Response> res);
      bool pauseTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res);
      bool resumeTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res);
      bool restartTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res);
      bool getSyncBBValuesCB (const std::shared_ptr<GetBBValuesSrv::Request> req, std::shared_ptr<GetBBValuesSrv::Response> res);
      bool getTreeStatusCB(const std::shared_ptr<GetTreeStatusSrv::Request> req, std::shared_ptr<GetTreeStatusSrv::Response> res);
      bool getAllTreeStatusCB(const std::shared_ptr<GetAllTreeStatusSrv::Request> req, std::shared_ptr<GetAllTreeStatusSrv::Response> res);
      void treeStatusTopicCB(const TreeStatus::SharedPtr msg);
      void syncBBCB(const BBEntry::SharedPtr msg) const;
      void initBB(const std::string& abs_file_path, BT::Blackboard::Ptr blackboard_ptr);

      template<typename T>
      std::pair<std::shared_ptr<typename T::Response>, rclcpp::FutureReturnCode> handleSyncSrvCall(const std::string& service_name)
      {
        const auto bad_request = std::make_pair(nullptr, rclcpp::FutureReturnCode::INTERRUPTED);
        if (!rclcpp::ok())  // ✅ Ensure ROS is still running before creating a node
        {
          std::cerr << "Trying to create a node and create a service client with ROS shutting down\n" << std::flush;
          return bad_request;
        }
        
        std::string client_node_name = "client_" + std::string(service_name);
        std::replace(client_node_name.begin(), client_node_name.end(), '/', '_');
        auto client_node = std::make_shared<rclcpp::Node>(client_node_name);

        const auto service_client = client_node->create_client<T>(service_name); 
        if (!service_client->service_is_ready()) 
        {
          RCLCPP_ERROR(client_node->get_logger(), "Failed service call: service %s does not exist", service_client->get_service_name());
          return bad_request;
        }

        // Create the request for the Empty service you want to call
        auto trigger_request = std::make_shared<typename T::Request>();
        auto future = service_client->async_send_request(trigger_request);
        rclcpp::FutureReturnCode ret_code = rclcpp::spin_until_future_complete(client_node, future, std::chrono::milliseconds(2000));
        return std::make_pair((ret_code == rclcpp::FutureReturnCode::SUCCESS)? future.get() : nullptr, ret_code);
      }

      bool handleCallEmptySrv(const std::string& service_name);
      bool handleCallTriggerSrv(const std::string& service_name);

      //Services
      rclcpp::Service<LoadTreeSrv>::SharedPtr load_tree_srv_;
      rclcpp::Service<TreeRequestSrv>::SharedPtr stop_tree_srv_;
      rclcpp::Service<TreeRequestSrv>::SharedPtr kill_tree_srv_;
      rclcpp::Service<EmptySrv>::SharedPtr kill_all_trees_srv_;
      rclcpp::Service<TreeRequestSrv>::SharedPtr restart_tree_srv_; 
      rclcpp::Service<GetBBValuesSrv>::SharedPtr get_sync_bb_values_srv_; 
      rclcpp::Service<GetTreeStatusSrv>::SharedPtr get_tree_status_srv_;
      rclcpp::Service<GetAllTreeStatusSrv>::SharedPtr get_all_trees_status_srv_;
      rclcpp::Service<TreeRequestSrv>::SharedPtr pause_tree_srv_;
      rclcpp::Service<TreeRequestSrv>::SharedPtr resume_tree_srv_;

      //Service Clients
      // rclcpp::Client<EmptySrv>::SharedPtr kill_service_client_;
      // rclcpp::Client<EmptySrv>::SharedPtr stop_service_client_;
      // rclcpp::Client<EmptySrv>::SharedPtr restart_service_client_;
      // rclcpp::Client<TriggerSrv>::SharedPtr  pause_service_client_;
      // rclcpp::Client<TriggerSrv>::SharedPtr resume_service_client_;
      //Subscribers
      rclcpp::Subscription<BBEntry>::SharedPtr sync_bb_sub_;
      
      //Publishers
      rclcpp::Publisher<BBEntry>::SharedPtr sync_bb_pub_;

      //Sync BB
      BT::Blackboard::Ptr sync_blackboard_ptr_;

      //BT_FACTORY
      BT::BehaviorTreeFactory bt_factory_;

      //Save Spawned Trees information
      std::map <unsigned int, TreeProcessInfo> uids_to_tree_info_;

      //Manage spawn Process using ROS2 LAUNCH command
      ROS2LaunchManager ros2_launch_manager_;

      //Manage Trees_UIDs
      unsigned int trees_UID_ = 0;

      rclcpp::Node::SharedPtr node_ ;
      rclcpp::executors::MultiThreadedExecutor executor_;
  };
}
#endif