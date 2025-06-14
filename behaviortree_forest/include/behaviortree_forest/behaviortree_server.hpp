#ifndef BEHAVIORTREE_SERVER_HPP
#define BEHAVIORTREE_SERVER_HPP

// Add these includes if not already present
#include <mutex>
#include <condition_variable>

#include <string>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "behaviortree_eut_plugins/eut_factory.h"

#include "behaviortree_forest_interfaces/msg/bb_entry.hpp"
#include "behaviortree_forest_interfaces/msg/bb_entries.hpp"
#include "behaviortree_forest_interfaces/msg/tree_execution_status.hpp"

#include "behaviortree_forest_interfaces/srv/load_tree.hpp"
#include "behaviortree_forest_interfaces/srv/get_tree_status_by_id.hpp"
#include "behaviortree_forest_interfaces/srv/get_tree_status.hpp"
#include "behaviortree_forest_interfaces/srv/get_bb_values.hpp"
#include "behaviortree_forest_interfaces/srv/tree_request.hpp"
#include "behaviortree_forest_interfaces/srv/tree_request_by_capability.hpp"
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
using BBEntries = behaviortree_forest_interfaces::msg::BBEntries;
using TreeStatus = behaviortree_forest_interfaces::msg::TreeExecutionStatus;
using LoadTreeSrv = behaviortree_forest_interfaces::srv::LoadTree;
using TreeRequestSrv = behaviortree_forest_interfaces::srv::TreeRequest;
using TreeRequestByCapabilitySrv = behaviortree_forest_interfaces::srv::TreeRequestByCapability;
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
      TreeProcessInfo(const std::string& in_tree_name, pid_t in_pid, const uint8_t current_exec_status=TreeStatus::UNKNOWN) :
        tree_name(in_tree_name),
        pid(in_pid)
      {
          killed = false;
          tree_status = TreeStatus{};
          tree_status.name = in_tree_name;
          tree_status.status = current_exec_status;
      };


      // Make these explicitly deleted to prevent accidental copying
      TreeProcessInfo(const TreeProcessInfo&) = delete;
      TreeProcessInfo& operator=(const TreeProcessInfo&) = delete;

      const pid_t pid;
      const std::string tree_name;
      TreeStatus tree_status; // extend tree status with LOADING
      bool killed;
      std::mutex loading_mutex;  // Mutex for the condition variable
      std::condition_variable loading_cv;  // Condition variable to wait for loading
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
      bool killTree(const uint32_t tree_uid, const bool force_kill = false);
      bool killAllTrees(const bool force_kill = false);
      bool killAllTreesCB(const std::shared_ptr<EmptySrv::Request> req, std::shared_ptr<EmptySrv::Response> res);
      
      bool pauseTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res);
      bool resumeTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res);
      bool restartTreeCB(const std::shared_ptr<TreeRequestSrv::Request> req, std::shared_ptr<TreeRequestSrv::Response> res);

      bool restartTreeByCapabilityCB(const std::shared_ptr<TreeRequestByCapabilitySrv::Request> req, std::shared_ptr<TreeRequestByCapabilitySrv::Response> res);

      bool getSyncBBValuesCB (const std::shared_ptr<GetBBValuesSrv::Request> req, std::shared_ptr<GetBBValuesSrv::Response> res);
      bool getTreeStatusCB(const std::shared_ptr<GetTreeStatusSrv::Request> req, std::shared_ptr<GetTreeStatusSrv::Response> res);
      bool getAllTreeStatusCB(const std::shared_ptr<GetAllTreeStatusSrv::Request> req, std::shared_ptr<GetAllTreeStatusSrv::Response> res);
      void treeStatusTopicCB(const TreeStatus::SharedPtr msg);
      void syncBBCB(const BBEntries::SharedPtr msg) const;
      bool syncBB(const BBEntry& msg) const;
      void republishUpdatedSyncEntries(const BBEntries& msg) const;
      void initBB(const std::string& abs_file_path, BT::Blackboard::Ptr blackboard_ptr);

      // Create a node to call the service
      // This is a workaround to avoid creating a new node for each service call
      // It is not the best practice, but it works for now
      template<typename T>
      std::pair<std::shared_ptr<typename T::Response>, rclcpp::FutureReturnCode> handleSyncSrvCall(const std::string& service_name, const typename T::Request& request = typename T::Request())
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
        auto trigger_request = std::make_shared<typename T::Request>(request);
        auto future = service_client->async_send_request(trigger_request);
        rclcpp::FutureReturnCode ret_code = rclcpp::spin_until_future_complete(client_node, future);
        std::string future_return_code = ret_code == rclcpp::FutureReturnCode::TIMEOUT ? "FutureReturnCode::TIMEOUT" : ret_code == rclcpp::FutureReturnCode::SUCCESS? "FutureReturnCode::SUCCESS" : "FutureReturnCode::INTEFRUPTED";
        RCLCPP_INFO(client_node->get_logger(), "Service %s called: future ret code: %s", service_name.c_str(), future_return_code.c_str());
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
      rclcpp::Service<TreeRequestByCapabilitySrv>::SharedPtr restart_tree_by_capability_srv_;
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
      rclcpp::Subscription<BBEntries>::SharedPtr sync_bb_sub_;
      
      //Publishers
      rclcpp::Publisher<BBEntries>::SharedPtr sync_bb_pub_;

      //Sync BB
      BT::Blackboard::Ptr sync_blackboard_ptr_;

      // trees update execution status
      rclcpp::CallbackGroup::SharedPtr trees_upd_cb_group_;

      //Save Spawned Trees information
      std::map <unsigned int, TreeProcessInfo> uids_to_tree_info_;
      
      
      //Manage Trees_UIDs
      unsigned int trees_UID_ = 0;
      
      rclcpp::Node::SharedPtr node_ ;
      rclcpp::executors::MultiThreadedExecutor executor_;
      
      //BT_FACTORY
      BT::EutBehaviorTreeFactory eut_bt_factory_;

      //Manage spawn Process using ROS2 LAUNCH command
      ROS2LaunchManager ros2_launch_manager_;

      // Capability to tree_uids map
      std::unordered_map<std::string, std::unordered_set<unsigned int>> capability_to_uids_map_;
  };
}
#endif