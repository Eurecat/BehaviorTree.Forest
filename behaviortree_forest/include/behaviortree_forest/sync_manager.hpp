#ifndef SYNC_MANAGER_HPP
#define SYNC_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_forest/sync_blackboard.hpp"

namespace BT_SERVER
{
    class SyncManager
    {

    public:
        SyncManager(const rclcpp::Node::SharedPtr& node);
        ~SyncManager();

        std::vector<std::string> getSyncKeysList();
        SyncMap getKeysValueToSync ();
        bool updateSyncMapEntrySyncStatus(const std::string& key, SyncStatus sync_status);
        bool updateSyncMapEntrySyncStatus(const std::string& key, SyncStatus expected_sync_status, SyncStatus new_sync_status) ;
        bool addToSyncMap(const std::string& bb_key, SyncStatus sync_status, BT::Blackboard::Ptr blackboard_) ;
        std::pair<bool, SyncEntry> checkForSyncKey(const std::string& key);
        bool checkSyncStatus(const std::string& key, SyncStatus sync_status);
        void checkForToSyncEntries(BT::Blackboard::Ptr blackboard_);
    private:
        rclcpp::Node::SharedPtr node_;
        std::mutex syncMap_lock_;
        SyncMap syncMap_;
    };
} // namespace BT_SERVER
#endif // SYNC_MANAGER_HPP