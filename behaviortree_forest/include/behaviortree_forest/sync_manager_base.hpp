#ifndef SYNC_MANAGER_BASE_HPP
#define SYNC_MANAGER_BASE_HPP

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_forest/sync_blackboard.hpp"

#include "behaviortree_forest_interfaces/msg/bb_entry.hpp"
#include "behaviortree_forest_interfaces/msg/bb_entries.hpp"
#include "behaviortree_eut_plugins/eut_factory.h"

using BBEntry = behaviortree_forest_interfaces::msg::BBEntry;
using BBEntries = behaviortree_forest_interfaces::msg::BBEntries;


namespace BT_SERVER
{
    class SyncManagerBase
    {

    public:
        SyncManagerBase(BT::Blackboard::Ptr blackboard, const std::string& bt_uid, const BT::EutBehaviorTreeFactory& eut_bt_factory);
        ~SyncManagerBase();

        virtual void syncBBUpdateCB(const BBEntries::SharedPtr sync_entries_upd_msg) = 0;
        /// @brief Publish sync entries that have been updated and are in TO_SYNC or SYNCING and have not ack yet
        /// @return 
        virtual void publishUpdatedSyncEntries() = 0;

        std::vector<std::string> getSyncKeysList();
        SyncMap getKeysValueToSync ();

        /*
            Retrieve sync entry returning empty argument if it does not exist
        */
        BT::Expected<SyncEntry> getSyncEntry(const std::string& key);
        bool hasSyncStatus(const std::string& key, SyncStatus sync_status);
        
        bool updateSyncMapEntrySyncStatus(const std::string& key, SyncStatus sync_status);
        bool updateSyncMapEntrySyncStatus(const std::string& key, SyncStatus expected_sync_status, SyncStatus new_sync_status);
        
        bool addToSyncMap(const std::string& bb_key, SyncStatus sync_status) ;

        bool processSyncEntryUpdate(const BBEntry& sync_entry_upd, const bool to_sync=false);

        void updateBlackboard(BT::Blackboard::Ptr blackboard);
        
        const BBEntries getSyncEntriesToPublish();

        static BT::Expected<BBEntry> buildBBEntryMsg(const std::string& bb_key, const BT::Blackboard::Ptr blackboard_ptr);
    protected:
        void resetLogger()
        {
            if(ros_logger_)
            {
                ros_logger_.reset();
            }
        }
        void setLogger(const rclcpp::Logger& logger)
        {
            if(logger.get_name())
            {
                ros_logger_ = std::make_shared<rclcpp::Logger>(logger);
            }
        }
    private:
        void refreshSyncMap();
        BT::Blackboard::Ptr blackboard_;
        const std::string bt_uid_;
        std::mutex syncMap_lock_;
        SyncMap syncMap_;

        const BT::EutBehaviorTreeFactory& eut_bt_factory_;

        std::shared_ptr<rclcpp::Logger> ros_logger_;
    };
} // namespace BT_SERVER
#endif // SYNC_MANAGER_BASE_HPP