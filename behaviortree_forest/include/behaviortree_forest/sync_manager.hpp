#ifndef SYNC_MANAGER_HPP
#define SYNC_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_forest/sync_blackboard.hpp"

#include "behaviortree_forest_interfaces/msg/bb_entry.hpp"
#include "behaviortree_forest_interfaces/msg/bb_entries.hpp"
#include "behaviortree_eut_plugins/eut_factory.h"

using BBEntry = behaviortree_forest_interfaces::msg::BBEntry;
using BBEntries = behaviortree_forest_interfaces::msg::BBEntries;

namespace BT_SERVER
{
    class SyncManager
    {

    public:
        SyncManager(const rclcpp::Node::SharedPtr& node, BT::Blackboard::Ptr blackboard, const std::string& bt_uid, const BT::EutBehaviorTreeFactory& eut_bt_factory);
        ~SyncManager();

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
        // bool rmFromSyncMap(const std::string& bb_key) ;
        

        void syncBBUpdateCB(const BBEntries::SharedPtr sync_entries_upd_msg);
        bool processSyncEntryUpdate(const BBEntry& sync_entry_upd, const bool to_sync=false);

        /// @brief Publish sync entries that have been updated and are in TO_SYNC or SYNCING and have not ack yet
        /// @return 
        void publishUpdatedSyncEntries();

        static BT::Expected<BBEntry> buildBBEntryMsg(const std::string& bb_key, const BT::Blackboard::Ptr blackboard_ptr);

    private:
        void refreshSyncMap();
        rclcpp::Node::SharedPtr node_;
        BT::Blackboard::Ptr blackboard_;
        const std::string bt_uid_;
        std::mutex syncMap_lock_;
        SyncMap syncMap_;

        const BT::EutBehaviorTreeFactory& eut_bt_factory_;


        rclcpp::CallbackGroup::SharedPtr bb_upd_cb_group_;

        //Publishers
        rclcpp::Publisher<BBEntries>::SharedPtr sync_bb_pub_;

        //Subscribers
        rclcpp::Subscription<BBEntries>::SharedPtr sync_bb_sub_;
    };
} // namespace BT_SERVER
#endif // SYNC_MANAGER_HPP