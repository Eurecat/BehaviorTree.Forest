 
#ifndef SYNC_BB_HPP
#define SYNC_BB_HPP

#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

#include "behaviortree_eut_plugins/utils/eut_utils.h"

namespace BT_SERVER
{
    enum class SyncStatus : uint8_t {
        TO_SYNC = 1,
        SYNCING = 2,
        SYNCED = 3
    };
    
    inline std::string toStr(SyncStatus value)
    {
        switch(value)
        {
        case SyncStatus::TO_SYNC: return "TO_SYNC";
        case SyncStatus::SYNCING: return "SYNCING";
        case SyncStatus::SYNCED:  return "SYNCED";
        default: return "UNKNOWN";
        }
    }

    struct SyncEntry
    {
      std::shared_ptr<BT::Blackboard::Entry> entry;
      SyncStatus status;
      std::chrono::nanoseconds last_synced;
      mutable std::mutex sync_entry_mutex;

      SyncEntry() :
        entry(nullptr),
        status(SyncStatus::SYNCED)
      {}


      SyncEntry(const std::shared_ptr<BT::Blackboard::Entry> entry_ptr, SyncStatus sync_status) :
        entry(entry_ptr),
        status(sync_status)
      {
        last_synced = entry_ptr? entry_ptr->stamp - std::chrono::nanoseconds{1} : std::chrono::nanoseconds{ 0 };
      }

      // Delete Copy Constructor (mutex is not copyable)
      SyncEntry(const SyncEntry&) = delete;

      // Define Move Constructor
      SyncEntry(SyncEntry&& other) noexcept
          : entry(std::move(other.entry)), 
            status(other.status), 
            last_synced(other.last_synced) 
      {
          // Note: std::mutex is left uninitialized, which is safe
      }

      // Delete Copy Assignment Operator
      SyncEntry& operator=(const SyncEntry&) = delete;

      // Define Move Assignment Operator
      SyncEntry& operator=(SyncEntry&& other) noexcept
      {
          if (this != &other)
          {
              entry = std::move(other.entry);
              status = other.status;
              last_synced = other.last_synced;
              // std::mutex is not moved
          }
          return *this;
      }

      bool updateSyncStatus(SyncStatus sync_status)
      {
        std::scoped_lock lock_entry(sync_entry_mutex);
        status = sync_status;
        if(status == SyncStatus::SYNCED) last_synced = std::chrono::steady_clock::now().time_since_epoch();
        return true;
      }

      // Update only if expected_sync_status == current_status
      bool updateSyncStatus(SyncStatus expected_sync_status, SyncStatus new_sync_status)
      {
        std::scoped_lock lock_entry(sync_entry_mutex);
        if(expected_sync_status == status)
        {
          status = new_sync_status;
          if(status == SyncStatus::SYNCED) last_synced = std::chrono::steady_clock::now().time_since_epoch();
          return true;
        }
        else
          return false;
      }

      SyncStatus syncStatus() const
      {
        std::scoped_lock lock_entry(sync_entry_mutex);
        return status;
      }

      /*
        Return safe copy of original
      */
      SyncEntry getSafeCopy() const
      {
        std::scoped_lock lock_entry(sync_entry_mutex);
        return SyncEntry(entry, status);
      }
    };

    typedef std::unordered_map<std::string, SyncEntry> SyncMap;
  
    static bool isCandidateKey(const std::string& str, int i)
    {
      int size = static_cast<int>(str.size());
      return (size-i >= 5 && str[i] == '$' && str[i+1] == '$' && str[i+2] == '{') || 
              (size-i >= 3 && str[i] == '{');
    }

    static bool isSharedBlackboardPointer(const std::string& key)
    {
        //Sync Keys Format: ${}
        const auto size = key.size();
        if (size >= 5 && key.back() == '}')
        {
          if (key[0] == '$' && key[1] == '$' && key[2] == '{')
          {
            return true;
          }
        }
        return false;
    }

    inline std::string_view stripBlackboardPointer(std::string_view str, bool shared_bb_ptr = false)
    {
      const auto size = str.size();
      if (size >= 3 && str.back() == '}')
      {
        if(shared_bb_ptr && str[0] == '$' && str[1] == '$' && str[2] == '{')
        {
          return str.substr(3, size - 4);
        }
      }

      return BT::TreeNode::stripBlackboardPointer(str);
    }

  /**
   * Replace every key reference with its string value, throws if even one referenced key does not exist
  */
  inline BT::Expected<std::string> replaceKeysWithStringValues(const std::string& boosted_key, BT::Blackboard::Ptr bb_ptr, const bool fail_with_invalid_reference = false)
  {
      std::string res = "";
      int size = static_cast<int>(boosted_key.size());
      for(int i=0; i<size; i++)
      {
          bool is_candidate = isCandidateKey(boosted_key, i);

          if (is_candidate)//we might have a candidate
          {
              bool bb_shared_ptr_candidate = (boosted_key[i] == '$');
              int j = (boosted_key[i] == '$')? i+3 : i+1;
              i = j-1;
              while(is_candidate && j < size && boosted_key[j] != '}')
              {
                is_candidate = !isCandidateKey(boosted_key, j); //keep being candidate if no other initial key char is found
                j++;
              }
              if(is_candidate && j < size)
              {
                //valid candidate
                std::string key = static_cast<std::string>(BT_SERVER::stripBlackboardPointer(boosted_key.substr(i, j-i+1), bb_shared_ptr_candidate));
                const auto possible_str_value = BT::EutUtils::getEntryAsString(key, bb_ptr);
                if(possible_str_value)
                {
                  res += possible_str_value.value();
                }
                else
                {
                  // if getAsString fails, i.e. value not found in the BB
                  if(fail_with_invalid_reference)//stricter version stop process
                  {
                    return nonstd::make_unexpected(BT::StrCat("replaceKeysWithStringValues: value for key [",key,"] not found in its replacement in ", boosted_key));
                  }
                  // more relaxed approach: copy and paste reference without substitution
                  res += boosted_key.substr(i, j-i+1);
                }
                
                i = j;
                continue;
              }
          }

          res += boosted_key[i];
      }
      return res;
  }
}
#endif