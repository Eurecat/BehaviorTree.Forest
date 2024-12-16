 
#ifndef SYNC_BB_HPP
#define SYNC_BB_HPP
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

#include "behavior_tree_eut_plugins/eut_debug.h"

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

    typedef std::pair<SyncStatus, std::shared_ptr<BT::Blackboard::Entry>> SyncEntry;
    typedef std::unordered_map<std::string, SyncEntry> SyncMap;
  
    static bool isCandidateKey(const std::string& str, int i, bool check_strict=false)
    {
      int size = static_cast<int>(str.size());
      return (size-i >= 4 && str[i] == '$' && str[i+1] == '{') || (!check_strict && size-i >= 3 && str[i] == '{');
    }

    static bool isSharedBlackboardPointer(const std::string& key)
    {
        //Sync Keys Format: ${}
        const auto size = key.size();
        if (size >= 4 && key.back() == '}')
        {
          if (key[0] == '$' && key[1] == '{')
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
        if(shared_bb_ptr && str[0] == '$' && str[1] == '{')
        {
          return str.substr(3, size - 4);
        }

        if (str[0] == '{')
        {
          return str.substr(1, size - 2);
        }
        if (str[0] == '$' && str[1] == '{')
        {
          return str.substr(2, size - 3);
        }
      }
      return {};
    }

    /**
   * Replace every key reference with its string value, throws if even one referenced key does not exist
  */
  inline BT::Expected<std::string> replaceKeysWithStringValues(const std::string& boosted_key, BT::Blackboard::Ptr bb_ptr, const bool check_strict = false, const bool fail_with_invalid_reference = false)
  {
      std::string res = "";
      int size = static_cast<int>(boosted_key.size());
      for(int i=0; i<size; i++)
      {
          bool is_candidate = isCandidateKey(boosted_key, i, check_strict);

          if (is_candidate)//we might have a candidate
          {
              int j = (boosted_key[i] == '$')? i+2 : i+1;
              i = j-1;
              while(is_candidate && j < size && boosted_key[j] != '}')
              {
                is_candidate = !isCandidateKey(boosted_key, j, check_strict); //keep being candidate if no other initial key char is found
                j++;
              }
              if(is_candidate && j < size)
              {
                //valid candidate
                std::string key = static_cast<std::string>(stripBlackboardPointer(boosted_key.substr(i, j-i+1)));
                const auto possible_str_value = BT::getEntryAsString(key, bb_ptr);
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