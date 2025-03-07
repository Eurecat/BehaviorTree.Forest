#ifndef BTFOREST_UTILS_HPP
#define BTFOREST_UTILS_HPP

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <regex>
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"


#include <ament_index_cpp/get_resources.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

// ASK DAVID why this?
// inline std::string const boolToString(bool b)
// {
//   return b ? "true" : "false";
// }

inline std::string getTreeFullPath(const std::string& _file, const std::string& trees_folder_)
{
    std::string full_name = _file;
    const char* home = getenv("HOME");

    full_name = full_name.front() == '~' ? std::string(home) + full_name.substr(1, full_name.size() - 1) : full_name;
    full_name = full_name.front() == '/' ? full_name : (trees_folder_.back() == '/' ? trees_folder_ : trees_folder_ + "/") + full_name;

    std::size_t pos = full_name.find_last_of('.');
    if (pos == std::string::npos)
        full_name += ".xml";
    else if (full_name.substr(pos) != ".xml")
        full_name += ".xml";

    return full_name;
}

// ASK DAVID and check in the library if there is no already this function...
inline bool loadXMLToString(const std::string& filename, std::string& output) 
{
    // Open the file
    std::ifstream file(filename);
    
    // Check if the file is open
    if (!file.is_open()) {
        //std::cerr << "Failed to open the file: " << filename << std::endl;
        return false;
    }

    // Read the entire file into a stringstream
    std::stringstream buffer;
    buffer << file.rdbuf();
    
    // Store the contents into the output string
    output = buffer.str();

    // Close the file
    file.close();
    
    return true;
}

inline std::vector<std::string> extractSyncKeys(std::string& input) {
    // Regular expression to match ${...}
    std::regex pattern(R"(\$\$(\{([^}]+)\}))");
    // Iterator to search for matches in the input string
    std::smatch matches;

    std::vector<std::string> sync_keys ;

    // Search through the string for the pattern
    std::string::const_iterator searchStart(input.cbegin());
    while (std::regex_search(searchStart, input.cend(), matches, pattern)) {
        // The content inside the {...} will be in the second capture group (index 1)
        sync_keys.push_back(matches[2]);
        // Move the search start to continue from the next match
        searchStart = matches.suffix().first;
    }

    // Now replace the string ${...} with {...}
    input = std::regex_replace(input, pattern, R"({$2})");
    return sync_keys;
}

/*
inline BT::PortsList getPrimitivesTypesInfoDictionary()
{
    BT::PortsList portListDict;
    auto factory = BT::BehaviorTreeFactory();
    for(const auto& manifest : factory.manifests())
    {
        for(const auto& portinfo : manifest.second.ports)
        if(portListDict.find(portinfo.first) == portListDict.end())
            portListDict.insert(std::make_pair(portinfo.first, portinfo.second));
    }
    portListDict.insert(std::make_pair(BT::demangle(typeid(bool)), BT::BidirectionalPort<bool>("bool")).second);
    portListDict.insert(std::make_pair(BT::demangle(typeid(signed char)), BT::BidirectionalPort<int8_t>("signed char")).second);
    portListDict.insert(std::make_pair(BT::demangle(typeid(unsigned char)), BT::BidirectionalPort<uint8_t>("unsigned char")).second);
    portListDict.insert(std::make_pair(BT::demangle(typeid(short)), BT::BidirectionalPort<int16_t>("short")).second);
    portListDict.insert(std::make_pair(BT::demangle(typeid(unsigned short)), BT::BidirectionalPort<uint16_t>("unsigned short")).second);
    portListDict.insert(std::make_pair(BT::demangle(typeid(int)), BT::BidirectionalPort<int32_t>("int")).second);
    portListDict.insert(std::make_pair(BT::demangle(typeid(unsigned int)), BT::BidirectionalPort<uint32_t>("unsigned int")).second);
    portListDict.insert(std::make_pair(BT::demangle(typeid(long)), BT::BidirectionalPort<int64_t>("long")).second);
    portListDict.insert(std::make_pair(BT::demangle(typeid(unsigned long)), BT::BidirectionalPort<uint64_t>("unsigned long")).second);
    portListDict.insert(std::make_pair(BT::demangle(typeid(float)), BT::BidirectionalPort<float>("float")).second);
    portListDict.insert(std::make_pair(BT::demangle(typeid(double)), BT::BidirectionalPort<double>("double")).second);
    portListDict.insert(std::make_pair(BT::demangle(typeid(std::string)), BT::BidirectionalPort<std::string>("string")).second);
    return portListDict;
}

inline BT::TypeInfo createTypeInfoFromType(const std::string type)
{
  if(type == "bool")
  {
    return BT::TypeInfo::Create<bool>();
  }
  if(type == "char")
  {
    return BT::TypeInfo::Create<uint8_t>();
  }
  if(type == "byte")
  {
    return BT::TypeInfo::Create<int8_t>();
  }
  if(type == "unsigned short")
  {
    return BT::TypeInfo::Create<uint16_t>();
  }
  if(type == "short")
  {
    return BT::TypeInfo::Create<int16_t>();
  }
  if(type == "unsigned int")
  {
    return BT::TypeInfo::Create<uint32_t>();
  }
  if(type == "int")
  {
    return BT::TypeInfo::Create<int32_t>();
  }
  if(type == "unsigned long")
  {
    return BT::TypeInfo::Create<uint64_t>();
  }
  if(type == "long")
  {
    return BT::TypeInfo::Create<int64_t>();
  }
  if(type == "double")
  {
    return BT::TypeInfo::Create<double>();
  }
  if(type == "float")
  {
    return BT::TypeInfo::Create<float>();
  }
  if(type == "string")
  {
    return BT::TypeInfo::Create<std::string>();
  }
  //if(type == "AnyTypeAllowed")
  {
    return BT::TypeInfo();
  }
}

*/

inline std::vector<std::string> getBTPluginsFolders()
{
  std::vector<std::string> discovered_btplugins_folders;
  std::map<std::string, std::string> packages = ament_index_cpp::get_resources("packages");
  for (auto const& [pkg_name, ws_path] : packages) 
  {
      try 
      {
          std::string bt_plugins_path = ament_index_cpp::get_package_share_directory(pkg_name) + "/bt_plugins";

          if (std::filesystem::exists(bt_plugins_path) && std::filesystem::is_directory(bt_plugins_path)) 
          {
            discovered_btplugins_folders.push_back(pkg_name + "/bt_plugins");
          }
      } catch (const std::exception &e) 
      {
          
      }
  }
  return discovered_btplugins_folders;

}


#endif
