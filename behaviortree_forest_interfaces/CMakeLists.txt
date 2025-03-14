cmake_minimum_required(VERSION 3.16)
project(behaviortree_forest_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(behaviortree_forest_interfaces
    "msg/BBEntry.msg"
    "msg/BBEntries.msg"
    "msg/TreeExecutionStatus.msg"
    "msg/Transition.msg"
    "srv/GetAllTreesStatus.srv"
    "srv/GetBBValues.srv"
    "srv/GetTreeStatus.srv"
    "srv/GetTreeStatusByID.srv"
    "srv/LoadTree.srv"
    "srv/TreeRequest.srv"
    "srv/GetLoadedPlugins.srv"
    DEPENDENCIES builtin_interfaces)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
