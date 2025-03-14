cmake_minimum_required(VERSION 3.16)
project(behaviortree_forest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THIS_PACKAGE_DEPS
    rclcpp
    rclcpp_action
    ament_index_cpp
    std_srvs
    behaviortree_cpp
    btcpp_ros2_interfaces
    generate_parameter_library
    behaviortree_ros2
    behaviortree_forest_interfaces
    )

find_package(ament_cmake REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

foreach(PACKAGE ${THIS_PACKAGE_DEPS})
    find_package(${PACKAGE} REQUIRED )
endforeach()


add_executable(behaviortree_server
    src/behaviortree_server.cpp )
ament_target_dependencies(behaviortree_server ${THIS_PACKAGE_DEPS})

target_include_directories(behaviortree_server PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

list(APPEND ${PROJECT_NAME}_targets behaviortree_server)

add_executable(behaviortree_node
    src/behaviortree_node.cpp 
    src/tree_wrapper.cpp
    src/bt_transition_logger.cpp
    src/sync_manager.cpp)


ament_target_dependencies(behaviortree_node ${THIS_PACKAGE_DEPS})
target_include_directories(behaviortree_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(behaviortree_node yaml-cpp Boost::filesystem)

######################################################


add_library(behaviortree_forest_lib
    src/sync_manager.cpp
)
target_include_directories(behaviortree_forest_lib PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)

ament_target_dependencies(behaviortree_forest_lib ${THIS_PACKAGE_DEPS}) 

target_link_libraries(behaviortree_forest_lib
    rclcpp
    yaml-cpp
    Boost::filesystem
)

######################################################
# INSTALL

install(DIRECTORY include/  DESTINATION include/)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets  HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPS})

install(
  TARGETS behaviortree_server behaviortree_node behaviortree_forest_lib
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

# Export the library for dependent packages
ament_export_targets(behaviortree_forestTargets HAS_LIBRARY_TARGET)
ament_export_libraries(behaviortree_forest_lib)

ament_package()
