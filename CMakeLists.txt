cmake_minimum_required(VERSION 3.16)
project(behaviortree_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

######################################################

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_action
    ament_index_cpp
    behaviortree_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED )
find_package(rclcpp_action REQUIRED )
find_package(behaviortree_cpp REQUIRED )
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

######################################################

include_directories(
  include
)

add_library(behavior_tree_engine SHARED
  src/behavior_tree_engine.cpp
)
ament_target_dependencies(behavior_tree_engine
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_library(behavior_tree_engine_old SHARED
  src/behavior_tree_engine_old.cpp
)
ament_target_dependencies(behavior_tree_engine_old
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_library(pipeline_sequence_bt_node SHARED plugins/control/pipeline_sequence.cpp)
list(APPEND plugin_libs pipeline_sequence_bt_node)

add_library(rate_controller_bt_node SHARED plugins/decorator/rate_controller.cpp)
list(APPEND plugin_libs rate_controller_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

ament_export_libraries(
  behavior_tree_engine
  behavior_tree_engine_old
  ${plugin_libs}
)

######################################################
# INSTALL

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS 
  behavior_tree_engine
  behavior_tree_engine_old
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

######################################################

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_dependencies(rosidl_default_runtime)
ament_package()

