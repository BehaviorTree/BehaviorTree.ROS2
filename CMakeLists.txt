cmake_minimum_required(VERSION 3.5)
project(behaviortree_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

######################################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED )
find_package(rclcpp_action REQUIRED )
find_package(behaviortree_cpp REQUIRED )
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost COMPONENTS coroutine REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_action
    ament_index_cpp
    behaviortree_cpp
    geometry_msgs
    std_msgs
    Boost)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/behavior_tree_engine.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_library(nav2_pipeline_sequence_bt_node SHARED plugins/control/pipeline_sequence.cpp)
list(APPEND plugin_libs nav2_pipeline_sequence_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS 
  ${PROJECT_NAME}
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
  ${plugin_libs}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
