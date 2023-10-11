cmake_minimum_required(VERSION 3.16)
project(btcpp_ros2_samples)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)

set(THIS_PACKAGE_DEPS 
    behaviortree_ros2
    std_msgs
    btcpp_ros2_interfaces )

######################################################
# Build a client that call the sleep action (STATIC version)

add_executable(sleep_client 
  src/sleep_action.cpp
  src/sleep_client.cpp)
ament_target_dependencies(sleep_client ${THIS_PACKAGE_DEPS})

######################################################
# Build a client that call the sleep action (Plugin version)

add_library(sleep_plugin SHARED src/sleep_action.cpp)
target_compile_definitions(sleep_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(sleep_plugin ${THIS_PACKAGE_DEPS})

add_executable(sleep_client_dyn src/sleep_client.cpp)
target_compile_definitions(sleep_client_dyn PRIVATE USE_SLEEP_PLUGIN )
target_link_libraries(sleep_client_dyn sleep_plugin )
ament_target_dependencies(sleep_client_dyn ${THIS_PACKAGE_DEPS})

######################################################
# Build Server
add_executable(sleep_server src/sleep_server.cpp)
ament_target_dependencies(sleep_server ${THIS_PACKAGE_DEPS}) 

######################################################
# Build subscriber_test
add_executable(subscriber_test src/subscriber_test.cpp)
ament_target_dependencies(subscriber_test ${THIS_PACKAGE_DEPS}) 

######################################################
# INSTALL

install(TARGETS
  sleep_client
  sleep_client_dyn
  sleep_server
  sleep_plugin
  subscriber_test
  DESTINATION lib/${PROJECT_NAME}
  )


ament_export_dependencies(behaviortree_ros2 btcpp_ros2_interfaces)

ament_package()
