cmake_minimum_required(VERSION 3.16)
project(behaviortree_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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

add_library(${PROJECT_NAME} src/bt_ros2.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

######################################################
rosidl_generate_interfaces(${PROJECT_NAME}_SLEEP
    "action/Sleep.action")

# macro to remove some boiler plate
function(add_target_dependencies target)
    target_include_directories(${target} PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
    ament_target_dependencies(${target} ${THIS_PACKAGE_INCLUDE_DEPENDS})

    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}_SLEEP" "rosidl_typesupport_cpp")
    target_link_libraries(${target} "${cpp_typesupport_target}")
endfunction()

######################################################
# Compile Sleep library as static dependeny or plugin

add_library(sleep_plugin SHARED test/sleep_action.cpp)
target_compile_definitions(sleep_plugin PRIVATE  BT_PLUGIN_EXPORT )
add_target_dependencies(sleep_plugin)

add_library(sleep_static STATIC test/sleep_action.cpp)
add_target_dependencies(sleep_static)

######################################################
# Build two clients, one that use the plugin and the other
# that links statically

add_executable(sleep_client_dyn test/sleep_client.cpp)
target_compile_definitions(sleep_client_dyn PRIVATE USE_SLEEP_PLUGIN )
add_target_dependencies(sleep_client_dyn)

add_executable(sleep_client test/sleep_client.cpp)
target_link_libraries(sleep_client sleep_static )
add_target_dependencies(sleep_client)

######################################################
# Build Server
add_executable(sleep_server test/sleep_server.cpp)
add_target_dependencies(sleep_server)

######################################################
# INSTALL

install(TARGETS
  sleep_client
  sleep_client_dyn
  sleep_server
  sleep_plugin
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  )

install(
    TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
  

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(behaviortree_cpp rclcpp_action rclcpp)

ament_package()
