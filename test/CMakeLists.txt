
rosidl_generate_interfaces(behaviortree_ros2
    "../action/Sleep.action")

# macro to remove some boiler plate
function(add_target_dependencies target)
    target_include_directories(${target} PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
    ament_target_dependencies(${target} ${THIS_PACKAGE_INCLUDE_DEPENDS})
    rosidl_target_interfaces(${target} behaviortree_ros2 "rosidl_typesupport_cpp")
endfunction()

######################################################
# Compile Sleep library as static dependeny or plugin

add_library(sleep_plugin SHARED sleep_action.cpp)
target_compile_definitions(sleep_plugin PRIVATE  BT_PLUGIN_EXPORT )
add_target_dependencies(sleep_plugin)

add_library(sleep_static STATIC sleep_action.cpp)
add_target_dependencies(sleep_static)

######################################################
# Build two clients, one that use the plugin and the other
# that links statically

add_executable(sleep_client_dyn sleep_client.cpp)
target_compile_definitions(sleep_client_dyn PRIVATE USE_SLEEP_PLUGIN )
add_target_dependencies(sleep_client_dyn)

add_executable(sleep_client sleep_client.cpp)
target_link_libraries(sleep_client sleep_static )
add_target_dependencies(sleep_client)

######################################################
# Build Server
add_executable(sleep_server sleep_server.cpp)
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



