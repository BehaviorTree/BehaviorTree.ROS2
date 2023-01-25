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

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Sleep.action"
)


######################################################

######################################################
# TESTS

add_executable(sleep_server test/sleep_server.cpp)
target_include_directories(sleep_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(sleep_server
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
rosidl_target_interfaces(sleep_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

######################################################
# Decide if Sleep action should be a plugin or not

set(SLEEP_PLUGIN 0) # set to 1 to make it a plugin

if(SLEEP_PLUGIN)
    set(LIBRARY_TYPE SHARED)
else()
    set(LIBRARY_TYPE STATIC)
endif()

add_library(sleep_action ${LIBRARY_TYPE} test/sleep_action.cpp)
target_include_directories(sleep_action PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(sleep_action ${THIS_PACKAGE_INCLUDE_DEPENDS})
rosidl_target_interfaces(sleep_action ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(SLEEP_PLUGIN)
    target_compile_definitions(sleep_action PRIVATE  BT_PLUGIN_EXPORT )
endif()

######################################################
# Build Executable
add_executable(test_client test/test_client.cpp)
if(SLEEP_PLUGIN)
     target_compile_definitions(test_client PRIVATE USE_SLEEP_PLUGIN )
else()
    target_link_libraries(test_client sleep_action )
endif()
target_include_directories(test_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(test_client
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
rosidl_target_interfaces(test_client ${PROJECT_NAME} "rosidl_typesupport_cpp")


#add_executable(test_server test/test_server.cpp)
#add_dependencies(test_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_server ${catkin_LIBRARIES} )

######################################################
# INSTALL

install(TARGETS
  sleep_server
  test_client
  sleep_action
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  )

install(
  DIRECTORY include/
  DESTINATION include
)

######################################################

ament_export_include_directories(include)

ament_package()


