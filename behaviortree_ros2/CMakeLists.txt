cmake_minimum_required(VERSION 3.16)
project(behaviortree_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THIS_PACKAGE_DEPS
    rclcpp
    rclcpp_action
    ament_index_cpp
    behaviortree_cpp
    btcpp_ros2_interfaces
    generate_parameter_library
    )

find_package(ament_cmake REQUIRED)

foreach(PACKAGE ${THIS_PACKAGE_DEPS})
    find_package(${PACKAGE} REQUIRED )
endforeach()


generate_parameter_library(
    bt_executor_parameters
    src/bt_executor_parameters.yaml)

add_library(${PROJECT_NAME}
    src/bt_ros2.cpp
    src/bt_utils.cpp
    src/tree_execution_server.cpp )

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPS})

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}  bt_executor_parameters)

if (BUILD_TESTING)
    add_subdirectory(test)
endif()

######################################################
# INSTALL

install(DIRECTORY include/  DESTINATION include/)

ament_export_targets(${PROJECT_NAME}Targets  HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPS})

install(
  TARGETS ${PROJECT_NAME} bt_executor_parameters
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
