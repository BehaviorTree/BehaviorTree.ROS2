cmake_minimum_required(VERSION 3.16)
project(behaviortree_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(THIS_PACKAGE_DEPS
    rclcpp
    rclcpp_action
    ament_index_cpp
    behaviortree_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED )
find_package(rclcpp_action REQUIRED )
find_package(behaviortree_cpp REQUIRED )
find_package(ament_index_cpp REQUIRED)

# This is compiled only to check if there are errors in the header file
# library will not be exported
include_directories(include)
add_library(${PROJECT_NAME} src/bt_ros2.cpp)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPS})

######################################################
# INSTALL

install(DIRECTORY include/  DESTINATION include/) 
        
ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_DEPS})

ament_package()
