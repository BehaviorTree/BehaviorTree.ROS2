cmake_minimum_required(VERSION 3.8)
project(bt_tutorials_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

# include directories
include_directories(
  include
)

# add libraries
add_library(${PROJECT_NAME}_action_client src/${PROJECT_NAME}/${PROJECT_NAME}_action_client.cpp)
add_library(${PROJECT_NAME}_service_client src/${PROJECT_NAME}/${PROJECT_NAME}_service_client.cpp)

# link libraries against ros dependencies
ament_target_dependencies(${PROJECT_NAME}_action_client rclcpp behaviortree_cpp behaviortree_ros2 rclcpp_action action_tutorials_interfaces)
ament_target_dependencies(${PROJECT_NAME}_service_client rclcpp behaviortree_cpp behaviortree_ros2 example_interfaces)

# add executables
add_executable(${PROJECT_NAME}_action_client_node src/${PROJECT_NAME}_action_client_node.cpp)
add_executable(${PROJECT_NAME}_service_client_node src/${PROJECT_NAME}_service_client_node.cpp)

# link executables against libraries
target_link_libraries(${PROJECT_NAME}_action_client_node ${PROJECT_NAME}_action_client)
target_link_libraries(${PROJECT_NAME}_service_client_node ${PROJECT_NAME}_service_client)

# link executables against ros dependencies
ament_target_dependencies(${PROJECT_NAME}_action_client_node rclcpp behaviortree_cpp)
ament_target_dependencies(${PROJECT_NAME}_service_client_node rclcpp behaviortree_cpp)

# install include directories
install(
  DIRECTORY include
  DESTINATION include
)

# install libraries
install(
  TARGETS ${PROJECT_NAME}_action_client ${PROJECT_NAME}_service_client
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# install executables
install(
  TARGETS ${PROJECT_NAME}_action_client_node ${PROJECT_NAME}_service_client_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
