find_package(ament_cmake_gmock REQUIRED)
find_package(std_msgs REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

ament_add_gmock(test_${PROJECT_NAME} test_bt_topic_sub_node.cpp)
target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
ament_target_dependencies(test_${PROJECT_NAME} std_msgs)
