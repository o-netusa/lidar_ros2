cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

find_package(rclcpp REQUIRED)

add_executable(test_node test_node.cpp)

target_include_directories(test_node PRIVATE ${CMAKE_SOURCE_DIR}/lidar_ros_driver)
target_link_libraries(test_node PUBLIC ${rclcpp_LIBRARIES})
