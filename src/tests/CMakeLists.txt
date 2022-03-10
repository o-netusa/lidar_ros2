include(CTest)
include(GoogleTest)

find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)

ament_add_gtest(lidar_ros_test DeviceTests.cpp)
target_include_directories(lidar_ros_test PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(lidar_ros_test ${rclcpp_LIBRARIES} ${GTEST_LIBRARIES} lidar_ros)
