cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

add_library(lidar_ros LidarRosDriver.cpp)
target_compile_features(lidar_ros PRIVATE ${CXX_STD})
target_link_libraries(lidar_ros PUBLIC lidardevice ${rclcpp_LIBRARIES} ${rosbag2_cpp_LIBRARIES} ${rosbag2_storage_LIBRARIES} ${sensor_msgs_LIBRARIES})

add_executable(lidar_ros_driver LidarRosNode.cpp)
target_compile_features(lidar_ros_driver PRIVATE ${CXX_STD})
target_link_libraries(lidar_ros_driver PUBLIC lidar_ros)


# install targets
install(TARGETS lidar_ros DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(FILES ${lidardevice_SOURCE_DIR}/lib/libLidarDevice.so DESTINATION lib/${PROJECT_NAME})

# install config and launch files
install(DIRECTORY ${CMAKE_SOURCE_DIR}/config DESTINATION lib/${PROJECT_NAME})
install(FILES ${CMAKE_SOURCE_DIR}/launch/run.launch.py DESTINATION .)

# install license and readme
install(FILES ${CMAKE_SOURCE_DIR}/../LICENSE.txt DESTINATION lib/${PROJECT_NAME}/../)
install(FILES ${CMAKE_SOURCE_DIR}/../README.md DESTINATION lib/${PROJECT_NAME}/../)
