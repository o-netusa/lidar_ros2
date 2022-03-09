add_library(lidar_ros LidarRosDriver.cpp)
target_compile_features(lidar_ros PRIVATE ${CXX_STD})
#target_link_libraries(lidar_ros PUBLIC lidardevice eigen sigslot stduuid  ${catkin_LIBRARIES})
# add_dependencies(lidar_ros common_msgs_generate_messages_cpp)
#ament_target_dependencies(lidar_ros PUBLIC sigslot stduuid  ${catkin_LIBRARIES})
#target_link_libraries(lidar_ros PUBLIC lidardevice eigen sigslot stduuid)

ament_target_dependencies(lidar_ros rclcpp)
#target_link_libraries(lidar_ros PUBLIC lidardevice eigen sigslot stduuid)

add_executable(lidar_ros_driver LidarRosNode.cpp)
target_compile_features(lidar_ros_driver PRIVATE ${CXX_STD})
target_link_libraries(lidar_ros_driver PUBLIC lidar_ros eigen sigslot stduuid  ${catkin_LIBRARIES})



# # copy config and launch files to devel
# add_custom_command(TARGET lidar_ros_driver POST_BUILD
#   COMMAND ${CMAKE_COMMAND} -E copy_directory
#     ${CMAKE_CURRENT_SOURCE_DIR}/../config
#     # ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/lidar_ros_driver/config
#     ${CATKIN_DEVEL_PREFIX}/ros2_foxy/lidar_ros_driver/config
#   COMMAND ${CMAKE_COMMAND} -E copy_directory
#     ${CMAKE_CURRENT_SOURCE_DIR}/../launch
#     ${CATKIN_DEVEL_PREFIX}/launch
# )

# install targets
# install(TARGETS lidar_ros DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS lidar_ros DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
    # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION bin/${PROJECT_NAME}
)
# install(FILES ${lidardevice_SOURCE_DIR}/lib/libLidarDevice.so DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES ${lidardevice_SOURCE_DIR}/lib/libLidarDevice.so DESTINATION lib/${PROJECT_NAME})

# install config and launch files
# install(DIRECTORY ${CMAKE_SOURCE_DIR}/config DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/lidar_ros_driver)
# install(FILES ${CMAKE_SOURCE_DIR}/launch/run.launch DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/../launch)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/config DESTINATION lib/${PROJECT_NAME}/lidar_ros_driver)
install(FILES ${CMAKE_SOURCE_DIR}/launch/run.launch DESTINATION lib/${PROJECT_NAME}/../launch)

# install license and readme
# install(FILES ${CMAKE_SOURCE_DIR}/../LICENSE.txt DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/../)
# install(FILES ${CMAKE_SOURCE_DIR}/../README.md DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/../)
install(FILES ${CMAKE_SOURCE_DIR}/../LICENSE.txt DESTINATION lib/${PROJECT_NAME}/../)
install(FILES ${CMAKE_SOURCE_DIR}/../README.md DESTINATION lib/${PROJECT_NAME}/../)

