cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(lidar_ros_driver VERSION 1.0 LANGUAGES CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-std=c++17 ")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")

include(DownloadProject)
add_subdirectory(external)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcpputils REQUIRED)



###########
## Build ##
###########
include_directories(
  ${CPPBASE_INCLUDE_DIR}
)

add_subdirectory(lidar_ros_driver)
# add_subdirectory(test_node)

#############
## Testing ##
#############
if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()

ament_export_dependencies(sensor_msgs rosbag2)
ament_package()
