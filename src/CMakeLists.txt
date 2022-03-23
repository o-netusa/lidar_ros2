cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(lidar_ros_driver VERSION 1.0 LANGUAGES CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CXX_STD cxx_std_17)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")

include(DownloadProject)
add_subdirectory(external)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcpputils REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)


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

ament_export_dependencies(sensor_msgs rosbag2 pcl_ros pcl_conversions)
ament_package()
