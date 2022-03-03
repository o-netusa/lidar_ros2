#cmake_minimum_required(VERSION 3.0.2)
cmake_minimum_required(VERSION 3.5)

project(common_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   std_msgs
#   message_generation
# )
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_generation REQUIRED)
add_message_files(
   FILES
   ParameterMsg.msg
 )
add_service_files(
    DIRECTORY srv
    FILES
    LidarRosService.srv
    )
generate_messages(
    DEPENDENCIES
      std_msgs
    )

# catkin_package(
#   CATKIN_DEPENDS message_runtime
# )
ament_package(
  AMENT_DEPENDS message_runtime
)

# include_directories(
#   ${catkin_INCLUDE_DIRS}
include_directories(include)




