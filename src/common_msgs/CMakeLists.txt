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
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# add_message_files(
#    FILES
#    ParameterMsg.msg
#  )sor 
set(msg_files "msg/ParameterMsg.msg")
# add_service_files(
#     DIRECTORY srv
#     FILES
#     LidarRosService.srv
#     )
set(srv_file "srv/LidarRosService.srv")
# generate_messages(
#     DEPENDENCIES
#       std_msgs
#     )
rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        ${srv_file}
        DEPENDENCIES builtin_interfaces)

# catkin_package(
#   CATKIN_DEPENDS message_runtime
# )


# include_directories(
#   # ${catkin_INCLUDE_DIRS}
#   )

# include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_export_dependencies(rosidl_default_generators)

ament_export_include_directories(rosidl_default_generators)

ament_package()



