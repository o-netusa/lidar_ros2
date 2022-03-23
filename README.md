# O-Net LiDAR ROS2 Driver

## Build within Docker container
* Start Docker container: `/opt/share/ros/start_ros_docker.sh $(pwd)`
* Inside the Docker container, cd into the repo directory and execute `colcon build`

## Build without Docker container
* cd into the repo directory and execute `colcon build`

## Launch the ROS node
* After building, set environment using `source install/setup.bash` 
* Add LD_LIBRARY_PATH using `export LD_LIBRARY_PATH=$(pwd)/install/lidar_ros_driver/lib/lidar_ros_driver:$LD_LIBRARY_PATH`
* Execute `ros2 launch lidar_ros_device run.launch.py`

