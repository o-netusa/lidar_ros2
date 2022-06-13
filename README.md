# O-Net LiDAR ROS2 Driver
* The driver is designed for ROS2 galactic (Ubuntu18.04)
* The driver works for x86_64, arm, aarch64

## Build within Docker container
* Start Docker container: `/opt/share/ros/start_ros_docker.sh $(pwd)`
* Inside the Docker container, cd into the repo directory and execute `colcon build`

## Build without Docker container
* cd into the repo directory and execute `colcon build`

## Launch the ROS node
* After building, set environment using `source install/setup.bash` 
* Add LD_LIBRARY_PATH using `export LD_LIBRARY_PATH=$(pwd)/install/lidar_ros_driver/lib/lidar_ros_driver:$LD_LIBRARY_PATH`
* Execute `ros2 launch lidar_ros_device run.launch.py`

## Export library env (optional)
export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:[path]/ros2_galactic/install/rclcpp/include
export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:[path]/onet/ros2_galactic/install/sensor_msgs/include

___
# O-Net 雷达 ROS2 驱动
* 该驱动适用于 ROS2 galactic (Ubuntu18.04)
* 适用平台 x86_64, arm, aarch64

# 设置IP（5号机为例）注意雷达ip

电脑端172.18.0.53，255.255.255.0
雷达端 src/launch/run.launch.py
```
Parameters = [{"device_ip": "172.18.0.56”}]
```

# 编译lidar_ros

```
cd lidar_ros
. ~/ros2_galactic/install/local_setup.bash
```
注意home/onet改成你自己的电脑的
```
export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:/home/onet/ros2_galactic/install/rclcpp/include
export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:/home/onet/ros2_galactic/install/sensor_msgs/include
colcon build
```

# 启动lidar_ros
```
source install/setup.bash
export LD_LIBRARY_PATH=$(pwd)/install/lidar_ros_driver/lib/lidar_ros_driver:$LD_LIBRARY_PATH
ros2 launch lidar_ros_driver run.launch.py
```

# 启动点云显示

重开终端窗口
```
. ~/ros2_galactic/install/local_setup.bash
rviz2
```