import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
         launch.actions.DeclareLaunchArgument(
            name='rviz2',
            default_value='false'
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('rviz2'))
        ),
        launch_ros.actions.Node(
            package='lidar_ros_driver',
            executable='lidar_ros_driver',
            name='onet_lidar_ros_driver',
            output='screen',
            emulate_tty=True ,
            parameters=[
                {"point_cloud_topic_name":'lidar_point_cloud'},
                {"frame_id":'lidar'},
                {"device_ip":'192.168.1.2'},
                {"port":2368},
                {"playback_file_path":""},
                {"auto_start":True},
                {"save_bag":True},
                {"near_noise_dist":19200},
                {"near_noise_intensity":135},
                {"time_dif":200},
                {"high_pul":350},
                {"time_fly":19200},
                {"pulse_dif":124},
                {"sample_rate":500}
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
