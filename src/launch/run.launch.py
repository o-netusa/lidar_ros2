import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz2',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='point_cloud_topic_name',
            default_value='lidar_point_cloud'
        ),
        launch.actions.DeclareLaunchArgument(
            name='frame_id',
            default_value='lidar'
        ),
        launch.actions.DeclareLaunchArgument(
            name='device_ip',
            default_value='192.168.1.2'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2368'
        ),
        launch.actions.DeclareLaunchArgument(
            name='playback_file_path',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='auto_start',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='save_bag',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='near_noise_dist',
            default_value='19200'
        ),
        launch.actions.DeclareLaunchArgument(
            name='near_noise_intensity',
            default_value='135'
        ),
        launch.actions.DeclareLaunchArgument(
            name='time_dif',
            default_value='200'
        ),
        launch.actions.DeclareLaunchArgument(
            name='high_pul',
            default_value='350'
        ),
        launch.actions.DeclareLaunchArgument(
            name='time_fly',
            default_value='19200'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pulse_dif',
            default_value='124'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sample_rate',
            default_value='500'
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
            parameters=[
                {
                    'point_cloud_topic_name': launch.substitutions.LaunchConfiguration('point_cloud_topic_name')
                },
                {
                    'frame_id': launch.substitutions.LaunchConfiguration('frame_id')
                },
                {
                    'device_ip': launch.substitutions.LaunchConfiguration('device_ip')
                },
                {
                    'port': launch.substitutions.LaunchConfiguration('port')
                },
                {
                    'playback_file_path': launch.substitutions.LaunchConfiguration('playback_file_path')
                },
                {
                    'auto_start': launch.substitutions.LaunchConfiguration('auto_start')
                },
                {
                    'save_bag': launch.substitutions.LaunchConfiguration('save_bag')
                },
                {
                    'near_noise_dist': launch.substitutions.LaunchConfiguration('near_noise_dist')
                },
                {
                    'near_noise_intensity': launch.substitutions.LaunchConfiguration('near_noise_intensity')
                },
                {
                    'time_dif': launch.substitutions.LaunchConfiguration('time_dif')
                },
                {
                    'high_pul': launch.substitutions.LaunchConfiguration('high_pul')
                },
                {
                    'time_fly': launch.substitutions.LaunchConfiguration('time_fly')
                },
                {
                    'pulse_dif': launch.substitutions.LaunchConfiguration('pulse_dif')
                },
                {
                    'sample_rate': launch.substitutions.LaunchConfiguration('sample_rate')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
