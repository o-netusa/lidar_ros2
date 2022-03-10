/**************************************************************************
 * @file: LidarRosNode.cpp
 * @brief:
 *
 * Copyright (c) 2021 O-Net Technologies (Group) Limited.
 * All rights reserved.
 *************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include "LidarRosDriver.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lidar_ros_driver");
    onet::lidar_ros::LidarRosDriver dvr(node);
    rclcpp::Rate loop_rate(100);
    rclcpp::spin(node);
    return 0;
}
