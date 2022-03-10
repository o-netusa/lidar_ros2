/**************************************************************************
 * @file: DeviceTests.cpp
 * @brief:
 *
 * Copyright (c) 2021 O-Net Technologies (Group) Limited.
 * All rights reserved.
 *************************************************************************/

#include <gtest/gtest.h>
#include <lidar_ros_driver/LidarRosDriver.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <thread>

TEST(LidarRosDriver, LidarRosDriver)
{
    auto node = rclcpp::Node::make_shared("tests");
    onet::lidar_ros::LidarRosDriver dvr(node);
    rclcpp::Rate loop_rate(100);
    rclcpp::spin_some(node);
    loop_rate.sleep();
    EXPECT_TRUE(dvr.IsRunning());
    EXPECT_FALSE(dvr.IsRunning());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
