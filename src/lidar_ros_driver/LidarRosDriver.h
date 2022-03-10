
/**************************************************************************
 * @file: LidarRosDriver.h
 * @brief:
 *
 * Copyright (c) 2021 O-Net Technologies (Group) Limited.
 * All rights reserved.
 *************************************************************************/

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace onet { namespace lidar_ros {

class LidarRosDriver
{
public:
    LidarRosDriver(rclcpp::Node::SharedPtr node);
    ~LidarRosDriver() = default;
    bool IsRunning() const;
    void UpdateParameter();
    void Run();

private:
    struct Impl;
    std::shared_ptr<Impl> m_impl;
};
}}  // namespace onet::lidar_ros
