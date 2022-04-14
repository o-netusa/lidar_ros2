/**************************************************************************
 * @file: LidarRosDriver.cpp
 * @brief:
 *
 * Copyright (c) 2021 O-Net Technologies (Group) Limited.
 * All rights reserved.
 *************************************************************************/

#include "LidarRosDriver.h"

#include <DeviceManager.h>
#include <DolphinDevice.h>
#include <LidarDevice.h>
#include <PlaybackDevice.h>
#include <common/FileSystem.h>
#include <common/Timer.h>
#include <config/DeviceParamsConfig.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

#include "rcpputils/filesystem_helper.hpp"
namespace fs = std::filesystem;
namespace onet { namespace lidar_ros {

static onet::lidar::PlaybackDevice *GetPlaybackDevice(const std::vector<std::string> &file_list)
{
    static uuids::uuid play_device_id{};
    if (!play_device_id.is_nil())
    {
        onet::lidar::DeviceManager::GetInstance().RemoveDevice(play_device_id);
    }
    play_device_id = lidar::DeviceManager::GetInstance().CreateDevice(file_list);
    return dynamic_cast<lidar::PlaybackDevice *>(
        lidar::DeviceManager::GetInstance().GetDevice(play_device_id));
}

static onet::lidar::LidarDevice *GetLidarDevice(const std::string &strIP, int port)
{
    static uuids::uuid lidar_device_id{};
    if (!lidar_device_id.is_nil())
    {
        onet::lidar::DeviceManager::GetInstance().RemoveDevice(lidar_device_id);
    }
    lidar_device_id = onet::lidar::DeviceManager::GetInstance().CreateDevice(strIP, port);
    return dynamic_cast<lidar::LidarDevice *>(
        lidar::DeviceManager::GetInstance().GetDevice(lidar_device_id));
}

struct LidarRosDriver::Impl
{
    rclcpp::Node::SharedPtr m_node;                                                 //节点
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_cloud_pub;  //点云发布者

    std::string m_point_cloud_topic_name{"lidar_point_cloud"};
    std::string m_frame_id{"lidar"};
    std::string m_device_ip{"192.168.1.2"};
    int m_port{2368};
    std::string m_playback_file_path;
    int m_playback_fps{10};

    std::function<void(uint32_t, onet::lidar::PointCloud<onet::lidar::PointXYZI> &)> m_callback{
        nullptr};

    lidar::LidarDevice *m_lidar_device{nullptr};
    lidar::PlaybackDevice *m_playback_device{nullptr};

    void InitLidar()
    {
        m_node->declare_parameter<std::string>("point_cloud_topic_name", m_point_cloud_topic_name);
        m_node->declare_parameter<std::string>("frame_id", m_frame_id);
        m_node->declare_parameter<std::string>("device_ip", m_device_ip);
        m_node->declare_parameter<int>("port", m_port);
        m_node->declare_parameter<std::string>("playback_file_path", m_playback_file_path);

        m_point_cloud_topic_name = (m_node->get_parameter("point_cloud_topic_name")).as_string();
        m_device_ip = (m_node->get_parameter("device_ip")).as_string();
        m_port = (m_node->get_parameter("port")).as_int();
        m_frame_id = (m_node->get_parameter("frame_id")).as_string();
        m_playback_file_path = (m_node->get_parameter("playback_file_path")).as_string();
    }

    Impl(rclcpp::Node::SharedPtr node) : m_node(node)
    {
        try
        {
            // fetch parameters
            InitLidar();

        } catch (const std::exception &e)
        {
            RCLCPP_ERROR(m_node->get_logger(), "Error fetching parameters: %s", e.what());
        }
        m_cloud_pub =
            m_node->create_publisher<sensor_msgs::msg::PointCloud2>(m_point_cloud_topic_name, 100);
        m_callback = [this](uint32_t frame_id, lidar::PointCloud<lidar::PointXYZI> &cloud) {
            HandlePointCloud(frame_id, cloud);
        };

        Run();
    }

    ~Impl()
    {
        if (m_lidar_device)
        {
            RCLCPP_INFO(m_node->get_logger(), "Stop lidar device");
            m_lidar_device->Stop();
        }
        if (m_playback_device)
        {
            RCLCPP_INFO(m_node->get_logger(), "Stop playback device");
            m_playback_device->Stop();
        }
    }

    void HandlePointCloud(uint32_t frame_id, lidar::PointCloud<onet::lidar::PointXYZI> cloud)
    {
        if (cloud.empty())
        {
            return;
        }
        cppbase::TimerUs timer;
        pcl::PointCloud<pcl::PointXYZI> pointcloud;
        pointcloud.points.resize(cloud.size());
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            const auto &pt = cloud.at(i);
            pointcloud.points[i].x = pt[0];
            pointcloud.points[i].y = pt[1];
            pointcloud.points[i].z = pt[2];
            pointcloud.points[i].intensity = pt[3];
        }
        sensor_msgs::msg::PointCloud2 msg_pointcloud;
        pcl::toROSMsg(pointcloud, msg_pointcloud);
        msg_pointcloud.header.stamp = m_node->now();
        msg_pointcloud.header.frame_id = m_frame_id;

        RCLCPP_INFO(m_node->get_logger(), "end time:%d us", static_cast<int>(timer.Elapsed()));
        timer.Stop();

        m_cloud_pub->publish(msg_pointcloud);
    }

    /**
     * @brief Run the device directly
     */
    void Run()
    {
        RCLCPP_INFO(m_node->get_logger(), "Current directory: %s",
                    fs::current_path().string().c_str());
        RCLCPP_INFO(m_node->get_logger(), "Playback file path: %s", m_playback_file_path.c_str());
        // Use PlaybackDevice if playback_file_path is not empty
        if (!m_playback_file_path.empty())
        {
            std::vector<std::string> file_list{m_playback_file_path};  // will improve later
            m_playback_device = GetPlaybackDevice(file_list);
            if (!m_playback_device)
            {
                RCLCPP_ERROR(m_node->get_logger(), "Error: failed to create playback device!");
                return;
            }
            try
            {
                m_playback_device->Init();
                m_playback_device->SetFPS(m_playback_fps);
                m_playback_device->RegisterPointCloudCallback(m_callback);
                if (!m_playback_device->Start())
                {
                    RCLCPP_ERROR(m_node->get_logger(), "Error: failed to start playback device!");
                    return;
                }
            } catch (std::exception &e)
            {
                RCLCPP_ERROR(m_node->get_logger(), "Error:%s", e.what());
            }
        } else
        {
            m_lidar_device = GetLidarDevice(m_device_ip, m_port);
            if (!m_lidar_device)
            {
                RCLCPP_ERROR(m_node->get_logger(), "Error: failed to connect to LiDAR device!");
                return;
            }
            try
            {
                m_lidar_device->Init();

                m_lidar_device->RegisterPointCloudCallback(m_callback);
                if (!m_lidar_device->Start())
                {
                    RCLCPP_ERROR(m_node->get_logger(), "Error: failed to start LiDAR device!");
                    return;
                }
            } catch (std::exception &e)
            {
                RCLCPP_ERROR(m_node->get_logger(), "Error:%s", e.what());
            }
        }
    }
};

LidarRosDriver::LidarRosDriver(rclcpp::Node::SharedPtr node)
    : m_impl(std::make_shared<LidarRosDriver::Impl>(node))
{}

}}  // namespace onet::lidar_ros
