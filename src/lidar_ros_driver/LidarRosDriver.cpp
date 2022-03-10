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

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/ros_helper.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

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
    bool m_running{true};
    bool m_auto_start{true};
    bool m_save_bag{false};
    std::string m_update_parameter;
    // rosbag2 example:
    // https://github.com/ros2/rosbag2/blob/master/rosbag2_tests/test/rosbag2_tests/test_rosbag2_cpp_api.cpp
    rosbag2_cpp::Writer m_rosbag_writer{
        std::move(std::make_unique<rosbag2_cpp::writers::SequentialWriter>())};
    rclcpp::Node::SharedPtr m_node;                                                 //节点
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_cloud_pub;  //点云发布者
    // rclcpp::Publisher m_param_pub;  //参数设置状态发布者
    // rclcpp::ServiceServer m_service;  // connect参数设置状态

    std::string m_point_cloud_topic_name{"lidar_point_cloud"};
    std::string m_frame_id{"lidar"};
    std::string m_device_ip{"192.168.1.2"};
    int m_port{2368};
    std::string m_playback_file_path;
    int m_playback_fps{10};

    int near_noise_dist{0};
    int near_noise_intensity{0};
    int time_dif{0};
    int high_pul{0};
    int time_fly{0};
    int pulse_dif{0};
    int sample_rate{0};

    std::function<void(uint32_t, onet::lidar::PointCloud<onet::lidar::PointXYZI> &)> m_callback{
        nullptr};

    lidar::LidarDevice *m_lidar_device{nullptr};
    lidar::PlaybackDevice *m_playback_device{nullptr};
    std::shared_ptr<onet::lidar::DlphDeviceParameter> m_dev_param;

    void InitLidar()
    {
        near_noise_dist =
            m_node->get_parameter<int>("/onet_lidar_ros_driver/near_noise_dist", near_noise_dist);
        near_noise_intensity = m_node->get_parameter<int>(
            "/onet_lidar_ros_driver/near_noise_intensity", near_noise_intensity);
        time_dif = m_node->get_parameter<int>("/onet_lidar_ros_driver/time_dif", time_dif);
        high_pul = m_node->get_parameter<int>("/onet_lidar_ros_driver/high_pul", high_pul);
        time_fly = m_node->get_parameter<int>("/onet_lidar_ros_driver/time_fly", time_fly);
        pulse_dif = m_node->get_parameter<int>("/onet_lidar_ros_driver/pulse_dif", pulse_dif);
        sample_rate = m_node->get_parameter<int>("/onet_lidar_ros_driver/sample_rate", sample_rate);

        m_auto_start =
            m_node->get_parameter<bool>("/onet_lidar_ros_driver/auto_start", m_auto_start);
        m_save_bag = m_node->get_parameter<bool>("/onet_lidar_ros_driver/save_bag", m_save_bag);
        m_point_cloud_topic_name = m_node->get_parameter<std::string>(
            "/onet_lidar_ros_driver/point_cloud_topic_name", m_point_cloud_topic_name);
        m_device_ip =
            m_node->get_parameter<std::string>("/onet_lidar_ros_driver/device_ip", m_device_ip);
        m_port = m_node->get_parameter<int>("/onet_lidar_ros_driver/port", m_port);
        m_frame_id =
            m_node->get_parameter<std::string>("/onet_lidar_ros_driver/frame_id", m_frame_id);
        m_playback_file_path = m_node->get_parameter<std::string>(
            "/onet_lidar_ros_driver/playback_file_path", m_playback_file_path);
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
        if (m_save_bag)
        {
            rosbag2_cpp::StorageOptions storage_options;
            storage_options.uri = "file:///tmp/lidar_ros_driver.bag";
            rosbag2_cpp::ConverterOptions converter_options;
            m_rosbag_writer.open(storage_options, converter_options);
            rosbag2_storage::TopicMetadata tm;
            tm.name = m_point_cloud_topic_name;
            tm.type = "sensor_msgs/msg/PointCloud2";
            m_rosbag_writer.create_topic(tm);
        }
        if (m_auto_start)
        {
            Run();
        }
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
        sensor_msgs::msg::PointCloud pointcloud;
        pointcloud.header.stamp = m_node->now();
        pointcloud.header.frame_id = m_frame_id;
        pointcloud.points.resize(cloud.size());
        pointcloud.channels.resize(1);
        pointcloud.channels[0].name = "intensity";
        pointcloud.channels[0].values.resize(cloud.size());
        for (size_t i = 0; i < cloud.size(); i++)
        {
            const auto &pt = cloud.at(i);
            pointcloud.points[i].x = pt[0];
            pointcloud.points[i].y = pt[1];
            pointcloud.points[i].z = pt[2];
            pointcloud.channels[0].values[i] = pt[3];
        }
        RCLCPP_INFO(m_node->get_logger(), "end time:%d us", static_cast<int>(timer.Elapsed()));
        timer.Stop();
        // convert pointcloud to pointcloud2
        sensor_msgs::msg::PointCloud2 pointcloud2;
        sensor_msgs::convertPointCloudToPointCloud2(pointcloud, pointcloud2);
        m_cloud_pub->publish(pointcloud2);
        if (m_save_bag)
        {
            auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            rcutils_system_time_now(&bag_message->time_stamp);
            bag_message->topic_name = m_point_cloud_topic_name;
            bag_message->serialized_data = rosbag2_storage::make_serialized_message(
                pointcloud2.data.data(), pointcloud2.row_step * pointcloud2.height);
            m_rosbag_writer.write(bag_message);
        }
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
                LidarParameter lidar_param = m_lidar_device->GetLidarParameter();
                {
                    lidar::RegisterData close_laser_param;
                    close_laser_param.parameters[0] = 0;
                    close_laser_param.parameters[1] = 0;
                    close_laser_param.parameters[2] = lidar_param.laser.factor;
                    close_laser_param.parameters[3] = lidar_param.laser.level;
                    close_laser_param.parameters[4] = lidar_param.laser.pulse_width;
                    m_lidar_device->SetRegisterParameter(lidar::LASER_CTL, close_laser_param);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                m_lidar_device->SetLaser(lidar_param.laser);
                {
                    //删除近处杂点
                    lidar::RegisterData data;
                    data.parameters[0] = near_noise_dist;
                    data.parameters[1] = near_noise_intensity;
                    m_lidar_device->SetRegisterParameter(lidar::TDC_GPX_REG6, data);
                }
                {
                    //删除远处重影
                    lidar::RegisterData data;
                    data.parameters[0] = pulse_dif;
                    data.parameters[1] = time_fly;
                    m_lidar_device->SetRegisterParameter(lidar::TDC_GPX_REG5, data);
                }
                {
                    lidar::RegisterData data;
                    data.parameters[0] = high_pul;
                    data.parameters[1] = time_dif;
                    m_lidar_device->SetRegisterParameter(lidar::TDC_GPX_REG4, data);
                }
                {
                    //设置采样频率
                    lidar::RegisterData data;
                    data.parameters[0] = sample_rate;
                    m_lidar_device->SetRegisterParameter(lidar::TDC_GPX_REG0, data);
                }
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

void LidarRosDriver::UpdateParameter()
{
    // m_impl->UpdateParameter();
}

bool LidarRosDriver::IsRunning() const
{
    return m_impl->m_running;
}

void LidarRosDriver::Run()
{
    m_impl->Run();
}

}}  // namespace onet::lidar_ros
