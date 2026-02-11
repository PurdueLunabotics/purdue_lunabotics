/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#pragma once

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// SDK
#include "unitree_lidar_sdk_pcl.h"

/**
 * @brief Publish a pointcloud
 *
 * @param thisPub
 * @param thisCloud
 * @param thisStamp
 * @param thisFrame
 * @return sensor_msgs::PointCloud2
 */
inline sensor_msgs::PointCloud2 publishCloud(
    ros::Publisher *thisPub,
    pcl::PointCloud<PointType>::Ptr thisCloud,
    ros::Time thisStamp,
    std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

/**
 * @brief Unitree Lidar SDK Node
 */
class UnitreeLidarRosNode
{
protected:
    // ROS
    ros::NodeHandle nh_;
    ros::Publisher pub_pointcloud_raw_;
    ros::Publisher pub_imu_;
    tf::TransformBroadcaster tfbc1_;
    ros::Publisher pub_laser_scan_;

    // Unitree Lidar Reader
    UnitreeLidarReader *lsdk_;

    // Config params
    std::string serial_port;
    int baudrate_;

    std::string cloud_frame_;
    std::string cloud_topic_;
    int cloud_scan_num_;

    std::string imu_frame_;
    std::string imu_topic_;

    std::string laserscan_frame_;
    std::string laserscan_topic_;

    int initialize_type_;

    int local_port_;
    std::string local_ip_;
    int lidar_port_;
    std::string lidar_ip_;

    int work_mode_;

    double range_min_;
    double range_max_;
    bool use_system_timestamp_;

public:
    UnitreeLidarRosNode(ros::NodeHandle nh)
    {

        // Load config parameters
        nh.param("/unitree_lidar_ros_node/initialize_type", initialize_type_, 1);
        nh.param("/unitree_lidar_ros_node/work_mode", work_mode_, 0);
        nh.param("/unitree_lidar_ros_node/range_min", range_min_, 0.0);
        nh.param("/unitree_lidar_ros_node/range_max", range_max_, 100.0);
        nh.param("/unitree_lidar_ros_node/use_system_timestamp", use_system_timestamp_, true);

        nh.param("/unitree_lidar_ros_node/serial_port", serial_port, std::string("/dev/ttyACM0"));
        nh.param("/unitree_lidar_ros_node/baudrate", baudrate_, 4000000);

        nh.param("/unitree_lidar_ros_node/lidar_port", lidar_port_, 6101);
        nh.param("/unitree_lidar_ros_node/lidar_ip", lidar_ip_, std::string("10.10.10.10"));
        nh.param("/unitree_lidar_ros_node/pc_port", local_port_, 6201);
        nh.param("/unitree_lidar_ros_node/pc_ip", local_ip_, std::string("10.10.10.100"));

        nh.param("/unitree_lidar_ros_node/cloud_frame", cloud_frame_, std::string("unilidar_lidar"));
        nh.param("/unitree_lidar_ros_node/cloud_topic", cloud_topic_, std::string("unilidar/cloud"));
        nh.param("/unitree_lidar_ros_node/cloud_scan_num", cloud_scan_num_, 18);

        nh.param("/unitree_lidar_ros_node/imu_frame", imu_frame_, std::string("unilidar_imu"));
        nh.param("/unitree_lidar_ros_node/imu_topic", imu_topic_, std::string("unilidar/imu"));

        nh.param("/unitree_lidar_ros_node/laserscan_frame", laserscan_frame_, std::string("unilidar_lidar"));
        nh.param("/unitree_lidar_ros_node/laserscan_topic", laserscan_topic_, std::string("unilidar/laserscan"));

        // Initialize UnitreeLidarReader
        lsdk_ = createUnitreeLidarReader();

        std::cout << "initialize_type_ = " << initialize_type_ << std::endl;
        std::cout << "work_mode_ = " << work_mode_ << std::endl;

        if (initialize_type_ == 1)
        {
            lsdk_->initializeSerial(serial_port, baudrate_, 
                cloud_scan_num_, use_system_timestamp_, range_min_, range_max_);
        }
        else if (initialize_type_ == 2)
        {
            lsdk_->initializeUDP(lidar_port_, lidar_ip_, local_port_, local_ip_, 
                cloud_scan_num_, use_system_timestamp_, range_min_, range_max_);
        }else{
            std::cout << "initialize_type is not right! exit now ...\n";
            exit(0);
        }

        lsdk_->setLidarWorkMode(work_mode_);

        // ROS
        nh_ = nh;
        pub_pointcloud_raw_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic_, 100);
        pub_imu_ = nh.advertise<sensor_msgs::Imu>(imu_topic_, 1000);
        pub_laser_scan_ = nh.advertise<sensor_msgs::LaserScan>(laserscan_topic_, 100);
    }

    ~UnitreeLidarRosNode()
    {
    }

    /**
     * @brief Run once
     */
    bool run()
    {
        int result = lsdk_->runParse();
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        if (result == LIDAR_IMU_DATA_PACKET_TYPE)
        {
            LidarImuData imu;
            if (lsdk_->getImuData(imu))
            {
                // publish imu message
                sensor_msgs::Imu imuMsg;
                imuMsg.header.frame_id = imu_frame_;
                imuMsg.header.stamp = ros::Time::now();

                imuMsg.orientation.w = imu.quaternion[0];
                imuMsg.orientation.x = imu.quaternion[1];
                imuMsg.orientation.y = imu.quaternion[2];
                imuMsg.orientation.z = imu.quaternion[3];

                imuMsg.angular_velocity.x = imu.angular_velocity[0];
                imuMsg.angular_velocity.y = imu.angular_velocity[1];
                imuMsg.angular_velocity.z = imu.angular_velocity[2];

                imuMsg.linear_acceleration.x = imu.linear_acceleration[0];
                imuMsg.linear_acceleration.y = imu.linear_acceleration[1];
                imuMsg.linear_acceleration.z = imu.linear_acceleration[2];

                pub_imu_.publish(imuMsg);

                // publish tf from initial imu to real-time imu
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(0, 0, 0));
                transform.setRotation(tf::Quaternion(imu.quaternion[1], imu.quaternion[2],
                                                     imu.quaternion[3], imu.quaternion[0]));
                tfbc1_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), imu_frame_ + "_initial", imu_frame_));

                // publish tf from imu to lidar
                transform.setOrigin(tf::Vector3(0.007698, 0.014655, -0.00667));
                transform.setRotation(tf::Quaternion(0, 0, 0, 1));
                tfbc1_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), imu_frame_, cloud_frame_));

            }
            return true;
        }
        else if (result == LIDAR_POINT_DATA_PACKET_TYPE)
        {
            // std::cout << "LIDAR_POINT_DATA_PACKET_TYPE" << std::endl;
            PointCloudUnitree cloud;
            if (lsdk_->getPointCloud(cloud))
            {
                transformUnitreeCloudToPCL(cloud, cloudOut);
                publishCloud(&pub_pointcloud_raw_, cloudOut, ros::Time::now().fromSec(cloud.stamp), cloud_frame_);
            }

            return true;
        }
        else if (result == LIDAR_2D_POINT_DATA_PACKET_TYPE)
        {
            // std::cout << "LIDAR_2D_POINT_DATA_PACKET_TYPE" << std::endl;
            Lidar2DPointDataPacket packet = lsdk_->getLidar2DPointDataPacket();
            Lidar2DPointData &data = packet.data;

            sensor_msgs::LaserScan scan;
            scan.header.stamp = ros::Time::now();
            scan.header.frame_id = laserscan_frame_;
            scan.angle_min = data.angle_min;
            scan.angle_max = data.angle_min + data.angle_increment * data.point_num;
            scan.angle_increment = data.angle_increment;
            scan.time_increment = data.time_increment;
            scan.range_min = 0.0;
            scan.range_max = 100.0;
            scan.ranges.resize(data.point_num);
            scan.intensities.resize(data.point_num);

            // std::cout << "data.point_num = " << data.point_num 
            //         << ", data.angle_min = " << data.angle_min
            //         << ", scan.angle_max = " << scan.angle_max
            //         << ", data.angle_increment = " << data.angle_increment
            //         << ", data.time_increment = " << data.time_increment
            //         << ", data.range_min = " << data.range_min
            //         << ", data.range_max = " << data.range_max
            //         << std::endl;
            
            int countValid = 0;
            for (unsigned int i = 0; i < data.point_num; ++i)
            {
                scan.ranges[i] = data.ranges[i] * data.param.range_scale;
                scan.intensities[i] = data.intensities[i];

                if (scan.ranges[i] > scan.range_min && scan.ranges[i] < scan.range_max)
                {
                    countValid++;
                }
            }

            pub_laser_scan_.publish(scan);

            return true;
        }
        else
        {
            return false;
        }
    }
};