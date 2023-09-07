/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-08-29
 ****************************************************************/

#include "ground_filter/data_interface.h"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "geometry_msgs/TransformStamped.h"

namespace hex {
namespace perception {

void DataInterface::Log(LogLevel level, const char* format, ...) {
  char* buffer;

  va_list args;
  va_start(args, format);
  int32_t len = vasprintf(&buffer, format, args);
  va_end(args);

  if (len < 0) {
    ROS_FATAL("### Wrong Log Message ###");
    return;
  }

  switch (level) {
    case LogLevel::kDebug: {
      ROS_DEBUG("%s", buffer);
      break;
    }
    case LogLevel::kInfo: {
      ROS_INFO("%s", buffer);
      break;
    }
    case LogLevel::kWarn: {
      ROS_WARN("%s", buffer);
      break;
    }
    case LogLevel::kError: {
      ROS_ERROR("%s", buffer);
      break;
    }
    case LogLevel::kFatal: {
      ROS_FATAL("%s", buffer);
      break;
    }
    default: {
      ROS_FATAL("### Wrong Log Level ###");
      ROS_FATAL("%s", buffer);
      break;
    }
  }

  free(buffer);
}

void DataInterface::Init(int argc, char* argv[], std::string name,
                         double period, void (*handle)()) {
  ros::init(argc, argv, name);
  static ros::NodeHandle nh;
  static ros::NodeHandle nh_local("~");
  nh_ptr_ = &nh;
  nh_local_ptr_ = &nh_local;

  ParameterInit();
  VariableInit();
  PublisherInit();
  SubscriberInit();
  TimerInit(period, handle);

  Log(LogLevel::kInfo,
      "\033[1;32m %s: ### data interface init finish ### \033[0m", name.data());
}

void DataInterface::Deinit() { Shutdown(); }

void DataInterface::ParameterInit() {
  nh_local_ptr_->param<std::string>("sensor_frame", ksensor_frame_, "lidar");
  nh_local_ptr_->param<int32_t>("sensor_lines", ksensor_lines_, 16);
  nh_local_ptr_->param<int32_t>("sensor_scans", ksensor_scans_, 1800);

  std::vector<double> sensor_position_vec;
  std::vector<double> sensor_orientation_vec;
  nh_local_ptr_->param<std::vector<double>>(
      "sensor_position", sensor_position_vec,
      std::vector<double>({0.0, 0.0, 0.0}));
  nh_local_ptr_->param<std::vector<double>>(
      "sensor_orientation", sensor_orientation_vec,
      std::vector<double>({1.0, 0.0, 0.0, 0.0}));
  ksensor_trans_.matrix().block<3, 3>(0, 0) =
      Eigen::Quaternionf(
          sensor_orientation_vec.at(0), sensor_orientation_vec.at(1),
          sensor_orientation_vec.at(2), sensor_orientation_vec.at(3))
          .toRotationMatrix();
  ksensor_trans_.matrix().block<3, 1>(0, 3) =
      Eigen::Vector3f(sensor_position_vec.at(0), sensor_position_vec.at(1),
                      sensor_position_vec.at(2));

  nh_local_ptr_->param<double>("ground_angle_thread", kground_angle_thread_,
                               0.15);
  nh_local_ptr_->param<double>("ground_height_range", kground_height_range_,
                               5.0);
  nh_local_ptr_->param<double>("ground_height_const", kground_height_const_,
                               0.65);
  nh_local_ptr_->param<double>("ground_height_factor", kground_height_factor_,
                               0.025);
}

void DataInterface::VariableInit() {
  lidar_points_flag_ = false;
  lidar_points_.points =
      pcl::PointCloud<PointXYZIRT>::Ptr(new pcl::PointCloud<PointXYZIRT>());
}

void DataInterface::PublisherInit() {
  ground_points_pub_ =
      nh_ptr_->advertise<sensor_msgs::PointCloud2>("ground_points", 1);
  obstacle_points_pub_ =
      nh_ptr_->advertise<sensor_msgs::PointCloud2>("obstacle_points", 1);
}

void DataInterface::SubscriberInit() {
  lidar_points_sub_ = nh_ptr_->subscribe(
      "lidar_points", 1, &DataInterface::LidarPointsHandle, this);
}

void DataInterface::TimerInit(double period, void (*handle)()) {
  timer_handle_ = handle;
  timer_ = nh_ptr_->createTimer(ros::Duration(period * 0.001),
                                &DataInterface::TimerCallback, this);
}

void DataInterface::PublishGroundPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_points, double time) {
  sensor_msgs::PointCloud2 ground_points_msg;

  pcl::toROSMsg(*ground_points, ground_points_msg);
  ground_points_msg.header.stamp = ros::Time::now();
  ground_points_msg.header.frame_id = ksensor_frame_;

  ground_points_pub_.publish(ground_points_msg);
}

void DataInterface::PublishObstaclePoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_points, double time) {
  sensor_msgs::PointCloud2 obstacle_points_msg;

  pcl::toROSMsg(*obstacle_points, obstacle_points_msg);
  obstacle_points_msg.header.stamp = ros::Time::now();
  obstacle_points_msg.header.frame_id = ksensor_frame_;

  obstacle_points_pub_.publish(obstacle_points_msg);
}

void DataInterface::LidarPointsHandle(const sensor_msgs::PointCloud2& msg) {
  if (ros::Time::now().toSec() - msg.header.stamp.toSec() < 0.2 &&
      !lidar_points_flag_) {
    lidar_points_.time = msg.header.stamp.toSec();
    lidar_points_.points->clear();
    pcl::fromROSMsg(msg, *lidar_points_.points);

    lidar_points_flag_ = true;
  }
}

}  // namespace perception
}  // namespace hex
