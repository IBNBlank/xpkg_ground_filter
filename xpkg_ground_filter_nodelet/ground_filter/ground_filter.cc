/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-08-29
 ****************************************************************/

#include "ground_filter/ground_filter.h"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <limits>
#include <vector>
// #include <chrono>

#include "ground_filter/data_interface.h"

namespace hex {
namespace perception {

bool GroundFilter::Init() {
  static DataInterface& data_interface = DataInterface::GetDataInterface();
  // parameters
  ksensor_lines_ = data_interface.GetSensorLines();
  ksensor_scans_ = data_interface.GetSensorScans();
  ksensor_trans_ = data_interface.GetSensorTrans();
  kground_angle_threshold_ = data_interface.GetGroundAngleThreshold();
  kground_height_range_ = data_interface.GetGroundHeightRange();
  kground_height_const_ = data_interface.GetGroundHeightConst();
  kground_height_factor_ = data_interface.GetGroundHeightFactor();

  // cv mat
  lidar_mat_ =
      cv::Mat(ksensor_lines_, ksensor_scans_, CV_32F, cv::Scalar::all(FLT_MAX));
  ground_mat_ =
      cv::Mat(ksensor_lines_, ksensor_scans_, CV_8S, cv::Scalar::all(0));

  // point cloud
  nan_point_.x = std::numeric_limits<float>::quiet_NaN();
  nan_point_.y = std::numeric_limits<float>::quiet_NaN();
  nan_point_.z = std::numeric_limits<float>::quiet_NaN();
  nan_point_.intensity = -1;
  nan_point_.ring = 255;
  nan_point_.time = -1;
  lidar_points_processed_ =
      pcl::PointCloud<PointXYZIRT>::Ptr(new pcl::PointCloud<PointXYZIRT>());
  ground_points_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  obstacle_points_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  lidar_points_processed_->points.resize(ksensor_lines_ * ksensor_scans_);
  std::fill(lidar_points_processed_->points.begin(),
            lidar_points_processed_->points.end(), nan_point_);

  return true;
}

bool GroundFilter::Work() {
  static DataInterface& data_interface = DataInterface::GetDataInterface();

  if (data_interface.GetLidarPointsFlag()) {
    HexPointsStamped lidar_points_stamped = data_interface.GetLidarPoints();
    pcl::PointCloud<PointXYZIRT>::Ptr lidar_points_raw =
        lidar_points_stamped.points;

    TransformPoints(lidar_points_raw, lidar_points_raw, false);

    ProjectPoints(lidar_points_raw);
    MarkPoints();
    SegmentatePoints();

    TransformPoints(ground_points_, ground_points_, true);
    TransformPoints(obstacle_points_, obstacle_points_, true);

    data_interface.PublishGroundPoints(ground_points_,
                                       lidar_points_stamped.time);
    data_interface.PublishObstaclePoints(obstacle_points_,
                                         lidar_points_stamped.time);

    ResetVariables();
    data_interface.ResetLidarPointsFlag();
  }

  return true;
}

void GroundFilter::ResetVariables() {
  lidar_mat_ =
      cv::Mat(ksensor_lines_, ksensor_scans_, CV_32F, cv::Scalar::all(FLT_MAX));
  ground_mat_ =
      cv::Mat(ksensor_lines_, ksensor_scans_, CV_8S, cv::Scalar::all(0));

  ground_points_->points.clear();
  obstacle_points_->points.clear();

  std::fill(lidar_points_processed_->points.begin(),
            lidar_points_processed_->points.end(), nan_point_);
}

void GroundFilter::ProjectPoints(
    const pcl::PointCloud<PointXYZIRT>::Ptr& source_points) {
  PointXYZIRT current_point;
  int32_t cloud_size = source_points->points.size();
  for (int32_t i = 0; i < cloud_size; i++) {
    current_point.x = source_points->points[i].x;
    current_point.y = source_points->points[i].y;
    current_point.z = source_points->points[i].z;
    current_point.intensity = 10.0;

    int32_t line_index = source_points->points[i].ring;
    if (line_index < 0 || line_index >= ksensor_lines_) {
      continue;
    }

    double scan_angle = atan2(current_point.y, current_point.x) * 0.5 * M_1_PI;
    if (scan_angle < 0) {
      scan_angle += 1;
    }
    int32_t scan_index = floor(scan_angle * ksensor_scans_);

    double range = sqrt(current_point.x * current_point.x +
                        current_point.y * current_point.y +
                        current_point.z * current_point.z);
    if (range < 0.5) {
      continue;
    }
    lidar_mat_.at<float>(line_index, scan_index) = range;

    lidar_points_processed_->points[line_index * ksensor_scans_ + scan_index] =
        current_point;
  }
}

void GroundFilter::MarkPoints() {
  static int32_t ground_index_ = floor(ksensor_lines_ * 0.5);

  for (int32_t j = 0; j < ksensor_scans_; j++) {
    for (int32_t i = 0; i < ground_index_; i++) {
      int32_t lower_index = j + (i)*ksensor_scans_;
      int32_t upper_index = j + (i + 1) * ksensor_scans_;

      bool lower_invalid = false;
      bool upper_invalid = false;
      if (lidar_points_processed_->points[lower_index].intensity == -1) {
        ground_mat_.at<int8_t>(i, j) = -1;
        lower_invalid = true;
      }
      if (lidar_points_processed_->points[upper_index].intensity == -1) {
        ground_mat_.at<int8_t>(i + 1, j) = -1;
        upper_invalid = true;
      }
      if (lower_invalid || upper_invalid) {
        continue;
      }

      double diff_x = lidar_points_processed_->points[upper_index].x -
                      lidar_points_processed_->points[lower_index].x;
      double diff_y = lidar_points_processed_->points[upper_index].y -
                      lidar_points_processed_->points[lower_index].y;
      double diff_z = lidar_points_processed_->points[upper_index].z -
                      lidar_points_processed_->points[lower_index].z;
      double ground_angle =
          atan2(diff_z, sqrt(diff_x * diff_x + diff_y * diff_y));
      if (fabs(ground_angle) < kground_angle_threshold_) {
        ground_mat_.at<int8_t>(i, j) = 1;
        ground_mat_.at<int8_t>(i + 1, j) = 1;
      }
    }
  }
}

void GroundFilter::SegmentatePoints() {
  pcl::PointXYZ current_point;

  for (int32_t j = 0; j < ksensor_scans_; j++) {
    for (int32_t i = 0; i < ksensor_lines_; i++) {
      if (ground_mat_.at<int8_t>(i, j) == -1) {
        continue;
      }

      int32_t index = j + i * ksensor_scans_;
      current_point.x = lidar_points_processed_->points[index].x;
      current_point.y = lidar_points_processed_->points[index].y;
      current_point.z = lidar_points_processed_->points[index].z;

      double range = lidar_mat_.at<float>(i, j);
      if (range < kground_height_range_) {
        if (current_point.z <
            kground_height_factor_ * range - kground_height_const_) {
          ground_points_->points.push_back(current_point);
        } else {
          obstacle_points_->points.push_back(current_point);
        }
      } else if (ground_mat_.at<int8_t>(i, j) == 1) {
        ground_points_->points.push_back(current_point);
      } else {
        obstacle_points_->points.push_back(current_point);
      }
    }
  }
}

void GroundFilter::TransformPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& raw_pcd,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out_pcd, bool inverse_flag) {
  if (inverse_flag) {
    pcl::transformPointCloud(*raw_pcd, *out_pcd,
                             ksensor_trans_.inverse().matrix());
  } else {
    pcl::transformPointCloud(*raw_pcd, *out_pcd, ksensor_trans_.matrix());
  }
}

void GroundFilter::TransformPoints(
    const pcl::PointCloud<PointXYZIRT>::Ptr& raw_pcd,
    pcl::PointCloud<PointXYZIRT>::Ptr& out_pcd, bool inverse_flag) {
  if (inverse_flag) {
    pcl::transformPointCloud(*raw_pcd, *out_pcd,
                             ksensor_trans_.inverse().matrix());
  } else {
    pcl::transformPointCloud(*raw_pcd, *out_pcd, ksensor_trans_.matrix());
  }
}

}  // namespace perception
}  // namespace hex
