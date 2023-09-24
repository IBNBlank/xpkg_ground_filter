/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-08-29
 ****************************************************************/

#ifndef GROUND_FILTER_GROUND_FILTER_H_
#define GROUND_FILTER_GROUND_FILTER_H_

#include <ground_filter/hex_utility.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

using hex::utility::HexPointsStamped;

namespace hex {
namespace perception {

class GroundFilter {
 public:
  static GroundFilter& GetGroundFilter() {
    static GroundFilter singleton;
    return singleton;
  }

  // Work Handle
  bool Init();
  bool Work();

 private:
  GroundFilter() = default;
  virtual ~GroundFilter() = default;

  // Work Handle
  void ResetVariables();
  void ProjectPoints(const pcl::PointCloud<PointXYZIRT>::Ptr&);
  void MarkPoints();
  void SegmentatePoints();

  // Help Handle
  void TransformPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr&, bool);
  void TransformPoints(const pcl::PointCloud<PointXYZIRT>::Ptr&,
                       pcl::PointCloud<PointXYZIRT>::Ptr&, bool);

  // Parameter
  int32_t ksensor_lines_;
  int32_t ksensor_scans_;
  Eigen::Affine3f ksensor_trans_;
  double kground_angle_threshold_;
  double kground_height_range_;
  double kground_height_const_;
  double kground_height_factor_;

  // Variable
  PointXYZIRT nan_point_;
  cv::Mat lidar_mat_;
  cv::Mat ground_mat_;
  pcl::PointCloud<PointXYZIRT>::Ptr lidar_points_processed_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_points_;
};

}  // namespace perception
}  // namespace hex

#endif  // GROUND_FILTER_GROUND_FILTER_H_
