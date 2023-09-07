/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-08-29
 ****************************************************************/

#ifndef GROUND_FILTER_HEX_UTILITY_H_
#define GROUND_FILTER_HEX_UTILITY_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

struct PointXYZIRT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(uint16_t, ring,
                                                       ring)(float, time, time))

namespace hex {
namespace utility {

typedef struct {
  double time;
  pcl::PointCloud<PointXYZIRT>::Ptr points;
} HexPointsStamped;

}  // namespace utility
}  // namespace hex

#endif  // GROUND_FILTER_HEX_UTILITY_H_
