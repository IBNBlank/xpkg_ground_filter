/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-08-29
 ****************************************************************/

#ifndef GROUND_FILTER_DATA_INTERFACE_H_
#define GROUND_FILTER_DATA_INTERFACE_H_

#include <ground_filter/hex_utility.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/PointCloud2.h"

using hex::utility::HexPointsStamped;

namespace hex {
namespace perception {

enum class LogLevel { kDebug = 0, kInfo, kWarn, kError, kFatal };

class DataInterface {
 public:
  static DataInterface& GetDataInterface() {
    static DataInterface singleton;
    return singleton;
  }

  // Interface Handle
  void Log(LogLevel, const char*, ...);
  inline void Work() { ros::spin(); }
  inline void Shutdown() { ros::shutdown(); }
  inline bool Ok() { return ros::ok(); }

  // Initialization Handle
  void Init(ros::NodeHandle*, const std::string&, double, void (*)());
  void Deinit();

  // Parameter Handle
  inline const std::string& GetSensorFrame() { return ksensor_frame_; }
  inline int32_t GetSensorLines() { return ksensor_lines_; }
  inline int32_t GetSensorScans() { return ksensor_scans_; }
  inline const Eigen::Affine3f& GetSensorTrans() { return ksensor_trans_; }
  inline double GetGroundAngleThreshold() { return kground_angle_threshold_; }
  inline double GetGroundHeightRange() { return kground_height_range_; }
  inline double GetGroundHeightConst() { return kground_height_const_; }
  inline double GetGroundHeightFactor() { return kground_height_factor_; }

  // Publisher Handle
  void PublishGroundPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr&, double);
  void PublishObstaclePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
                             double);

  // Subscriber Handle
  inline bool GetLidarPointsFlag() { return lidar_points_flag_; }
  inline void ResetLidarPointsFlag() { lidar_points_flag_ = false; }
  inline const HexPointsStamped& GetLidarPoints() { return lidar_points_; }

 protected:
  // Timer Handle
  inline void TimerCallback(const ros::TimerEvent&) { timer_handle_(); }

  // Subscriber Handle
  void LidarPointsHandle(const sensor_msgs::PointCloud2Ptr&);

 private:
  DataInterface() = default;
  virtual ~DataInterface() = default;

  // Initialization Handle
  void ParameterInit();
  void VariableInit();
  void PublisherInit();
  void SubscriberInit();
  void TimerInit(double, void (*)());

  // Node Handle
  ros::NodeHandle* nh_ptr_;

  // Timer Handle
  ros::Timer timer_;
  void (*timer_handle_)();

  // Publisher Handle
  ros::Publisher ground_points_pub_;
  ros::Publisher obstacle_points_pub_;

  // Subscriber Handle
  ros::Subscriber lidar_points_sub_;

  // Parameters Handle
  std::string ksensor_frame_;
  int32_t ksensor_lines_;
  int32_t ksensor_scans_;
  Eigen::Affine3f ksensor_trans_;
  double kground_angle_threshold_;
  double kground_height_range_;
  double kground_height_const_;
  double kground_height_factor_;

  // Variable Handle
  bool lidar_points_flag_;
  HexPointsStamped lidar_points_;
};

}  // namespace perception
}  // namespace hex

#endif  // GROUND_FILTER_DATA_INTERFACE_H_
