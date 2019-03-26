#ifndef SEGMAPPER_SEGMAPPER_HPP_
#define SEGMAPPER_SEGMAPPER_HPP_

#include <string>
#include <vector>

#include <laser_slam/benchmarker.hpp>
#include <laser_slam/parameters.hpp>
#include <laser_slam/incremental_estimator.hpp>
#include <laser_slam_ros/laser_slam_worker.hpp>
#include <segmatch/common.hpp>
#include <segmatch/local_map.hpp>
#include <segmatch_ros/common.hpp>
#include <segmatch_ros/segmatch_worker.hpp>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>

#include "segmapper/SaveMap.h"

struct SegMapperParams {
  // Multi robot parameters.
  int number_of_robots;
  std::string robot_prefix;

  bool clear_local_map_after_loop_closure = true;

  // Enable publishing a tf transform from world to odom.
  bool publish_world_to_odom;
  std::string world_frame;
  double tf_publication_rate_hz;

  // Trajectory estimator parameters.
  laser_slam::EstimatorParams online_estimator_params;
}; // struct SegMapperParams

class SegMapper {

 public:
  explicit SegMapper(ros::NodeHandle& n);
  ~SegMapper();

  /// \brief A thread function for handling map publishing.
  void publishMapThread();

  /// \brief A thread function for updating the transform between world and odom.
  void publishTfThread();

  /// \brief A thread function for localizing and closing loops with SegMatch.
  void segMatchThread();

  void controlCall1(const std_msgs::String::ConstPtr& msg);

 protected:
  /// \brief Call back of the save_map service.
  // 保存一定半径内的点云地图
  bool saveMapServiceCall(segmapper::SaveMap::Request& request,
                          segmapper::SaveMap::Response& response);

    /// \brief Call back of the save_local_map service.
  bool saveLocalMapServiceCall(segmapper::SaveMap::Request& request,
                               segmapper::SaveMap::Response& response);
  
  
  
 private:
  // Get ROS parameters.
  void getParameters();

  // / The local map for each \c LaserSlamWoker.
  // 每个机器人的局部地图
  std::vector<segmatch::LocalMap<segmatch::PclPoint, segmatch::MapPoint>> local_maps_;
  std::vector<std::mutex> local_maps_mutexes_;

  // Node handle.
  ros::NodeHandle& nh_;
  // 接收控制命令
  // ros::Subscriber control_sub;

  // Publisher of the local maps
  ros::Publisher local_map_pub_;
  static constexpr unsigned int kPublisherQueueSize = 50u;

  // Parameters.
  SegMapperParams params_;
  laser_slam::BenchmarkerParams benchmarker_params_;

  // 发布tf
  tf::TransformBroadcaster tf_broadcaster_;

  // Services.
  ros::ServiceServer save_map_;
  ros::ServiceServer save_local_map_;
  // 未使用
  ros::ServiceServer save_distant_map_;
  ros::ServiceServer show_statistics_;

  // Incremental estimator.
  std::shared_ptr<laser_slam::IncrementalEstimator> incremental_estimator_;

  // SegMatch objects.
  segmatch_ros::SegMatchWorkerParams segmatch_worker_params_;
  segmatch_ros::SegMatchWorker segmatch_worker_;
  static constexpr double kSegMatchSleepTime_s = 0.01;

  // laser_slam objects.
  std::vector<std::unique_ptr<laser_slam_ros::LaserSlamWorker> > laser_slam_workers_;
  laser_slam_ros::LaserSlamWorkerParams laser_slam_worker_params_;

  std::vector<unsigned int> skip_counters_;
  unsigned int deactivate_track_when_skipped_x_ = 5u;
  std::vector<bool> first_points_received_;
  
  // Pose of the robot when localization occured. Used to compute statistics on dead-reckoning
  // distances.
  // 上次定位时的机器人位姿，用于航位推算
  laser_slam::SE3 pose_at_last_localization_;
  // 是否设置过上次定位时机器人的位姿（用于判断是否为空）
  bool pose_at_last_localization_set_ = false;


  static constexpr laser_slam::Time kHeadDurationToExport_ns = 60000000000u;
}; // SegMapper

#endif /* SEGMAPPER_SEGMAPPER_HPP_ */
