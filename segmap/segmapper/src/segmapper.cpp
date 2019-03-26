#include "segmapper/segmapper.hpp"

#include <stdlib.h>

#include <laser_slam/benchmarker.hpp>
#include <laser_slam/common.hpp>
#include <laser_slam_ros/common.hpp>
#include <ros/ros.h>
#include <segmatch/utilities.hpp>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "map_msgs/SaveMap.h"
#include <unistd.h> 
#include <string>


using namespace laser_slam;
using namespace laser_slam_ros;
using namespace segmatch;
using namespace segmatch_ros;

SegMapper::SegMapper(ros::NodeHandle& n) : nh_(n) {
  // Load ROS parameters from server.
  getParameters();

  // 订阅控制命令
  // control_sub = nh_.subscribe("/control", 10, &SegMapper::controlCall1, this);

  // TODO: it would be great to have a cleaner check here, e.g. by having the segmenter interface
  // telling us if normals are needed or not. Unfortunately, at the moment the segmenters are
  // created much later ...
  const std::string& segmenter_type =         // "IncrementalEuclideanDistance"
      segmatch_worker_params_.segmatch_params.segmenter_params.segmenter_type;
      // 是否需要计算法线
  const bool needs_normal_estimation =
      (segmenter_type == "SimpleSmoothnessConstraints") || //平滑度约束
      (segmenter_type == "IncrementalSmoothnessConstraints");

  // Configure benchmarker
  Benchmarker::setParameters(benchmarker_params_);

  // Create an incremental estimator.
  std::shared_ptr<IncrementalEstimator> incremental_estimator(
      new IncrementalEstimator(params_.online_estimator_params, params_.number_of_robots));

  incremental_estimator_ = incremental_estimator;

  // Create local map publisher
  // 发布局部地图
  local_maps_mutexes_ = std::vector<std::mutex>(params_.number_of_robots);
  if (laser_slam_worker_params_.publish_local_map) {   // true
    local_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        laser_slam_worker_params_.local_map_pub_topic, // "local_map"
        kPublisherQueueSize); // 50
  }

  // Setup the laser_slam workers.
  ROS_INFO_STREAM("Number of laser_slam workers: " << params_.number_of_robots);
  for (unsigned int i = 0u; i < params_.number_of_robots; ++i) {
    // Adjust the topics and frames for that laser_slam worker.
    LaserSlamWorkerParams params = laser_slam_worker_params_;

    // Create a local map for each robot.
    // 法线计算对象
    std::unique_ptr<NormalEstimator> normal_estimator = nullptr;
    if (needs_normal_estimation) {
      normal_estimator = NormalEstimator::create(
          segmatch_worker_params_.segmatch_params.normal_estimator_type,
          segmatch_worker_params_.segmatch_params.radius_for_normal_estimation_m);
    }
    // std::vector<segmatch::LocalMap<segmatch::PclPoint, segmatch::MapPoint>> SegMapper::local_maps_;
    // emplace_back会调用构造函数
    // LocalMap(const LocalMapParameters& params, std::unique_ptr<NormalEstimator> normal_estimator);
    // 存放每个机器人的局部地图
    local_maps_.emplace_back(segmatch_worker_params_.segmatch_params.local_map_params, std::move(normal_estimator));

    // TODO rm offset when updating mr_foundry.
    const unsigned int offset = 0;
    // 机器人多于一个时，给每个机器人的topic赋予不同的连续编号
    // offset用于确定起始编号
    if (params_.number_of_robots > 1) {
      // Subscribers.
      params.assembled_cloud_sub_topic = "/" + params_.robot_prefix + std::to_string(i + offset) +
          "/" + laser_slam_worker_params_.assembled_cloud_sub_topic;

      // TF frames.
      params.odom_frame =  params_.robot_prefix + std::to_string(i + offset) +
          "/" + laser_slam_worker_params_.odom_frame;    // "world"
      params.sensor_frame =  params_.robot_prefix + std::to_string(i + offset) +
          "/" + laser_slam_worker_params_.sensor_frame;  // "velodyne"

      // Publishers.
      params.trajectory_pub_topic = params_.robot_prefix + std::to_string(i + offset) + "/" +
          laser_slam_worker_params_.trajectory_pub_topic;    // "trajectory"

      params.local_map_pub_topic = params_.robot_prefix + std::to_string(i + offset) + "/" +
          laser_slam_worker_params_.local_map_pub_topic;     // "local_map"
    }

    LOG(INFO) << "Robot " << i << " subscribes to " << params.assembled_cloud_sub_topic << " "
        << params.odom_frame << " and " << params.sensor_frame;

    LOG(INFO) << "Robot " << i << " publishes to " << params.trajectory_pub_topic << " and "
        << params.local_map_pub_topic;

    // laser_slam_worker的创建及初始化
    std::unique_ptr<LaserSlamWorker> laser_slam_worker(new LaserSlamWorker());
    laser_slam_worker->init(nh_, params, incremental_estimator_, i);
    laser_slam_workers_.push_back(std::move(laser_slam_worker));
  }

  // Advertise the save_map service.
  // 只创建服务
  save_map_ = nh_.advertiseService("save_map", &SegMapper::saveMapServiceCall, this);
  save_local_map_ = nh_.advertiseService("save_local_map", &SegMapper::saveLocalMapServiceCall, this);

  // Initialize the SegMatchWorker.
  if (segmatch_worker_params_.localize || segmatch_worker_params_.close_loops) {  //true
    segmatch_worker_.init(n, segmatch_worker_params_, params_.number_of_robots);
  }
  
  for (size_t i = 0u; i < laser_slam_workers_.size(); ++i) {
      skip_counters_.push_back(0u);
      first_points_received_.push_back(false);
  }
}

SegMapper::~SegMapper() {}

// 发布局部地图
void SegMapper::publishMapThread() {
  // Check if map publication is required.
  if (!laser_slam_worker_params_.publish_local_map)
    return;

  ros::Rate thread_rate(laser_slam_worker_params_.map_publication_rate_hz);
  while (ros::ok()) {
    LOG(INFO) << "publishing local maps";
    MapCloud local_maps;
    // 多个机器人滤波后的局部地图相加
    for (size_t i = 0u; i < local_maps_.size(); ++i) {
      std::unique_lock<std::mutex> map_lock(local_maps_mutexes_[i]);
      local_maps += local_maps_[i].getFilteredPoints();
      map_lock.unlock();
    }
    // 把pcl中的点云转换为ros中的点云格式
    sensor_msgs::PointCloud2 msg;
    laser_slam_ros::convert_to_point_cloud_2_msg(
        local_maps,
        params_.world_frame, &msg);
    local_map_pub_.publish(msg);
    thread_rate.sleep();
  }
}

// 广播world到odom的tf
void SegMapper::publishTfThread() {
  if (params_.publish_world_to_odom) {
    ros::Rate thread_rate(params_.tf_publication_rate_hz);
    while (ros::ok()) {
      for (size_t i = 0u; i < laser_slam_workers_.size(); ++i) {
        // world to odom
        tf::StampedTransform world_to_odom = laser_slam_workers_[i]->getWorldToOdom();
        world_to_odom.stamp_ = ros::Time::now();
        tf_broadcaster_.sendTransform(world_to_odom);
      }
      thread_rate.sleep();
    }
  }
}

// 定位和回环检测
void SegMapper::segMatchThread() {
  // Terminate the thread if localization and loop closure are not needed.
  if ((!segmatch_worker_params_.localize &&
      !segmatch_worker_params_.close_loops) ||
      laser_slam_workers_.empty())
    return;

  // 初始化为最后一个机器人，下面循环中首先加１，因此是从第一个开始处理
  unsigned int track_id = laser_slam_workers_.size() - 1u;
  // Number of tracks skipped because waiting for new voxels to activate.
  // 不追踪的机器人数量
  unsigned int skipped_tracks_count = 0u;
  ros::Duration sleep_duration(kSegMatchSleepTime_s); //0.01s

  // 回环检测功能中检测到的回环数量，只用于日志输出
  unsigned int n_loops = 0u;

  while (ros::ok()) {
    // If all the tracks have been skipped consecutively, sleep for a bit to
    // free some CPU time.
    // 所有机器人均跳过
    if (skipped_tracks_count == laser_slam_workers_.size()) {
      skipped_tracks_count = 0u;
      sleep_duration.sleep();
      // 加上 continue; ??
      // 可以，但没必要，加上只会多执行一次该if语句
    }

    // Make sure that all the measurements in this loop iteration will get the same timestamp. This
    // makes it easier to plot the data.
    BENCHMARK_START_NEW_STEP();
    // No, we don't include sleeping in the timing, as it is an intended delay.
    BENCHMARK_START("SM");
    // Set the next source cloud to process.
    track_id = (track_id + 1u) % laser_slam_workers_.size();

    // Get the queued points.
    auto new_points = laser_slam_workers_[track_id]->getQueuedPoints();
    // 当前处理的机器人没有新数据，则跳过该机器人，处理下一个机器人
    if (new_points.empty()) {
      BENCHMARK_STOP_AND_IGNORE("SM");
      ++skipped_tracks_count;
      // Keep asking for publishing to increase the publishing counter.
      // 发布target与source的匹配情况
      // 发布消息与该函数的执行次数有关
      segmatch_worker_.publish();
      continue;
    } else {
        // 有什么用??
        if (!first_points_received_[track_id]) {
            first_points_received_[track_id] = true;
            skip_counters_[track_id] = 0u;
        }
    }

    // Update the local map with the new points and the new pose.
    // 获取指定机器人的当前位姿
    Pose current_pose = incremental_estimator_->getCurrentPose(track_id);
    // 大括号为了定义作用域，确定mutex锁的范围
    {
      // 把mutex放入lock_guard时，mutex自动上锁，lock_guard析构时自动解锁
      std::lock_guard<std::mutex> map_lock(local_maps_mutexes_[track_id]);
      // 根据新的位姿和点云更新局部地图
      local_maps_[track_id].updatePoseAndAddPoints(new_points, current_pose);
      // mutex解锁
    }

    // Process the source cloud.
    // 功能为定位
    if (segmatch_worker_params_.localize) {
      // 若检测到回环
      if (segmatch_worker_.processLocalMap(local_maps_[track_id], current_pose, track_id)) {
        if (!pose_at_last_localization_set_) {
          pose_at_last_localization_set_ = true;
          // 第一次设置位姿
          pose_at_last_localization_ = current_pose.T_w;
        } else {
          BENCHMARK_RECORD_VALUE("SM.LocalizationDistances", distanceBetweenTwoSE3(
              pose_at_last_localization_, current_pose.T_w));
          // 更新位姿
          pose_at_last_localization_ = current_pose.T_w;
        }
      }
    // 功能为回环检测
    } else {
      RelativePose loop_closure;

      // If there is a loop closure.
      // 检测到回环
      if (segmatch_worker_.processLocalMap(local_maps_[track_id], current_pose, track_id, &loop_closure)) {
        BENCHMARK_BLOCK("SM.ProcessLoopClosure");
        LOG(INFO)<< "Found loop closure! track_id_a: " << loop_closure.track_id_a <<
            " time_a_ns: " << loop_closure.time_a_ns <<
            " track_id_b: " << loop_closure.track_id_b <<
            " time_b_ns: " << loop_closure.time_b_ns;

        // Prevent the workers to process further scans (and add variables to the graph).
        BENCHMARK_START("SM.ProcessLoopClosure.WaitingForLockOnLaserSlamWorkers");
        for (auto& worker: laser_slam_workers_) {
          // 禁用scanCallback，
          worker->setLockScanCallback(true);
        }
        BENCHMARK_STOP("SM.ProcessLoopClosure.WaitingForLockOnLaserSlamWorkers");

        // Save last poses for updating the local maps.
        BENCHMARK_START("SM.ProcessLoopClosure.GettingLastPoseOfTrajectories");
        Trajectory trajectory;
        std::vector<SE3> last_poses_before_update;
        std::vector<laser_slam::Time> last_poses_timestamp_before_update_ns;
        // 检测到回环后不清空局部地图
        if (!params_.clear_local_map_after_loop_closure) {
          for (const auto& worker: laser_slam_workers_) {
            worker->getTrajectory(&trajectory);
            // 保存每个机器人轨迹中的最后一个元素
            // 最新位姿和最新时间不对应？
            last_poses_before_update.push_back(trajectory.rbegin()->second);
            last_poses_timestamp_before_update_ns.push_back(trajectory.rbegin()->first);
          }
        }
        BENCHMARK_STOP("SM.ProcessLoopClosure.GettingLastPoseOfTrajectories");

        BENCHMARK_START("SM.ProcessLoopClosure.UpdateIncrementalEstimator");
        // ??
        incremental_estimator_->processLoopClosure(loop_closure);
        BENCHMARK_STOP("SM.ProcessLoopClosure.UpdateIncrementalEstimator");

        BENCHMARK_START("SM.ProcessLoopClosure.ProcessLocalMap");
        for (size_t i = 0u; i < laser_slam_workers_.size(); ++i) {
          if (!params_.clear_local_map_after_loop_closure) {
            // 得到位姿变换
            laser_slam::SE3 local_map_update_transform =
                laser_slam_workers_[i]->getTransformBetweenPoses(
                    last_poses_before_update[i], last_poses_timestamp_before_update_ns[i]);
            std::unique_lock<std::mutex> map_lock2(local_maps_mutexes_[i]);
            // 对局部地图进行变换
            local_maps_[i].transform(local_map_update_transform.cast<float>());
            map_lock2.unlock();
          } else {
            // 清空局部地图
            std::unique_lock<std::mutex> map_lock2(local_maps_mutexes_[i]);
            local_maps_[i].clear();
            map_lock2.unlock();
          }
        }
        BENCHMARK_STOP("SM.ProcessLoopClosure.ProcessLocalMap");

        // 把所有机器人的局部地图相加并发布
        MapCloud local_maps;
        for (size_t i = 0u; i < local_maps_.size(); ++i) {
          std::unique_lock<std::mutex> map_lock(local_maps_mutexes_[i]);
          local_maps += local_maps_[i].getFilteredPoints();
          map_lock.unlock();
        }
        sensor_msgs::PointCloud2 msg;
        laser_slam_ros::convert_to_point_cloud_2_msg(
            local_maps,
            params_.world_frame, &msg);
        local_map_pub_.publish(msg);

        // Update the Segmatch object.
        std::vector<Trajectory> updated_trajectories;
        for (const auto& worker: laser_slam_workers_) {
          worker->getTrajectory(&trajectory);
          updated_trajectories.push_back(trajectory);
        }

        BENCHMARK_START("SM.ProcessLoopClosure.UpdateSegMatch");
        segmatch_worker_.update(updated_trajectories);
        BENCHMARK_STOP("SM.ProcessLoopClosure.UpdateSegMatch");

        //Publish the trajectories.
        for (const auto& worker : laser_slam_workers_) {
          worker->publishTrajectories();
        }

        // Unlock the workers.
        for (auto& worker: laser_slam_workers_) {
          worker->setLockScanCallback(false);
        }

        n_loops++;
        LOG(INFO) << "That was the loop number " << n_loops << ".";
      }

      for (const auto& worker : laser_slam_workers_) {
        worker->publishTrajectories();
      }
    }

    // The track was processed, reset the counter.
    skipped_tracks_count = 0;
    skip_counters_[track_id] = 0u;
    BENCHMARK_STOP("SM");
  }

  Benchmarker::logStatistics(LOG(INFO));
  // Benchmarker::saveData();
}

// 保存第一个机器人的一定半径内的局部地图
bool SegMapper::saveMapServiceCall(segmapper::SaveMap::Request& request,
                                   segmapper::SaveMap::Response& response) {
 
  try {
    LOG(INFO) << "save call\n\n\n\n\n";
    pcl::io::savePCDFileASCII(request.filename.data,
                              local_maps_.front().getFilteredPoints());
    LOG(INFO) << "saved call\n\n\n\n\n";

  }
  catch (const std::runtime_error& e) {
    ROS_ERROR_STREAM("Unable to save: " << e.what());
    return false;
  }
  return true;
}
// 有什么区别？
// 没区别
bool SegMapper::saveLocalMapServiceCall(segmapper::SaveMap::Request& request,
                                        segmapper::SaveMap::Response& response) {
  // TODO this is saving only the local map of worker ID 0.
  std::unique_lock<std::mutex> map_lock(local_maps_mutexes_[0]);
  MapCloud local_map;
  local_map += local_maps_[0].getFilteredPoints();
  map_lock.unlock();
  try {
    pcl::io::savePCDFileASCII(request.filename.data, mapPoint2PointCloud(local_map));
  }
  catch (const std::runtime_error& e) {
    ROS_ERROR_STREAM("Unable to save: " << e.what());
    return false;
  }
  return true;
}

// 从参数服务器加载参数
void SegMapper::getParameters() {
  // SegMapper parameters.
  const std::string ns = "/SegMapper";
  nh_.getParam(ns + "/number_of_robots",  //1
               params_.number_of_robots);
  nh_.getParam(ns + "/robot_prefix",      //"na"
               params_.robot_prefix);

  CHECK_GE(params_.number_of_robots, 0u);

  nh_.getParam(ns + "/publish_world_to_odom",   // true
               params_.publish_world_to_odom); 
  nh_.getParam(ns + "/world_frame",             // "map"
               params_.world_frame);
  nh_.getParam(ns + "/tf_publication_rate_hz",  // 10
               params_.tf_publication_rate_hz);

  nh_.getParam(ns + "/clear_local_map_after_loop_closure",  // false
               params_.clear_local_map_after_loop_closure);

  // laser_slam worker parameters.
  laser_slam_worker_params_ = laser_slam_ros::getLaserSlamWorkerParams(nh_, ns);
  laser_slam_worker_params_.world_frame = params_.world_frame;

  // Online estimator parameters.
  params_.online_estimator_params = laser_slam_ros::getOnlineEstimatorParams(nh_, ns);

  // Benchmarker parameters.
  benchmarker_params_ = laser_slam_ros::getBenchmarkerParams(nh_, ns);

  // ICP configuration files.
  nh_.getParam("icp_configuration_file",
               params_.online_estimator_params.laser_track_params.icp_configuration_file);
  nh_.getParam("icp_input_filters_file",
               params_.online_estimator_params.laser_track_params.icp_input_filters_file);

  // SegMatchWorker parameters.
  segmatch_worker_params_ = segmatch_ros::getSegMatchWorkerParams(nh_, ns);
  segmatch_worker_params_.world_frame = params_.world_frame;
}

// void SegMapper::controlCall1(const std_msgs::String::ConstPtr& msg){
//   std::string s_msg(msg->data.c_str());
//   LOG(INFO) << "recevied message: " <<  s_msg;
//   if(s_msg=="save"){

//   }
// }