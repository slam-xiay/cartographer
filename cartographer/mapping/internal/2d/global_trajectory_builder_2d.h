/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef CARTOGRAPHER_MAPPING_INTERNAL_GLOBAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_GLOBAL_TRAJECTORY_BUILDER_H_

#include <memory>

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
// #include "cartographer/mapping/trajectory_builder_interface.h"

namespace cartographer {
namespace mapping {

  struct InsertionResult {
    NodeId node_id;
    std::shared_ptr<const TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  };

  // A callback which is called after local SLAM processes an accumulated
  // 'sensor::RangeData'. If the data was inserted into a submap, reports the
  // assigned 'NodeId', otherwise 'nullptr' if the data was filtered out.
  using LocalSlamResultCallback =
      std::function<void(int /* trajectory ID */, common::Time,
                         transform::Rigid3d /* local pose estimate */,
                         sensor::RangeData /* in local frame */,
                         std::unique_ptr<const InsertionResult>)>;

  struct SensorId {
    enum class SensorType {
      RANGE = 0,
      IMU,
      ODOMETRY,
      FIXED_FRAME_POSE,
      LANDMARK,
      LOCAL_SLAM_RESULT
    };

    SensorType type;
    std::string id;

    bool operator==(const SensorId& other) const {
      return std::forward_as_tuple(type, id) ==
             std::forward_as_tuple(other.type, other.id);
    }

    bool operator<(const SensorId& other) const {
      return std::forward_as_tuple(type, id) <
             std::forward_as_tuple(other.type, other.id);
    }
  };

class GlobalTrajectoryBuilder2D {
 public:
  GlobalTrajectoryBuilder2D(
      std::shared_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder_2d,
      const int trajectory_id, PoseGraph2D* const pose_graph_2d,
      const LocalSlamResultCallback& local_slam_result_callback,
      // const absl::optional<MotionFilter>& pose_graph_odometry_motion_filter)
      const std::optional<MotionFilter>& pose_graph_odometry_motion_filter)
      : trajectory_id_(trajectory_id),
        pose_graph_2d_(pose_graph_2d),
        local_trajectory_builder_2d_(std::move(local_trajectory_builder_2d)),
        local_slam_result_callback_(local_slam_result_callback),
        pose_graph_odometry_motion_filter_(pose_graph_odometry_motion_filter) {}
  ~GlobalTrajectoryBuilder2D()  {}

  GlobalTrajectoryBuilder2D(const GlobalTrajectoryBuilder2D&) = delete;
  GlobalTrajectoryBuilder2D& operator=(const GlobalTrajectoryBuilder2D&) = delete;

  void AddSensorData(const std::string& sensor_id,
                     const sensor::TimedPointCloudData& timed_point_cloud_data);

  void AddSensorData(const std::string& sensor_id,
                     const sensor::ImuData& imu_data);

  void AddSensorData(const std::string& sensor_id,
                     const sensor::OdometryData& odometry_data);

  // void AddLocalSlamResultData(
  //     std::unique_ptr<cartographer::mapping::LocalSlamResultData>
  //         local_slam_result_data);

 private:
  const int trajectory_id_;
  PoseGraph2D* const pose_graph_2d_;
  std::shared_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder_2d_;
  LocalSlamResultCallback local_slam_result_callback_;
  // absl::optional<MotionFilter> pose_graph_odometry_motion_filter_;
  std::optional<MotionFilter> pose_graph_odometry_motion_filter_;
};

// proto::SensorId ToProto(const SensorId& sensor_id);
// SensorId FromProto(const proto::SensorId& sensor_id_proto);
}  // namespace
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_GLOBAL_TRAJECTORY_BUILDER_H_
