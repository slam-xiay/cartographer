/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/internal/2d/global_trajectory_builder_2d.h"

#include <memory>

#include "absl/memory/memory.h"
#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
  void GlobalTrajectoryBuilder2D::AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data)  {
    CHECK(local_trajectory_builder_2d_)
        << "Cannot add TimedPointCloudData without a LocalTrajectoryBuilder.";
    std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult> matching_result =
        local_trajectory_builder_2d_->AddRangeData(sensor_id,
                                                   timed_point_cloud_data);
    if (matching_result == nullptr) {
      return;
    }
    std::unique_ptr<InsertionResult> insertion_result;
    if (matching_result->insertion_result != nullptr) {
      auto node_id = pose_graph_2d_->AddNode(
          matching_result->insertion_result->constant_data, trajectory_id_,
          matching_result->insertion_result->insertion_submaps);
      CHECK_EQ(node_id.trajectory_id, trajectory_id_);
      insertion_result = std::make_unique<InsertionResult>(InsertionResult{
          node_id, matching_result->insertion_result->constant_data,
          std::vector<std::shared_ptr<const Submap>>(
              matching_result->insertion_result->insertion_submaps.begin(),
              matching_result->insertion_result->insertion_submaps.end())});
    }
    if (local_slam_result_callback_) {
      local_slam_result_callback_(
          trajectory_id_, matching_result->time, matching_result->local_pose,
          std::move(matching_result->range_data_in_local),
          std::move(insertion_result));
    }
  }

  void GlobalTrajectoryBuilder2D::AddSensorData(const std::string& sensor_id,
                     const sensor::ImuData& imu_data)  {
    if (local_trajectory_builder_2d_) {
      local_trajectory_builder_2d_->AddImuData(imu_data);
    }
    pose_graph_2d_->AddImuData(trajectory_id_, imu_data);
  }

  void GlobalTrajectoryBuilder2D::AddSensorData(const std::string& sensor_id,
                     const sensor::OdometryData& odometry_data)  {
    CHECK(odometry_data.pose.IsValid()) << odometry_data.pose;
    if (local_trajectory_builder_2d_) {
      local_trajectory_builder_2d_->AddOdometryData(odometry_data);
    }
    if (pose_graph_odometry_motion_filter_.has_value() &&
        pose_graph_odometry_motion_filter_.value().IsSimilar(
            odometry_data.time, odometry_data.pose)) {
      return;
    }
    pose_graph_2d_->AddOdometryData(trajectory_id_, odometry_data);
  }

  // void GlobalTrajectoryBuilder2D::AddLocalSlamResultData(
  //     std::unique_ptr<cartographer::mapping::LocalSlamResultData>
  //         local_slam_result_data) {
  //   CHECK(!local_trajectory_builder_2d_)
  //       << "Can't add LocalSlamResultData with "
  //          "local_trajectory_builder_2d_ present.";
  //   local_slam_result_data->AddToPoseGraph(trajectory_id_, pose_graph_2d_);
  // }

  // void PopulatePureLocalizationTrimmerOptions(
  //     proto::TrajectoryBuilderOptions* const trajectory_builder_options,
  //     common::LuaParameterDictionary* const parameter_dictionary) {
  //   constexpr char kDictionaryKey[] = "pure_localization_trimmer";
  //   if (!parameter_dictionary->HasKey(kDictionaryKey)) return;

  //   auto options_dictionary =
  //   parameter_dictionary->GetDictionary(kDictionaryKey); auto* options =
  //       trajectory_builder_options->mutable_pure_localization_trimmer();
  //   options->set_max_submaps_to_keep(
  //       options_dictionary->GetInt("max_submaps_to_keep"));
  // }

  // void PopulatePoseGraphOdometryMotionFilterOptions(
  //     proto::TrajectoryBuilderOptions* const trajectory_builder_options,
  //     common::LuaParameterDictionary* const parameter_dictionary) {
  //   constexpr char kDictionaryKey[] = "pose_graph_odometry_motion_filter";
  //   if (!parameter_dictionary->HasKey(kDictionaryKey)) return;

  //   auto options_dictionary =
  //   parameter_dictionary->GetDictionary(kDictionaryKey); auto* options =
  //       trajectory_builder_options->mutable_pose_graph_odometry_motion_filter();
  //   options->set_max_time_seconds(
  //       options_dictionary->GetDouble("max_time_seconds"));
  //   options->set_max_distance_meters(
  //       options_dictionary->GetDouble("max_distance_meters"));
  //   options->set_max_angle_radians(
  //       options_dictionary->GetDouble("max_angle_radians"));
  // }

  // }  // namespace

  // proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
  //     common::LuaParameterDictionary* const parameter_dictionary) {
  //   proto::TrajectoryBuilderOptions options;
  //   *options.mutable_trajectory_builder_2d_options() =
  //       CreateLocalTrajectoryBuilderOptions2D(
  //           parameter_dictionary->GetDictionary("trajectory_builder_2d").get());
  //   // *options.mutable_trajectory_builder_3d_options() =
  //   //     CreateLocalTrajectoryBuilderOptions3D(
  //   // parameter_dictionary->GetDictionary("trajectory_builder_3d").get());
  //   // options.set_collate_fixed_frame(
  //   //     parameter_dictionary->GetBool("collate_fixed_frame"));
  //   // options.set_collate_landmarks(
  //   //     parameter_dictionary->GetBool("collate_landmarks"));
  //   PopulatePureLocalizationTrimmerOptions(&options, parameter_dictionary);
  //   PopulatePoseGraphOdometryMotionFilterOptions(&options,
  //   parameter_dictionary); return options;
  // }

  // proto::SensorId ToProto(const SensorId& sensor_id) {
  //   proto::SensorId sensor_id_proto;
  //   switch (sensor_id.type) {
  //     case SensorId::SensorType::RANGE:
  //       sensor_id_proto.set_type(proto::SensorId::RANGE);
  //       break;
  //     case SensorId::SensorType::IMU:
  //       sensor_id_proto.set_type(proto::SensorId::IMU);
  //       break;
  //     case SensorId::SensorType::ODOMETRY:
  //       sensor_id_proto.set_type(proto::SensorId::ODOMETRY);
  //       break;
  //     case SensorId::SensorType::LOCAL_SLAM_RESULT:
  //       sensor_id_proto.set_type(proto::SensorId::LOCAL_SLAM_RESULT);
  //       break;
  //     default:
  //       LOG(FATAL) << "Unsupported sensor type.";
  //   }
  //   sensor_id_proto.set_id(sensor_id.id);
  //   return sensor_id_proto;
  // }

  // SensorId FromProto(const proto::SensorId& sensor_id_proto) {
  //   SensorId sensor_id;
  //   switch (sensor_id_proto.type()) {
  //     case proto::SensorId::RANGE:
  //       sensor_id.type = SensorId::SensorType::RANGE;
  //       break;
  //     case proto::SensorId::IMU:
  //       sensor_id.type = SensorId::SensorType::IMU;
  //       break;
  //     case proto::SensorId::ODOMETRY:
  //       sensor_id.type = SensorId::SensorType::ODOMETRY;
  //       break;
  //     case proto::SensorId::LOCAL_SLAM_RESULT:
  //       sensor_id.type = SensorId::SensorType::LOCAL_SLAM_RESULT;
  //       break;
  //     default:
  //       LOG(FATAL) << "Unsupported sensor type.";
  //   }
  //   sensor_id.id = sensor_id_proto.id();
  //   return sensor_id;
  // }

}  // namespace mapping
}  // namespace cartographer
