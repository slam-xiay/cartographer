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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include <memory>

#include "cartographer/common/thread_pool.h"
// #include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/internal/global_trajectory_builder.h"
// #include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/pose_graph_data.h"
// #include "cartographer/mapping/proto/map_builder_options.pb.h"
// #include "cartographer/sensor/collator_interface.h"
#include <set>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/config.h"
// #include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/internal/pose_graph_data.h"
// #include "cartographer/mapping/pose_graph_interface.h"
// #include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
// #include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
// #include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/internal/collator.h"

namespace cartographer {
namespace mapping {

// proto::MapBuilderOptions CreateMapBuilderOptions(
//     common::LuaParameterDictionary* const parameter_dictionary);
// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.
class MapBuilder {
 public:
  //   using LocalSlamResultCallback =
  //       TrajectoryBuilderInterface::LocalSlamResultCallback;

  //   using SensorId = TrajectoryBuilderInterface::SensorId;
  explicit MapBuilder();
  // explicit MapBuilder(const proto::MapBuilderOptions &options);
  ~MapBuilder() {}

  MapBuilder(const MapBuilder &) = delete;
  MapBuilder &operator=(const MapBuilder &) = delete;

  //   int AddTrajectoryBuilder(
  //       const std::set<SensorId>& expected_sensor_ids,
  //       const proto::TrajectoryBuilderOptions& trajectory_options,
  //       LocalSlamResultCallback local_slam_result_callback);

  int AddTrajectoryBuilder(
      const std::set<SensorId>& expected_sensor_ids,
      //   const proto::TrajectoryBuilderOptions& trajectory_options,
      LocalSlamResultCallback local_slam_result_callback);

  //   int AddTrajectoryForDeserialization(
  //       const proto::TrajectoryBuilderOptionsWithSensorIds&
  //           options_with_sensor_ids_proto);
  int AddTrajectoryForDeserialization();
  void FinishTrajectory(int trajectory_id);

  MapById<SubmapId, ::cartographer::mapping::SubmapData> GetAllSubmapData();

  std::string SubmapToProto(const SubmapId& submap_id,
                            proto::SubmapQuery::Response* response);

  void SerializeState(bool include_unfinished_submaps,
                      io::ProtoStreamWriter* writer);

  bool SerializeStateToFile(bool include_unfinished_submaps,
                            const std::string& filename);

  std::map<int, int> LoadState(io::ProtoStreamReader* reader,
                               bool load_frozen_state);

  std::map<int, int> LoadStateFromFile(const std::string& filename,
                                       const bool load_frozen_state);

  PoseGraph2D* pose_graph() { return pose_graph_2d_.get(); }

  int num_trajectory_builders() const { return trajectory_builders_.size(); }

  GlobalTrajectoryBuilder2D* GetTrajectoryBuilder(int trajectory_id) const {
    return trajectory_builders_.at(trajectory_id).get();
  }

  //   const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>&
  //   GetAllTrajectoryBuilderOptions() const {
  //     return all_trajectory_builder_options_;
  //   }

 private:
  //   const proto::MapBuilderOptions options_;
  common::ThreadPool thread_pool_;

  std::unique_ptr<PoseGraph2D> pose_graph_2d_;

  // std::unique_ptr<sensor::CollatorInterface> sensor_collator_;
  std::unique_ptr<sensor::Collator> sensor_collator_;
  //   std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>>
  //       trajectory_builders_;
  std::vector<std::shared_ptr<mapping::GlobalTrajectoryBuilder2D>>
      trajectory_builders_;
  //   std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
  //       all_trajectory_builder_options_;
};

// std::unique_ptr<MapBuilder> CreateMapBuilder(
//     const proto::MapBuilderOptions& options);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
