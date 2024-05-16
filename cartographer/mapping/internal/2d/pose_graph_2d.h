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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_

#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
// #include "absl/container/flat_hash_map.h"
// #include "absl/synchronization/mutex.h"
#include <mutex>

#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/constraints/constraint_builder_2d.h"
#include "cartographer/mapping/internal/optimization/optimization_problem_2d.h"
#include "cartographer/mapping/internal/pose_graph_data.h"
#include "cartographer/mapping/internal/trajectory_connectivity_state.h"
#include "cartographer/mapping/internal/work_queue.h"
// #include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/pose_graph_trimmer.h"
#include "cartographer/mapping/value_conversion_tables.h"
// #include "cartographer/metrics/family_factory.h"
// #include "cartographer/sensor/fixed_frame_pose_data.h"
// #include "cartographer/sensor/landmark_data.h"
#include "cartographer/common/config.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
namespace cartographer {
namespace mapping {

// Implements the loop closure method called Sparse Pose Adjustment (SPA) from
// Konolige, Kurt, et al. "Efficient sparse pose adjustment for 2d mapping."
// Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference
// on (pp. 22--29). IEEE, 2010.
//
// It is extended for submapping:
// Each node has been matched against one or more submaps (adding a constraint
// for each match), both poses of nodes and of submaps are to be optimized.
// All constraints are between a submap i and a node j.
class PoseGraph2D {
 public:
  //   PoseGraph2D(
  //       const proto::PoseGraphOptions& options,
  //       std::unique_ptr<optimization::OptimizationProblem2D>
  //       optimization_problem, common::ThreadPool* thread_pool);
  PoseGraph2D(
      std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem,
      common::ThreadPool* thread_pool);
  ~PoseGraph2D();
  PoseGraph2D(const PoseGraph2D&) = delete;
  PoseGraph2D& operator=(const PoseGraph2D&) = delete;
  // Adds a new node with 'constant_data'. Its 'constant_data->local_pose' was
  // determined by scan matching against 'insertion_submaps.front()' and the
  // node data was inserted into the 'insertion_submaps'. If
  // 'insertion_submaps.front().finished()' is 'true', data was inserted into
  // this submap for the last time.
  NodeId AddNode(
      std::shared_ptr<const TrajectoryNode::Data> constant_data,
      int trajectory_id,
      const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps);
  void AddImuData(int trajectory_id, const sensor::ImuData& imu_data);
  void AddOdometryData(int trajectory_id,
                       const sensor::OdometryData& odometry_data);
  //   void AddFixedFramePoseData(
  //       int trajectory_id,
  //       const sensor::FixedFramePoseData& fixed_frame_pose_data)
  //       ;
  //   void AddLandmarkData(int trajectory_id,
  //                        const sensor::LandmarkData& landmark_data)
  //       ;

  void DeleteTrajectory(int trajectory_id);
  void FinishTrajectory(int trajectory_id);
  bool IsTrajectoryFinished(int trajectory_id) const;
  void FreezeTrajectory(int trajectory_id);
  bool IsTrajectoryFrozen(int trajectory_id) const;
  void AddSubmapFromProto(const transform::Rigid3d& global_submap_pose,
                          const proto::Submap& submap);
  void AddNodeFromProto(const transform::Rigid3d& global_pose,
                        const proto::Node& node);
  void SetTrajectoryDataFromProto(const proto::TrajectoryData& data);
  void AddNodeToSubmap(const NodeId& node_id, const SubmapId& submap_id);
  void AddSerializedConstraints(const std::vector<Constraint>& constraints);
  void AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer);
  void RunFinalOptimization();
  std::vector<std::vector<int>> GetConnectedTrajectories() const;
  ::cartographer::mapping::SubmapData GetSubmapData(
      const SubmapId& submap_id) const;
  MapById<SubmapId, ::cartographer::mapping::SubmapData> GetAllSubmapData()
      const;
  MapById<SubmapId, SubmapPose> GetAllSubmapPoses() const;
  transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id) const;
  MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const;
  //   MapById<SubmapId, TrajectoryNode> GetSubmapNodes() const;
  MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses() const;
  MapById<NodeId, TrajectoryNodePose> GetNodePosesBySubmapId(
      const SubmapId& submap_id) const;
  std::map<int, TrajectoryState> GetTrajectoryStates() const;
  //   std::map<std::string, transform::Rigid3d> GetLandmarkPoses() const
  //
  //       ;
  //   void SetLandmarkPose(const std::string& landmark_id,
  //                        const transform::Rigid3d& global_pose,
  //                        const bool frozen = false)
  //       ;
  sensor::MapByTime<sensor::ImuData> GetImuData() const;
  sensor::MapByTime<sensor::OdometryData> GetOdometryData() const;
  //   sensor::MapByTime<sensor::FixedFramePoseData> GetFixedFramePoseData()
  //       const  ;
  //   std::map<std::string /* landmark ID */, PoseGraph::LandmarkNode>
  //   GetLandmarkNodes() const  ;
  std::map<int, TrajectoryData> GetTrajectoryData() const;
  std::vector<Constraint> constraints() const;
  void SetInitialTrajectoryPose(int from_trajectory_id, int to_trajectory_id,
                                const transform::Rigid3d& pose,
                                const common::Time time);
  void SetGlobalSlamOptimizationCallback(
      std::function<void(const std::map<int, SubmapId>&,
                         const std::map<int, NodeId>&)>
          callback);
  transform::Rigid3d GetInterpolatedGlobalTrajectoryPose(
      int trajectory_id, const common::Time time) const;

  //   static void RegisterMetrics(metrics::FamilyFactory* family_factory);
  proto::PoseGraph ToProto(bool include_unfinished_submaps) const;

 private:
  std::condition_variable cv_;
  MapById<SubmapId, ::cartographer::mapping::SubmapData>
  GetSubmapDataUnderLock() const;

  // Handles a new work item.
  void AddWorkItem(const std::function<WorkItem::Result()>& work_item);

  // Adds connectivity and sampler for a trajectory if it does not exist.
  void AddTrajectoryIfNeeded(int trajectory_id);

  // Appends the new node and submap (if needed) to the internal data
  // structures.
  NodeId AppendNode(
      std::shared_ptr<const TrajectoryNode::Data> constant_data,
      int trajectory_id,
      const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps,
      const transform::Rigid3d& optimized_pose);

  // Grows the optimization problem to have an entry for every element of
  // 'insertion_submaps'. Returns the IDs for the 'insertion_submaps'.
  std::vector<SubmapId> InitializeGlobalSubmapPoses(
      int trajectory_id, const common::Time time,
      const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps);

  // Adds constraints for a node, and starts scan matching in the background.
  WorkItem::Result ComputeConstraintsForNode(
      const NodeId& node_id,
      std::vector<std::shared_ptr<const Submap2D>> insertion_submaps,
      bool newly_finished_submap);

  // Computes constraints for a node and submap pair.
  void ComputeConstraint(const NodeId& node_id, const SubmapId& submap_id);

  // Deletes trajectories waiting for deletion. Must not be called during
  // constraint search.
  void DeleteTrajectoriesIfNeeded();

  // Runs the optimization, executes the trimmers and processes the work queue.
  void HandleWorkQueue(const constraints::ConstraintBuilder2D::Result& result);

  // Process pending tasks in the work queue on the calling thread, until the
  // queue is either empty or an optimization is required.
  void DrainWorkQueue();

  // Waits until we caught up (i.e. nothing is waiting to be scheduled), and
  // all computations have finished.
  void WaitForAllComputations();

  // Runs the optimization. Callers have to make sure, that there is only one
  // optimization being run at a time.
  void RunOptimization();

  bool CanAddWorkItemModifying(int trajectory_id);

  // Computes the local to global map frame transform based on the given
  // 'global_submap_poses'.
  transform::Rigid3d ComputeLocalToGlobalTransform(
      const MapById<SubmapId, optimization::SubmapSpec2D>& global_submap_poses,
      int trajectory_id) const;

  SubmapData GetSubmapDataUnderLock(const SubmapId& submap_id) const;

  common::Time GetLatestNodeTime(const NodeId& node_id,
                                 const SubmapId& submap_id) const;

  // Updates the trajectory connectivity structure with a new constraint.
  void UpdateTrajectoryConnectivity(const Constraint& constraint);

  //   const proto::PoseGraphOptions options_;
  std::function<void(const std::map<int, SubmapId>&,
                     const std::map<int, NodeId>&)>
      global_slam_optimization_callback_;
  //   mutable absl::Mutex mutex_;
  //   absl::Mutex work_queue_mutex_;
  mutable std::mutex mutex_;
  std::mutex work_queue_mutex_;

  // If it exists, further work items must be added to this queue, and will be
  // considered later.
  std::unique_ptr<WorkQueue> work_queue_;

  // We globally localize a fraction of the nodes from each trajectory.
  //   absl::flat_hash_map<int, std::unique_ptr<common::FixedRatioSampler>>
  //       global_localization_samplers_ ;

  std::unordered_map<int, std::unique_ptr<common::FixedRatioSampler>>
      global_localization_samplers_;
  // Number of nodes added since last loop closure.
  size_t num_nodes_since_last_loop_closure_ = 0;

  // Current optimization problem.
  std::unique_ptr<optimization::OptimizationProblem2D> optimization_problem_;
  constraints::ConstraintBuilder2D constraint_builder_;

  // Thread pool used for handling the work queue.
  common::ThreadPool* const thread_pool_;

  // List of all trimmers to consult when optimizations finish.
  std::vector<std::unique_ptr<PoseGraphTrimmer>> trimmers_;

  PoseGraphData data_;

  ValueConversionTables conversion_tables_;

  // Allows querying and manipulating the pose graph by the 'trimmers_'. The
  // 'mutex_' of the pose graph is held while this class is used.
  class TrimmingHandle : public Trimmable {
   public:
    TrimmingHandle(PoseGraph2D* parent);
    ~TrimmingHandle() {}

    int num_submaps(int trajectory_id) const;
    std::vector<SubmapId> GetSubmapIds(int trajectory_id) const;
    MapById<SubmapId, SubmapData> GetOptimizedSubmapData() const;
    const MapById<NodeId, TrajectoryNode>& GetTrajectoryNodes() const;
    const std::vector<Constraint>& GetConstraints() const;
    void TrimSubmap(const SubmapId& submap_id);
    bool IsFinished(int trajectory_id) const;
    void SetTrajectoryState(int trajectory_id, TrajectoryState state);

   private:
    PoseGraph2D* const parent_;
  };
};

std::vector<Constraint> FromProto(
    const ::google::protobuf::RepeatedPtrField<
        ::cartographer::mapping::proto::PoseGraph::Constraint>&
        constraint_protos);
proto::PoseGraph::Constraint ToProto(const Constraint& constraint);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_2D_H_
