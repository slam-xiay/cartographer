#ifndef CARTOGRAPHER_COMMON_CONFIG_H_
#define CARTOGRAPHER_COMMON_CONFIG_H_
#include "math.h"

namespace cartographer {

constexpr size_t kBackgroundThreadsCount = 4;  // num_background_threads
constexpr bool kCollateByTrajectory = false;   // collate_by_trajectory

constexpr bool kUseImuData = true;
constexpr double kLidarMinRange = 0.2;
constexpr double kLidarMaxRange = 50.0;
constexpr double kMinHeight = -0.8;
constexpr double kMaxHeight = 2.0;
constexpr double kMissingDataRayDistance = 5.0;
constexpr size_t kAccumulatedRangeCount = 1;
constexpr double kVoxelFilterSize = 0.025;

constexpr double kAdaptiveVoxelFilterMaxLength = 0.5;
constexpr size_t kAdaptiveVoxelFilterCount = 200;
constexpr double kAdaptiveVoxelFilterMaxRange = 50.;

constexpr double kLoopClosureAdaptiveVoxelFilterMaxLength = 0.9;
constexpr size_t kLoopClosureAdaptiveVoxelFilterMaxCount = 200;
constexpr double kLoopClosureAdaptiveVoxelFilterMaxRange = 50.;

constexpr bool kUseOnlineCSM = false;
constexpr double kRealTimeCSMLinearSearchWindow = 0.1;
constexpr double kRealTimeCSMAngularSearchWindow = common::DegToRad(20.);
constexpr double kRealTimeCSMTranslationDeltaCostWeight =
    1e-1;  // translation_delta_cost_weight
constexpr double kRealTimeCSMRotationDeltaCostWeight =
    1e-1;  // options_.rotation_delta_cost_weight()

constexpr double kCSMOccupiedSapceWeight = 1.;  // occupied_space_weight
constexpr double kCSMTranslationWeight = 10.;
constexpr double kCSMRotationWeight = 40.;
constexpr bool kCSMUseNonmonotonicSteps = false;
constexpr double kCSMMaxIterationsCount = 20;    // max_num_iterations
constexpr size_t kCSMCeresThreadsCount = 1;      // num_threads
constexpr double kMotionFilterMaxDuration = 5.;  // max_time_seconds
constexpr double kMotionFilterTranslation = 0.2;
constexpr double kMotionFilterRotaiton = common::DegToRad(1.);

constexpr double kImuGravityTimeConstant = 10.;  // imu_gravity_time_constant
constexpr bool kPoseExtrapolatorUseImuBase = false;
constexpr bool kPoseExtrapolatorDuration = 0.5;  // pose_queue_duration

constexpr size_t kSubmapsNodeCount = 50;  // num_range_data
constexpr double kResolution = 0.10;      // resolution
constexpr bool kSubmapsInsertFreeSpace = true;  // insert_free_space
constexpr double kSubmapsHitPorbility = 0.55;
constexpr double kSubmapsMissPorbility = 0.49;

constexpr size_t kOptimizeEveryNNodes = 90;
constexpr double kSamplingRatio = 0.3;
constexpr double kMaxConstrantDistance = 15;
constexpr double kLocalMatchMinScore = 0.55;  // 注意重复
constexpr double kGlobalMatchMinScore = 0.6;
constexpr double kLoopClosureTranslationWeight = 1.1e4;
constexpr double kLoopClosureRotationWeight = 1.1e4;
constexpr bool kLogMatches = false;
constexpr double kFastCSMLinearSearchWindow = 7.;
constexpr double kFastCSMAngularSearchWindow = common::DegToRad(30.);
constexpr size_t kBranchAndBoundDepth = 7;
constexpr double kFastCSMOccupiedSapceWeight = 20.;
constexpr double kFastCSMTranslationWeight = 10.;
constexpr double kFastCSMRotationWeight = 1.;
constexpr double kFastCSMUseNonmonotonicSteps = true;
constexpr double kFastCSMCeresMaxIterationsCount = 10;
constexpr double kFastCSMCeresThreadsCount = 1;

// optimization
constexpr double kHuberScale = 1e1;  // huber_scale
constexpr double kLocalSlmPoseTranslationWeight = 1e5;
// local_slam_pose_translation_weight
constexpr double kLocalSlamPoseRotationWeight = 1e5;
// local_slam_pose_rotation_weight
constexpr double kOdometryTranslationWeight = 1e5;
// odometry_translation_weight
constexpr double kOdometryRotationWeight = 1e5;  // odometry_rotation_weight
constexpr bool kLogSolverSummary = false;        // log_solver_summary
// constexpr bool kUseOnlineImuExtrinsicsIn3d = true;
// use_online_imu_extrinsics_in_3d
// constexpr bool kFixZIn3D = false;  // fix_z_in_3d
constexpr double kOptimizationUseNonmonotonicSteps = true;
// use_nonmonotonic_steps
constexpr double kOptimizationMaxIterationsCount = 10;  // max_num_iterations
constexpr double kOptimizationThreadsCount = 7;         // num_threads
constexpr double kMaxFinalIterationsCount = 50;  // max_num_final_iterations
constexpr double kGlobalSamplingRatio = 0.003;   // global_sampling_ratio
constexpr bool kLogResidualHistograms = false;   // log_residual_histograms
constexpr double kGlobalConstraintSearchInterval = 10;
// global_constraint_search_after_n_seconds

}  // namespace cartographer
#endif