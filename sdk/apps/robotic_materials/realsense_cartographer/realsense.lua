
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
}

MAP_BUILDER.num_background_threads = 10
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.use_trajectory_builder_3d = false

-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 4e2
TRAJECTORY_BUILDER_2D.max_range = 10
TRAJECTORY_BUILDER_2D.min_range = 0.13
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 150
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 0.2
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10
POSE_GRAPH.matcher_rotation_weight = 1e0
POSE_GRAPH.matcher_translation_weight=1e2
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.constraint_builder.max_constraint_distance = 100
POSE_GRAPH.constraint_builder.min_score=0.65
POSE_GRAPH.constraint_builder.sampling_ratio = 0.9
POSE_GRAPH.global_constraint_search_after_n_seconds = 4

POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = true
-- POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1000
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1000
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.8
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1
POSE_GRAPH.optimize_every_n_nodes = 150

return options