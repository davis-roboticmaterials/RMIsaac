/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/alice/components/PoseTreeRelink.hpp"

#include <string>

#include "engine/alice/components/Pose.hpp"

namespace isaac {
namespace alice {

void PoseTreeRelink::start() {
  // Read parameters
  const std::string current_lhs_frame = get_current_lhs_frame();
  const std::string desired_lhs_frame = get_desired_lhs_frame();
  const std::string rhs_frame = get_rhs_frame();

  // Store current time
  const double time = getTickTime();

  // Read poses
  auto maybe_current_lhs_T_rhs = node()->pose().tryGet(current_lhs_frame, rhs_frame, time);
  if (!maybe_current_lhs_T_rhs) {
    reportFailure("Failed to read %s_T_%s from PoseTree at %f", current_lhs_frame.c_str(),
                  rhs_frame.c_str(), time);
    return;
  }
  auto maybe_desired_lhs_T_current_lhs =
      node()->pose().tryGet(desired_lhs_frame, current_lhs_frame, time);
  if (!maybe_desired_lhs_T_current_lhs) {
    reportFailure("Failed to read %s_T_%s from PoseTree at %f", desired_lhs_frame.c_str(),
                  current_lhs_frame.c_str(), time);
    return;
  }
  auto desired_lhs_T_rhs = *maybe_desired_lhs_T_current_lhs * *maybe_current_lhs_T_rhs;

  // Remove edge
  const bool removed_edge = node()->pose().removeEdge(current_lhs_frame, rhs_frame, time);
  if (!removed_edge) {
    reportFailure("Failed to remove %s_T_%s from PoseTree at %f", current_lhs_frame.c_str(),
                  rhs_frame.c_str(), time);
    return;
  }

  // Set the new edge
  const bool wrote_edge =
      node()->pose().trySet(desired_lhs_frame, rhs_frame, desired_lhs_T_rhs, time);
  if (!wrote_edge) {
    reportFailure("Failed to set %s_T_%s to PoseTree at %f", desired_lhs_frame.c_str(),
                  rhs_frame.c_str(), time);
    return;
  }
  reportSuccess();
}

}  // namespace alice
}  // namespace isaac
