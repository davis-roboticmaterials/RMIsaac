/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Pose2Comparer.hpp"

namespace isaac {
namespace alice {

void Pose2Comparer::start() {
  tickPeriodically();
}

void Pose2Comparer::tick() {
  // Read poses
  const auto maybe_pose1 =
      node()->pose().tryGetPose2XY(get_first_lhs_frame(), get_first_rhs_frame(), getTickTime());
  const auto maybe_pose2 =
      node()->pose().tryGetPose2XY(get_second_lhs_frame(), get_second_rhs_frame(), getTickTime());
  if (!maybe_pose1 || !maybe_pose2) {
    return;
  }

  // Compute delta
  const Vector2d delta = PoseMagnitude(maybe_pose1->inverse() * *maybe_pose2);
  show("delta.p", delta[0]);
  show("delta.q", delta[1]);

  // Compare against threshold
  if (const auto maybe_threshold = try_get_threshold()) {
    const Vector2d& threshold = *maybe_threshold;
    if (delta[0] < threshold[0] && delta[1] < threshold[1]) {
      reportSuccess();
    }
  }
}

}  // namespace alice
}  // namespace isaac
