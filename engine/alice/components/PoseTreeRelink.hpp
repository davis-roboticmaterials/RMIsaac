/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/alice_codelet.hpp"

namespace isaac {
namespace alice {

// At start, changes the parent of rhs_frame from current_lhs_frame to desired_lhs_frame. Reports
// success if parent is changed and reports failure otherwise. This can be used, for example, after
// a robot lifts a cart that is detected in the world frame. After the lift, the cart would move
// with the robot, so the parent of the cart frame can be updated with this codelet from world to
// robot.
class PoseTreeRelink : public Codelet {
 public:
  void start() override;

  // Name of the current reference frame of the left side of the pose
  ISAAC_PARAM(std::string, current_lhs_frame);
  // Name of the desired reference frame of the left side of the pose
  ISAAC_PARAM(std::string, desired_lhs_frame);
  // Name of the reference frame of the right side of the pose
  ISAAC_PARAM(std::string, rhs_frame);
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::PoseTreeRelink);
