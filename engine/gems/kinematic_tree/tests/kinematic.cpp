/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/kinematic_tree/kinematic.hpp"

#include <random>

#include "gtest/gtest.h"

#include "engine/gems/kinematic_tree/constant_motor.hpp"
#include "engine/gems/kinematic_tree/kinematic_tree.hpp"
#include "engine/gems/kinematic_tree/revolute_motor.hpp"
#include "engine/gems/math/test_utils.hpp"

namespace isaac {
namespace kinematic_tree {

namespace {
// Create an example of manipulator with 8 joints
KinematicTree CreateEightJointChain() {
  KinematicTree model;
  auto* link = model.addLink("root", nullptr,
                             std::make_unique<RevoluteMotor>(Vector3d(0.0, 0.0, 1.0)));
  link = model.addLink(
      "axis1", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(0.0, 0.0, 0.4))));
  link = model.addLink("j1", link, std::make_unique<RevoluteMotor>(Vector3d(0.0, 1.0, 0.0)));
  link = model.addLink(
      "axis2", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(0.0, 0.1, 0.2))));
  link = model.addLink("j2", link, std::make_unique<RevoluteMotor>(Vector3d(0.0, 0.0, 1.0)));
  link = model.addLink(
      "axis3", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(0.0, 0.0, 0.2))));
  link = model.addLink("j3", link, std::make_unique<RevoluteMotor>(Vector3d(1.0, 0.0, 0.0)));
  link = model.addLink(
      "axis4", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(0.1, 0.0, 0.2))));
  link = model.addLink("j4", link, std::make_unique<RevoluteMotor>(Vector3d(0.0, 0.0, 1.0)));
  link = model.addLink(
      "axis5", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(0.0, 0.3, 0.0))));
  link = model.addLink("j5", link, std::make_unique<RevoluteMotor>(Vector3d(0.0, 1.0, 0.0)));
  link = model.addLink(
      "axis6", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(0.0, 0.3, 0.0))));
  link = model.addLink("j6", link, std::make_unique<RevoluteMotor>(Vector3d(1.0, 0.0, 0.0)));
  link = model.addLink(
      "axis7", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(0.3, 0.0, 0.0))));
  link = model.addLink("j7", link, std::make_unique<RevoluteMotor>(Vector3d(0.0, 0.0, 1.0)));
  return model;
}
}  // namespace

TEST(ik, random) {
  auto manipulator = CreateEightJointChain();
  const int end_effector_id = manipulator.getNumberOfLinks() - 1;
  int failures = 0;
  for (int i = 0; i < 1000; i++) {
    const VectorXd state = VectorXd::Random(manipulator.getMotorStateDimensions());
    const auto target = ForwardKinematic(manipulator, state, end_effector_id);
    const auto ik = InverseKinematic(manipulator, target, end_effector_id, VectorXd::Zero(8));
    if (!ik) {
      failures++;
      continue;
    }
    const auto pose = ForwardKinematic(manipulator, *ik, end_effector_id).toPose3();
    ISAAC_EXPECT_POSE_NEAR(target.toPose3(), pose, 1e-3);
  }
  EXPECT_LE(failures, 20);
}

}  // namespace planner
}  // namespace isaac
