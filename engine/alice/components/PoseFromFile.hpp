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

// Reads pose from json file on disk and sets it to the PoseTree. This codelet can be used, for
// example, to read last known localization, which was written to disk by PoseFromFile codelet.
class PoseFromFile : public Codelet {
 public:
  void start() override;

  // Name of the reference frame of the left side of the pose
  ISAAC_PARAM(std::string, lhs_frame);
  // Name of the reference frame of the right side of the pose
  ISAAC_PARAM(std::string, rhs_frame);
  // Path to the file that contains the pose json. Expected layout:
  // {
  //   "rotation": { "yaw_degrees": 180.0 },
  //   "translation": [25.0, 20.0, 0.0]
  // }
  // or
  // [
  //    0.0,
  //    0.0,
  //    0.0,
  //   -1.0,
  //    25.0,
  //    20.0,
  //    0.0
  // ]
  // Please see "What is the syntax for setting a Pose3d in json?"
  // in FAQ of documentation for more details.
  ISAAC_PARAM(std::string, filename);
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::PoseFromFile);
