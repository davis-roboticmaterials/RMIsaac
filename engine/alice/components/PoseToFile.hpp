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

// Reads pose from PoseTree and writes it as json to disk. This codelet can be used, for example, to
// save the last known localization. PoseFromFile codelet can then read the pose to continue
// localization from there.
class PoseToFile : public Codelet {
 public:
  void start() override;
  void tick() override;

  // Name of the reference frame of the left side of the pose
  ISAAC_PARAM(std::string, lhs_frame);
  // Name of the reference frame of the right side of the pose
  ISAAC_PARAM(std::string, rhs_frame);
  // Path for the file to create with pose as json
  ISAAC_PARAM(std::string, filename);
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::PoseToFile);
