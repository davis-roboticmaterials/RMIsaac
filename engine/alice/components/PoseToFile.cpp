/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/alice/components/PoseToFile.hpp"

#include "engine/core/math/pose3.hpp"
#include "engine/gems/serialization/json_formatter.hpp"

namespace isaac {
namespace alice {

void PoseToFile::start() {
  tickPeriodically();
}

void PoseToFile::tick() {
  // Read pose from PoseTree
  auto maybe_pose = node()->pose().tryGet(get_lhs_frame(), get_rhs_frame(), getTickTime());
  if (!maybe_pose) {
    return;
  }

  // Write pose as json file
  Json json;
  serialization::Set(json, *maybe_pose);
  serialization::WriteJsonToFile(get_filename(), json);
}

}  // namespace alice
}  // namespace isaac
