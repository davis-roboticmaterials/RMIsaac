/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/alice/components/PoseFromFile.hpp"

#include <string>

#include "engine/core/math/pose3.hpp"
#include "engine/gems/serialization/json_formatter.hpp"

namespace isaac {
namespace alice {

void PoseFromFile::start() {
  // Read file to json
  const std::string filename = get_filename();
  auto maybe_json = serialization::TryLoadJsonFromFile(filename);
  if (!maybe_json) {
    reportFailure("Failed to load json from '%s'", filename.c_str());
    return;
  }

  // Read pose from json
  auto maybe_pose = serialization::TryGet<Pose3d>(*maybe_json);
  if (!maybe_pose) {
    reportFailure("Failed to parse pose in '%s'", filename.c_str());
    return;
  }

  // Set the pose
  const std::string lhs_frame = get_lhs_frame();
  const std::string rhs_frame = get_rhs_frame();
  const bool ok = node()->pose().trySet(lhs_frame.c_str(), rhs_frame.c_str(), *maybe_pose, 0.0);
  if (!ok) {
    reportFailure("Failed to set %s_T_%s", lhs_frame.c_str(), rhs_frame.c_str());
    return;
  }
  reportSuccess();
}

}  // namespace alice
}  // namespace isaac
