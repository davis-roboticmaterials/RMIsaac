"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_test(
    name = "kinematic",
    srcs = ["kinematic.cpp"],
    deps = [
        "//engine/core/math",
        "//engine/gems/kinematic_tree",
        "//engine/gems/kinematic_tree:kinematic",
        "//engine/gems/kinematic_tree:motors",
        "//engine/gems/math:test_utils",
        "@gtest//:main",
    ],
)

cc_test(
    name = "json_loader",
    srcs = ["json_loader.cpp"],
    data = glob(["*.kinematic.json"]),
    deps = [
        "//engine/core/math",
        "//engine/gems/kinematic_tree",
        "//engine/gems/kinematic_tree:json_loader",
        "//engine/gems/kinematic_tree:kinematic",
        "//engine/gems/kinematic_tree:motors",
        "//engine/gems/math:test_utils",
        "@gtest//:main",
    ],
)
