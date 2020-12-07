"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_test_group")

isaac_cc_test_group(
    srcs = [
        "application_test.cpp",
        "message_test.cpp",
        "thread.cpp",
    ],
    data = [
        "test.app.json",
        "thread.app.json",
        "//packages/message_generators:libmessage_generators_module.so",
    ],
    tags = ["host"],
    deps = [
        "//engine/alice/c_api",
        "//engine/core",
    ],
)
