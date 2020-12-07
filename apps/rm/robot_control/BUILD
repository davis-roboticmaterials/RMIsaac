"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_jupyter_app")

isaac_jupyter_app(
    name = "rm_isaac_bridge",
    data = [
        "//apps/assets/kinematic_trees",
        "//packages/navsim/apps:navsim_tcp_subgraph",
        "//packages/planner/apps:multi_joint_lqr_control_subgraph",
    ],
    modules = [
        "sight",
        "viewers",
        "rgbd_processing"
    ],
    notebook = "rm_isaac_bridge.ipynb",
    visibility = ["//visibility:public"],
)