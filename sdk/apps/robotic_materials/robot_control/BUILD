"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

# DEPLOY: ./../engine/engine/build/deploy.sh -p //apps/robotic_materials/robot_control:rm_isaac_bridge-pkg -h localhost -d x86_64

load("//bzl:py.bzl", "isaac_jupyter_app", "isaac_py_app")

isaac_jupyter_app(
    name = "rm_isaac_bridge",
    data = [
        "//apps/assets/kinematic_trees",
        "//packages/navsim/apps:navsim_tcp_subgraph",
        "//packages/planner/apps:multi_joint_lqr_control_subgraph",
        "//packages/ros_bridge/apps:ros_to_perception_subgraph"
    ],
    modules = [
        "sight",
        "viewers",
        "rgbd_processing"
    ],
    notebook = "rm_isaac_bridge.ipynb",
    visibility = ["//visibility:public"],
)

# load("//bzl:module.bzl", "isaac_py_app")
# isaac_py_app(
#     name = "rm_isaac_bridge",
#     data = [
#         "//apps/assets/kinematic_trees",
#         "//packages/navsim/apps:navsim_tcp_subgraph",
#         "//packages/planner/apps:multi_joint_lqr_control_subgraph",
#         "//packages/ros_bridge/apps:ros_to_perception_subgraph"
#     ],
#     modules = [
#         "sight",
#         "viewers",
#         "rgbd_processing"
#     ],
#     deps = [
#         "//engine/pyalice",
#     ],
# )
