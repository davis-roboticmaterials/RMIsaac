"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:pybind.bzl", "pybind_library")

pybind_library(
    name = "bindings",
    cc_deps = [
        "//engine/alice",
        "//engine/alice/tools:websight",
        "//engine/core/tensor:element_type",
        "//engine/core/tensor:universal_tensor",
    ],
    cc_srcs = glob([
        "bindings/*.cpp",
        "bindings/*.hpp",
    ]),
)

py_library(
    name = "pyalice",
    srcs = [
        "Application.py",
        "CapnpMessages.py",
        "Cask.py",
        "Codelet.py",
        "CodeletBackend.py",
        "CodeletFlowControl.py",
        "CodeletHooks.py",
        "Component.py",
        "Composite.py",
        "Message.py",
        "Node.py",
        "__init__.py",
        "config.py",
        "gui/__init__.py",
        "gui/composite_widget.py",
        "module_explorer.py",
    ],
    data = ["//engine:py_init"],
    visibility = ["//visibility:public"],
    deps = [":bindings"],
)

exports_files(["jupyter_launcher.sh.tpl"])
