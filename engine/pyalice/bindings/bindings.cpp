/*
Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/pyalice/bindings/pybind_application.hpp"
#include "engine/pyalice/bindings/pybind_atlas.hpp"
#include "engine/pyalice/bindings/pybind_cask.hpp"
#include "engine/pyalice/bindings/pybind_clock.hpp"
#include "engine/pyalice/bindings/pybind_component.hpp"
#include "engine/pyalice/bindings/pybind_core.hpp"
#include "engine/pyalice/bindings/pybind_message.hpp"
#include "engine/pyalice/bindings/pybind_node.hpp"
#include "engine/pyalice/bindings/pybind_py_codelet.hpp"
#include "pybind11/pybind11.h"

PYBIND11_MODULE(bindings, m) {
  m.doc() = R"pbdoc(
        Isaac Alice Python Bridge
        -----------------------

        .. currentmodule:: pyalice

    )pbdoc";

  isaac::alice::InitPybindApplication(m);
  isaac::alice::InitPybindAtlas(m);
  isaac::alice::InitPybindCask(m);
  isaac::alice::InitPybindClock(m);
  isaac::alice::InitPybindComponent(m);
  isaac::alice::InitPybindCore(m);
  isaac::alice::InitPybindMessage(m);
  isaac::alice::InitPybindNode(m);
  isaac::alice::InitPybindPyCodelet(m);
}
