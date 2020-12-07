/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "pybind_node.hpp"

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/node_backend.hpp"
#include "engine/alice/node.hpp"

namespace isaac {
namespace alice {

PybindNode::PybindNode() : node_(nullptr) {}

PybindNode::PybindNode(alice::Node* node) : node_(node) {}

PybindNode::~PybindNode() {}

const std::string& PybindNode::name() const {
  return node_->name();
}

alice::Status PybindNode::getStatus() const {
  return (node_ ? node_->getStatus() : Status::INVALID);
}

PybindComponent PybindNode::addComponent(const std::string& type, const std::string& name) {
  return PybindComponent(node_->addComponent(type, name));
}

pybind11::object PybindNode::getComponent(const std::string& name) {
  auto* component_ptr = node_->findComponentByName(name);
  if (component_ptr == nullptr) {
    return pybind11::object(pybind11::cast(nullptr));
  }
  return pybind11::cast(PybindComponent(component_ptr));
}

void PybindNode::start() {
  LOG_ERROR("starting %s", node_->name().c_str());
  node_->app()->backend()->node_backend()->startNode(node_);
}
void PybindNode::stop() {
  node_->app()->backend()->node_backend()->stopNode(node_);
}

void InitPybindNode(pybind11::module& m) {
  pybind11::enum_<Status>(m, "Status")
      .value("Success", Status::SUCCESS)
      .value("Failure", Status::FAILURE)
      .value("Running", Status::RUNNING)
      .value("Invalid", Status::INVALID)
      .export_values();
  pybind11::class_<PybindNode>(m, "PybindNode")
      .def(pybind11::init<>())
      .def("add_component", &isaac::alice::PybindNode::addComponent)
      .def("get_component", &isaac::alice::PybindNode::getComponent)
      .def("get_status", &isaac::alice::PybindNode::getStatus)
      .def("is_null", &isaac::alice::PybindNode::isNull)
      .def("name", &isaac::alice::PybindNode::name)
      .def("start", &isaac::alice::PybindNode::start)
      .def("stop", &isaac::alice::PybindNode::stop);
}

}  // namespace alice
}  // namespace isaac
