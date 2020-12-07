/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "engine/pyalice/bindings/pybind_component.hpp"
#include "engine/pyalice/bindings/pybind_message.hpp"
#include "engine/pyalice/bindings/pybind_node.hpp"
#include "pybind11/pybind11.h"

namespace isaac {
namespace alice {

class Application;

// Owns and provides access to an alice application in Python
// Unless specified otherwise function comments are the same as in engine/alice/application.hpp
class PybindApplication {
 public:
  PybindApplication(const std::string& app_json_str, const std::string& more_jsons,
                    const std::string& asset_path);
  ~PybindApplication();

  // The alice application held by the wrapper
  Application& app() const { return *app_; }

  // Calls ModuleManager::getComponentNames on application backend
  std::set<std::string> getLoadedComponents() const;

  // Calls Application::start
  void start();
  // Calls Application::start_wait_stop
  void start_wait_stop();
  // Calls Application::run
  void run(pybind11::object max_duration, pybind11::object node_name);
  // Calls Application::stop
  void stop();

  // Calls Application::uuid
  pybind11::str uuid() const;

  // Calls Application::createNode
  PybindNode createNode(const std::string& name);
  // Calls Application::connect
  void connect(const std::string& source, const std::string& target);
  // Calls Application::connect
  void connect(const PybindComponent& tx_component, const std::string& tx_tag,
               const PybindComponent& rx_component, const std::string& rx_tag);

  // Calls Application::findNodeByName
  PybindNode findNodeByName(const std::string& name);

  // Get a configration parameter at the supplied node/component/parameter. Return value will be
  // formatted as a JSON string.
  std::string getConfig(const std::string& node, const std::string& component,
                        const std::string& parameter) const;

  // Get all configuration parameters at the supplied node/component. Return value will be
  // formatted as a JSON string. NOTE: Implemented by Davis Robertson
  std::string getAllConfigForComponent(const std::string& node, const std::string& component);

  // Erases the specified parameter value for specified component. Returns true if the value exists,
  // otherwise returns false.
  bool eraseConfig(const std::string& node_name, const std::string& component_name,
                   const std::string& parameter_name);

  // Set a configration parameter at the supplied node/component/parameter. Value format is a JSON
  // string.
  void setConfig(const std::string& node, const std::string& component,
                 const std::string& parameter, const std::string& value);

  // Receive latest message from the supplied node/component/channel. If no new message is
  // available an empty message will be returned.
  PybindMessage receive(const std::string& node_name, const std::string& component_name,
                        const std::string& channel_name);

  // Publishes proto message to specified node/component/channel with buffers and acqtime.
  // Buffer list could be empty.  0 acqtime means now.
  bool publish(const std::string& node_name, const std::string& component_name,
               const std::string& channel_name, const std::string& uuid_str, uint64_t type_id,
               const std::string proto_bytes, std::vector<pybind11::buffer>& buffers,
               int64_t acqtime);

  // Loads graph definition from json file and adds it to graph. Merges the graph (nodes, edges and
  // config) if prefix is empty. Otherwise creates subgraph with given prefix name.
  bool load(const std::string& filename, const std::string& prefix);

  // Loads a specified module (shared lib)
  bool loadModule(const std::string& module_name);

 private:
  std::unique_ptr<Application> app_;
};

// Initializes the python module
void InitPybindApplication(pybind11::module& m);

}  // namespace alice
}  // namespace isaac
