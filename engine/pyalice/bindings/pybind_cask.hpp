/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "engine/alice/utils/utils.hpp"
#include "engine/gems/cask/cask.hpp"
#include "engine/pyalice/bindings/pybind_component.hpp"
#include "engine/pyalice/bindings/pybind_message.hpp"
#include "pybind11/pybind11.h"

namespace isaac {
namespace alice {

// Pybind API to provide access to a cask log file in Python
class PybindCask {
 public:
  // Loads the cask at the given location
  PybindCask(const std::string& root, const bool writable = false);
  ~PybindCask();

  // Gets a list of all channels in this log file. Each entry contains channel name and channel
  // UUID as string.
  std::vector<std::pair<std::string, std::string>> getChannels() const;
  // Gets a list of all messages in the channel with given UUID. Entries contain message timestamp
  // and message UUID as string.
  std::vector<std::pair<int64_t, std::string>> getChannelMessages(
      const std::string& channel_uuid_str) const;

  // Gets the message for the given UUID string
  PybindMessage readMessage(const std::string& uuid_str) const;
  // Writes proto data and buffer data as message into the cask. Fails if the cask is not opened in
  // write mode.
  bool writeMessage(PybindMessage& pybind_message);
  // Creates and writes proto message into cask from proto data and buffers
  bool writeMessage(const std::string& uuid_str, const uint64_t type_id,
                    const std::string proto_bytes, std::vector<pybind11::buffer>& buffers,
                    int64_t acqtime, int64_t pubtime);

  // Writes a message into cask and update channel index
  bool writeChannelMessage(PybindComponent& pybind_component, const std::string& tag,
                           PybindMessage& pybind_message);
  // Creates a message from data and writes it into cask, and update channel index
  bool writeChannelMessage(PybindComponent& pybind_component, const std::string& tag,
                           const std::string& uuid_str, const uint64_t type_id,
                           const std::string proto_bytes, std::vector<pybind11::buffer>& buffers,
                           int64_t acqtime, int64_t pubtime);

 private:
  std::unique_ptr<cask::Cask> cask_;
  bool writable_;
  // Mapping from {component_uuid, tag} to series uuid for writing message to channel
  MessageChannelIndexMap component_key_to_uuid_;
};

// Initializes the python module
void InitPybindCask(pybind11::module& module);

}  // namespace alice
}  // namespace isaac
