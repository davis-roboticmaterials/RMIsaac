/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "engine/alice/message.hpp"
#include "engine/gems/algorithm/timeseries.hpp"
#include "engine/gems/cask/cask.hpp"
#include "engine/gems/uuid/uuid.hpp"

namespace isaac {
namespace alice {

// Entry to be written to Cask
struct CaskMessageEntry {
  // The UUID of the component which published the message
  Uuid component_uuid;
  // The channel tag under which the message was published
  std::string tag;
  // The message which was published
  ConstMessageBasePtr message;
};

// Data structure for series of messages in a channel
struct CaskMessageChannelSeries {
  // UUID for the series used in the log file
  Uuid series_uuid;
  // Message history of all messages from the channel
  Timeseries<Uuid, int64_t> history;
};

// Data structure for reading list of channels in cask
struct CaskMessageChannelEntry {
  // UUID of the publishing component
  Uuid component_uuid;
  // Tag of the message channel
  std::string tag;
  // UUID for timeseries data
  Uuid series_uuid;
};

// Reads and parses channel indices from Cask
void FromCask(const cask::Cask& cask, std::vector<CaskMessageChannelEntry>& indices);
// Reads and parses channel message series from Cask
void FromCask(cask::Cask& cask, const Uuid& series_uuid, CaskMessageChannelSeries& channel);

// Hash function for channel index map
struct MessageChannelHash {
  std::size_t operator()(const std::pair<Uuid, std::string>& value) const {
    return std::hash<std::string>{}.operator()(value.second) ^
           std::hash<isaac::Uuid>{}.operator()(value.first);
  }
};

// Data structure for tracking channels while writing to cask, mapping {source_uuid, channel_tag} to
// series_uuid
using MessageChannelIndexMap =
    std::unordered_map<std::pair<Uuid, std::string>, Uuid, MessageChannelHash>;

// Writes Message channel index map to cask
void WriteToCask(const MessageChannelIndexMap& index, cask::Cask& cask);
// Writes log entry to cask, updates index map accordingly and writes index map to cask
void WriteToCask(const CaskMessageEntry& log_entry, MessageChannelIndexMap& index_map,
                 cask::Cask& cask);

// Writes message to cask using message uuid as key
void WriteMessageToCask(ConstMessageBasePtr message, cask::Cask& cask);

// Reads message from cask with uuid
MessageBasePtr ReadMessageFromCask(const Uuid& message_uuid, cask::Cask& cask);

// Creates Isaac Message from data and buffers
MessageBasePtr CreateProtoMessage(const Uuid uuid_str, const uint64_t type_id,
                                  const std::string& proto_bytes, std::vector<CpuBuffer>& buffers);

}  // namespace alice
}  // namespace isaac
