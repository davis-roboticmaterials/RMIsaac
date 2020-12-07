/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "utils.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/message.hpp"
#include "engine/core/buffers/buffer.hpp"
#include "engine/gems/serialization/blob.hpp"
#include "engine/gems/serialization/capnp.hpp"
#include "engine/gems/serialization/header.hpp"
#include "messages/alice.capnp.h"
#include "messages/uuid.hpp"

namespace isaac {
namespace alice {
namespace {
constexpr char kMessageChannelIndexKey[] = "msg_chnl_idx";
}

void FromCask(const cask::Cask& cask, std::vector<CaskMessageChannelEntry>& indices) {
  std::vector<uint8_t> blob;
  cask.keyValueRead(Uuid::FromAsciiString(kMessageChannelIndexKey), blob);

  serialization::Header header;
  const uint8_t* segment_start = Deserialize(blob.data(), blob.data() + blob.size(), header);
  ASSERT(segment_start, "invalid header");
  ASSERT(header.segments.size() > 0, "invalid header");
  std::vector<kj::ArrayPtr<const ::capnp::word>> segments;
  for (uint32_t length : header.segments) {
    const uint8_t* segment_next = segment_start + length;
    segments.push_back(
        kj::ArrayPtr<const ::capnp::word>(reinterpret_cast<const ::capnp::word*>(segment_start),
                                          reinterpret_cast<const ::capnp::word*>(segment_next)));
    segment_start = segment_next;
  }
  ::capnp::SegmentArrayMessageReader reader(
      kj::ArrayPtr<const kj::ArrayPtr<const ::capnp::word>>(segments.data(), segments.size()));
  auto proto = reader.getRoot<MessageChannelIndexProto>();
  auto channels = proto.getChannels();
  indices.clear();
  indices.reserve(channels.size());
  for (size_t i = 0; i < channels.size(); i++) {
    const Uuid component_uuid = FromProto(channels[i].getComponentUuid());
    const std::string tag = channels[i].getTag();
    const Uuid series_uuid = FromProto(channels[i].getSeriesUuid());
    indices.push_back(CaskMessageChannelEntry{component_uuid, tag, series_uuid});
    LOG_DEBUG("Available channel in cask: '%s' (series: %s)", tag.c_str(), series_uuid.c_str());
  }
}

void FromCask(cask::Cask& cask, const Uuid& series_uuid, CaskMessageChannelSeries& channel) {
  cask.seriesOpen(series_uuid);
  size_t count, value_size;
  cask.seriesDimensions(series_uuid, count, value_size);
  if (count == 0) {
    return;
  }
  channel.series_uuid = series_uuid;
  channel.history.clear();
  // Read the series
  const uint32_t flags = serialization::TIP_1_TIMESTAMP | serialization::TIP_2_UUID;
  auto& history = channel.history;
  serialization::Header uuid_ts;
  std::vector<uint8_t> blob;
  for (size_t i = 0; i < count; i++) {
    cask.seriesRead(series_uuid, i, blob);
    auto result = DeserializeWithoutTip(blob, flags, uuid_ts);
    ASSERT(result, "could not parse");
    history.push(*uuid_ts.timestamp, *uuid_ts.uuid);
  }
  cask.seriesClose(series_uuid);  // close since no more required for this channel
}

void WriteToCask(const MessageChannelIndexMap& indices, cask::Cask& cask) {
  ::capnp::MallocMessageBuilder header_builder;
  auto index = header_builder.initRoot<MessageChannelIndexProto>();
  auto channels = index.initChannels(indices.size());
  size_t counter = 0;
  for (auto pair = indices.begin(); pair != indices.end(); pair++) {
    auto channel = channels[counter++];
    ToProto(pair->first.first, channel.initComponentUuid());
    channel.setTag(pair->first.second);
    ToProto(pair->second, channel.initSeriesUuid());
  }
  std::vector<ByteArrayConstView> blobs;
  serialization::CapnpArraysToBlobs(header_builder.getSegmentsForOutput(), blobs);
  // the header
  serialization::Header header;
  header.segments.reserve(blobs.size());
  for (const auto& blob : blobs) {
    header.segments.push_back(blob.size());
  }
  std::vector<uint8_t> buffer;
  Serialize(header, buffer);
  // write all segments (including the header)
  blobs.insert(blobs.begin(), ByteArrayConstView{buffer.data(), buffer.size()});
  cask.keyValueWrite(Uuid::FromAsciiString(kMessageChannelIndexKey), blobs);
}

void WriteToCask(const CaskMessageEntry& log_entry, MessageChannelIndexMap& index_map,
                 cask::Cask& cask) {
  // Write the message
  WriteMessageToCask(log_entry.message, cask);

  // Add message to channel series
  const std::pair<Uuid, std::string> channel_key{log_entry.component_uuid, log_entry.tag};
  auto it = index_map.find(channel_key);
  if (it == index_map.end()) {
    Uuid uuid = Uuid::Generate();
    it = index_map.insert({channel_key, uuid}).first;
    cask.seriesOpen(uuid, 24);
    // Writes indices to cask
    WriteToCask(index_map, cask);
  }
  serialization::Header uuid_ts;
  uuid_ts.timestamp = log_entry.message->pubtime;
  uuid_ts.uuid = log_entry.message->uuid;
  std::array<uint8_t, 24> keybytes;
  const uint32_t flags = serialization::TIP_1_TIMESTAMP | serialization::TIP_2_UUID;
  SerializeWithoutTip(uuid_ts, flags, keybytes.data(), keybytes.data() + keybytes.size());
  cask.seriesAppend(it->second, ByteArrayConstView{keybytes.data(), keybytes.size()});
}

void WriteMessageToCask(ConstMessageBasePtr message, cask::Cask& cask) {
  const ProtoMessageBase* proto_message = dynamic_cast<const ProtoMessageBase*>(message.get());
  ASSERT(proto_message != nullptr, "Only proto messages are supported for logging");

  // Prepares message proto
  std::vector<ByteArrayConstView> segments;
  serialization::CapnpArraysToBlobs(proto_message->segments(), segments);

  // Prepares buffers
  std::vector<ByteArrayConstView> buffer_blobs;
  for (const auto& buffer : message->buffers) {
    buffer_blobs.push_back(
        ByteArrayConstView{buffer.host_buffer().begin(), buffer.host_buffer().size()});
  }

  // Prepares message header
  serialization::Header header;
  header.proto_id = proto_message->proto_id();
  header.timestamp = message->pubtime;
  header.acqtime = message->acqtime;
  serialization::BlobsToLengths32u(segments, header.segments);
  serialization::BlobsToLengths32u(buffer_blobs, header.buffers);

  // Prepares message header length
  size_t header_length;
  if (!Size(header, true, &header_length, nullptr)) {
    PANIC("could not compute header size");
  }

  // Prepares total segment length
  const size_t segment_length = serialization::AccumulateLength(segments);
  const size_t buffer_length = serialization::AccumulateLength(buffer_blobs);

  // Writes data
  cask.keyValueWrite(message->uuid, header_length + segment_length + buffer_length,
                     [&](byte* begin, byte* end) {
                       begin = Serialize(header, begin, end);
                       ASSERT(begin, "error serializing header");
                       begin = serialization::CopyAll(segments, begin, end);
                       begin = serialization::CopyAll(buffer_blobs, begin, end);
                     });
}

MessageBasePtr ReadMessageFromCask(const Uuid& message_uuid, cask::Cask& cask) {
  serialization::Header header;
  std::vector<byte> proto_buffer;
  std::vector<SharedBuffer> buffers;
  cask.keyValueRead(message_uuid, [&](const byte* begin, const byte* end) {
    begin = Deserialize(begin, end, header);
    // compute proto buffer length
    const size_t total_proto_length =
        std::accumulate(header.segments.begin(), header.segments.end(), size_t{0});
    // copy out proto buffer
    proto_buffer.resize(total_proto_length);
    std::copy(begin, begin + total_proto_length, proto_buffer.begin());
    begin += total_proto_length;
    // de-serialize buffers
    buffers.reserve(header.buffers.size());
    for (const size_t length : header.buffers) {
      CpuBuffer output(length);
      std::copy(begin, begin + length, output.begin());
      buffers.emplace_back(std::move(output));
      begin += length;
    }
  });
  ASSERT(header.timestamp, "Timestamp missing in message header");
  ASSERT(header.acqtime, "Acqtime timestamp missing in message header");
  ASSERT(!header.segments.empty(), "Message must have at least one segment");
  std::vector<size_t> segment_offsets(header.segments.size());
  for (size_t i = 0; i < segment_offsets.size(); i++) {
    segment_offsets[i] = static_cast<size_t>(header.segments[i]);
  }

  auto message = std::make_shared<BufferedProtoMessage>(std::move(proto_buffer), 0,
                                                        std::move(segment_offsets));
  message->buffers = std::move(buffers);
  message->uuid = message_uuid;
  message->pubtime = *header.timestamp;
  message->acqtime = *header.acqtime;
  if (header.proto_id) {
    message->setProtoId(*header.proto_id);
  }
  return message;
}

MessageBasePtr CreateProtoMessage(const Uuid uuid, const uint64_t type_id,
                                  const std::string& proto_bytes, std::vector<CpuBuffer>& buffers) {
  if (proto_bytes.empty() || type_id == 0) {
    // Fails on invalid proto data
    return nullptr;
  }

  // Creates and populates proto message payload
  std::vector<uint8_t> proto_buffer;
  std::vector<size_t> segment_lengths;
  isaac::serialization::StringToCapnpBuffer(proto_bytes, segment_lengths, proto_buffer);
  MessageBasePtr message_base =
      std::make_shared<BufferedProtoMessage>(proto_buffer, 0, segment_lengths);

  // Populates proto id
  message_base->type = type_id;
  // Generates UUID
  message_base->uuid = uuid;

  // Populates buffers
  message_base->buffers.reserve(buffers.size());
  for (auto& buffer : buffers) {
    message_base->buffers.emplace_back(std::move(buffer));
  }

  return message_base;
}

}  // namespace alice
}  // namespace isaac
