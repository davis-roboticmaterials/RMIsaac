/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "pybind_cask.hpp"

#include "capnp/message.h"
#include "capnp/serialize.h"
#include "engine/alice/component_impl.hpp"
#include "engine/alice/utils/utils.hpp"
#include "engine/core/byte.hpp"
#include "engine/gems/cask/cask.hpp"
#include "engine/gems/serialization/capnp.hpp"
#include "engine/gems/serialization/header.hpp"
#include "messages/alice.capnp.h"
#include "messages/uuid.hpp"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

namespace isaac {
namespace alice {
namespace {
MessageBasePtr GenerateProtoMessageFromPybindData(const std::string& uuid_str, uint64_t type_id,
                                                  const std::string proto_bytes,
                                                  std::vector<pybind11::buffer>& buffers) {
  // Generate uuid is not provided
  const Uuid uuid = uuid_str.empty() ? Uuid::Generate() : Uuid::FromString(uuid_str);

  // Populates buffers
  std::vector<CpuBuffer> cpu_buffers;
  cpu_buffers.reserve(buffers.size());
  for (auto& buffer : buffers) {
    pybind11::buffer_info info = buffer.request();
    const size_t buffer_size = info.itemsize * info.size;
    CpuBuffer cpuBuffer(buffer_size);
    std::copy(reinterpret_cast<byte*>(info.ptr), reinterpret_cast<byte*>(info.ptr) + buffer_size,
              cpuBuffer.begin());
    cpu_buffers.emplace_back(std::move(cpuBuffer));
  }

  return CreateProtoMessage(uuid, type_id, proto_bytes, cpu_buffers);
}

}  // namespace

PybindCask::PybindCask(const std::string& root, bool writable) : writable_(writable) {
  cask_ = std::make_unique<cask::Cask>(root,
                                       writable ? cask::Cask::Mode::Write : cask::Cask::Mode::Read);
}

PybindCask::~PybindCask() {}

std::vector<std::pair<std::string, std::string>> PybindCask::getChannels() const {
  std::vector<std::pair<std::string, std::string>> result;
  if (writable_) {
    // FIXME Handles reading in write mode
    return result;
  }
  // Reads all channels
  std::vector<CaskMessageChannelEntry> channels;
  FromCask(*cask_, channels);

  result.reserve(channels.size());
  for (const auto& channel : channels) {
    const std::string tag = channel.tag;
    const Uuid series_uuid = channel.series_uuid;
    result.push_back({tag, series_uuid.str()});
    LOG_DEBUG("Available channel in cask: '%s' (series: %s)", tag.c_str(), series_uuid.c_str());
  }
  return result;
}

std::vector<std::pair<int64_t, std::string>> PybindCask::getChannelMessages(
    const std::string& channel_uuid_str) const {
  std::vector<std::pair<int64_t, std::string>> result;
  if (writable_) {
    // FIXME Handles reading in write mode
    return result;
  }
  const Uuid series_uuid = Uuid::FromString(channel_uuid_str);

  CaskMessageChannelSeries series;
  // Read the series
  FromCask(*cask_, series_uuid, series);

  const size_t count = series.history.size();
  result.reserve(count);

  for (size_t i = 0; i < count; i++) {
    const auto& record = series.history.at(i);
    result.push_back({record.stamp, record.state.str()});
  }

  return result;
}

PybindMessage PybindCask::readMessage(const std::string& uuid_str) const {
  return PybindMessage(ReadMessageFromCask(Uuid::FromString(uuid_str), *cask_));
}

bool PybindCask::writeMessage(PybindMessage& pybind_message) {
  if (!writable_) {
    return false;
  }
  if (pybind_message.isNull()) {
    return false;
  }

  WriteMessageToCask(pybind_message.getMessage(), *cask_);

  return true;
}

bool PybindCask::writeMessage(const std::string& uuid_str, uint64_t type_id,
                              const std::string proto_bytes, std::vector<pybind11::buffer>& buffers,
                              int64_t acqtime, int64_t pubtime) {
  if (!writable_) {
    return false;
  }

  MessageBasePtr message_base =
      GenerateProtoMessageFromPybindData(uuid_str, type_id, proto_bytes, buffers);
  if (message_base == nullptr) {
    return false;
  }

  // Assumes that both pubtime and acqtime are well populated
  message_base->pubtime = pubtime;
  message_base->acqtime = acqtime;

  WriteMessageToCask(message_base, *cask_);

  return true;
}

bool PybindCask::writeChannelMessage(PybindComponent& pybind_component, const std::string& tag,
                                     PybindMessage& pybind_message) {
  if (!writable_) {
    return false;
  }
  WriteToCask(
      CaskMessageEntry{pybind_component.component()->uuid(), tag, pybind_message.getMessage()},
      component_key_to_uuid_, *cask_);
  return true;
}
bool PybindCask::writeChannelMessage(PybindComponent& pybind_component, const std::string& tag,
                                     const std::string& uuid_str, const uint64_t type_id,
                                     const std::string proto_bytes,
                                     std::vector<pybind11::buffer>& buffers, int64_t acqtime,
                                     int64_t pubtime) {
  if (!writable_) {
    return false;
  }

  MessageBasePtr message_base =
      GenerateProtoMessageFromPybindData(uuid_str, type_id, proto_bytes, buffers);
  if (message_base == nullptr) {
    return false;
  }

  // Assumes that both pubtime and acqtime are well populated
  message_base->pubtime = pubtime;
  message_base->acqtime = acqtime;

  WriteToCask(CaskMessageEntry{pybind_component.component()->uuid(), tag, message_base},
              component_key_to_uuid_, *cask_);

  return true;
}

void InitPybindCask(pybind11::module& m) {
  pybind11::class_<PybindCask>(m, "PybindCask")
      .def(pybind11::init<const std::string&, const bool>())
      .def("get_channels", &PybindCask::getChannels)
      .def("get_channel_messages", &PybindCask::getChannelMessages)
      .def("read_message", &PybindCask::readMessage)
      .def("write_channel_message",
           pybind11::overload_cast<PybindComponent&, const std::string&, PybindMessage&>(
               &PybindCask::writeChannelMessage))
      .def(
          "write_channel_message",
          pybind11::overload_cast<PybindComponent&, const std::string&, const std::string&,
                                  const uint64_t, const std::string, std::vector<pybind11::buffer>&,
                                  int64_t, int64_t>(&PybindCask::writeChannelMessage))
      .def("write_message", pybind11::overload_cast<PybindMessage&>(&PybindCask::writeMessage))
      .def("write_message",
           pybind11::overload_cast<const std::string&, const uint64_t, const std::string,
                                   std::vector<pybind11::buffer>&, int64_t, int64_t>(
               &PybindCask::writeMessage));
}

}  // namespace alice
}  // namespace isaac
