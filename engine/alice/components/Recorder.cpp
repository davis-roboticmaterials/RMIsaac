/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Recorder.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/components/MessageLedger.hpp"
#include "engine/alice/utils/utils.hpp"
#include "engine/core/assert.hpp"
#include "engine/gems/cask/cask.hpp"
#include "engine/gems/serialization/blob.hpp"
#include "engine/gems/serialization/header.hpp"
#include "messages/alice.capnp.h"
#include "messages/uuid.hpp"

namespace isaac {
namespace alice {

namespace {

// Maximum time period for tick to block waiting for new messages to record
constexpr std::chrono::milliseconds kMaxWaitDurationMs(100);
// Waiting time when recording is not enable yet.
constexpr int64_t kMaxWaitDurationNs(100'000'000);

// Number of bytes in a megabyte
constexpr size_t kMegaByte = 1ull << 20ull;  // 1 MB

// If the stop estimates to take longer than this time duration it will write a warning.
constexpr double kStopDurationWarnLimit = 1.0;

}  // namespace

Recorder::Recorder() {}
Recorder::~Recorder() {}

void Recorder::startRecording(const std::string& base_directory, const std::string& log_tag) {
  std::unique_lock<std::mutex> lock(items_mutex_);
  ASSERT(!recording_enabled_, "Recording has already started");

  // Open the cask
  std::string root = base_directory + "/" + node()->app()->uuid().str();
  if (!log_tag.empty()) {
    root = root + "/" + log_tag;
  }

  {
    std::unique_lock<std::mutex> cask_lock(cask_mutex_);
    cask_ = std::make_unique<cask::Cask>(root, cask::Cask::Mode::Write);
  }

  // Unlock the queue to accept messages for logging
  items_queue_locked_ = false;
  recording_enabled_ = true;
  // Initialize the max buffer size
  items_max_memory_size_ = static_cast<size_t>(get_max_buffer_size_mb()) * kMegaByte;
}

void Recorder::stopRecording() {
  std::unique_lock<std::mutex> lock(items_mutex_);
  ASSERT(recording_enabled_, "Recording was not started");
  // Lock the queue to not accept any new items
  recording_enabled_ = false;
  items_queue_locked_ = true;

  // Estimate how long it will take to write out remaining items
  const double items_queue_memory_size_mb =
      static_cast<double>(items_queue_memory_size_) / static_cast<double>(kMegaByte);
  const double estimated_write_time = items_queue_memory_size_mb / write_speed_mb_per_s_;
  if (estimated_write_time > kStopDurationWarnLimit) {
    LOG_WARNING(
        "Recorder still needs to write about %.1f MB which will take about %.1f s. "
        "Please be patient...",
        items_queue_memory_size_mb, estimated_write_time);
  }

  {
    std::unique_lock<std::mutex> cask_lock(cask_mutex_);
    // Write out all remaining items
    for (const auto& item : items_queue_) {
      record(item);
    }
    // Close the cask
    cask_.reset();
    component_key_to_uuid_.clear();
  }

  // Clear the queue
  items_queue_.clear();
  items_queue_memory_size_ = 0;
}

void Recorder::initialize() {
  items_max_memory_size_ = 0;
  items_queue_memory_size_ = 0;
  items_in_process_memory_size_ = 0;

  // Start with a locked queue as we can only accept messages after start.
  items_queue_locked_ = true;
  recording_enabled_ = false;

  // Messages received by Recorder will be written to the log
  MessageLedger* ledger = node()->getComponent<MessageLedger>();
  ledger->addOnConnectAsRxCallback(
      [ledger, this](const MessageLedger::Endpoint& tx, const MessageLedger::Endpoint& rx) {
        if (rx.component != this) {
          return;
        }
        // notify about replayed messages
        ledger->addOnMessageCallback(rx, tx.component, [this, tx, rx](ConstMessageBasePtr message) {
          if (!recording_enabled_) {
            return;
          }
          onRequestRecord(CaskMessageEntry{tx.component->uuid(), rx.tag, message});
        });
      });
}

void Recorder::start() {
  if (get_start_recording_automatically()) {
    startRecording(get_base_directory(), get_tag());
  }
  tickBlocking();
}

void Recorder::tick() {
  if (!recording_enabled_) {
    // Sleep a bit to release the cpu.
    Sleep(kMaxWaitDurationNs);
    return;
  }
  std::vector<CaskMessageEntry> current_items_in_process;
  {
    std::unique_lock<std::mutex> lock(items_mutex_);

    // Wait for new data
    items_queue_cv_.wait_for(lock, kMaxWaitDurationMs);
    // Check that the recording is still enabled.
    if (!recording_enabled_) {
      return;
    }

    // Periodically update max buffer size
    items_max_memory_size_ = static_cast<size_t>(get_max_buffer_size_mb()) * kMegaByte;

    // Get all queued items
    current_items_in_process = std::move(items_queue_);
    items_queue_ = {};
    items_queue_.reserve(current_items_in_process.size());
    items_in_process_memory_size_ = items_queue_memory_size_;
    items_queue_memory_size_ = 0;
  }
  // Release the items_queue_ while writting the messages to allow more queuing.

  // Show how much data is currently being written out
  const double memory_size_mb =
      static_cast<double>(items_in_process_memory_size_) / static_cast<double>(kMegaByte);
  show("in_process_size_mb", memory_size_mb);

  {
    std::unique_lock<std::mutex> cask_lock(cask_mutex_);
    if (recording_enabled_ && cask_) {
      // Write data to log and show the write throughput
      const int64_t time_begin = node()->clock()->timestamp();
      for (const auto& item : current_items_in_process) {
        record(item);
      }
      const int64_t time_end = node()->clock()->timestamp();
      write_speed_mb_per_s_ = memory_size_mb / ToSeconds(time_end - time_begin);
    }
  }
  show("write_speed_mb_per_s", write_speed_mb_per_s_);

  { // Unlock the queue as we can process new data again.
    std::lock_guard<std::mutex> lock(items_mutex_);
    items_in_process_memory_size_ = 0;
    items_queue_locked_ = !recording_enabled_;
  }
}

void Recorder::stop() {
  if (recording_enabled_) {
    stopRecording();
  }
}

size_t Recorder::numChannels() const {
  MessageLedger* ledger = node()->getComponent<MessageLedger>();
  return ledger ? ledger->numSourceChannels() : 0;
}

void Recorder::onRequestRecord(CaskMessageEntry&& item) {
  {
    std::lock_guard<std::mutex> lock(this->items_mutex_);
    // Do not accept requests when the queue is locked
    if (items_queue_locked_) {
      return;
    }

    // Add the new item to the queue
    items_queue_.emplace_back(item);
    items_queue_memory_size_ += item.message->size();

    // If too many items are queued lock the queue
    if (items_queue_memory_size_ + items_in_process_memory_size_ > items_max_memory_size_) {
      items_queue_locked_ = true;
      LOG_WARNING(
          "Could not keep up with writing messages. Will now drop all messages "
          "until all queued messages are written out.");
    }
  }

  // Notify the tick that new work has arrived
  this->items_queue_cv_.notify_one();
}

void Recorder::record(const CaskMessageEntry& item) {
  ASSERT(cask_, "Cask not opened");

  WriteToCask(item, component_key_to_uuid_, *cask_);
}

}  // namespace alice
}  // namespace isaac
