/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "engine/alice/message.hpp"
#include "engine/alice/utils/utils.hpp"
#include "engine/gems/uuid/uuid.hpp"

namespace isaac {
namespace cask {
class Cask;
}
}  // namespace isaac

namespace isaac {
namespace alice {

// Records data into a log file. This component can for example be used to write incoming messages
// to a log file. The messages could be replayed later using the Replay component.
//
// In order to record a message channel setup an edge from the publishing component to the Recorder
// component. The source channel is the name of the channel under which the publishing component
// publishes the data. The target channel name on the Recorder component can be chosen freely.
// When data is replayed it will be published by the Replay component under that same channel name.
//
// Warning: Please note that the log container format is not yet final and that breaking changes
// might occur in in the future.
//
// The root directory used to log data is `base_directory/exec_uuid/tag/...` where both
// `base_directory` and `tag` are configuration parameters. `exec_uuid` is a UUID which changed for
// every execution of an app and is unique over all possible executions. If `tag` is the empty
// string the root log directory is just `base_directory/exec_uuid/...`.
//
// Multiple recorders can write to the same root log directory. In this case they share the same
// key-value database. However only one recorder is allowed per log series. This means if the same
// component/key channel is logged by two different recorders they can not write to the same log
// directory.
class Recorder : public Codelet {
 public:
  Recorder();
  ~Recorder();

  void initialize() override;
  void start() override;
  void tick() override;
  void stop() override;

  // Gets the cask log container
  cask::Cask* cask() const { return cask_.get(); }

  // Return the number of channels connected to this component
  size_t numChannels() const;

  // Returns whether or not this Recorder is currently recording incoming messages.
  bool recordingEnabled() const { return recording_enabled_; }
  // Opens the cask with the given base_directory and tag, and turns on the recording.
  void startRecording(const std::string& base_directory, const std::string& log_tag);
  // Stops the recording and close the cask.
  void stopRecording();

  // The base directory used as part of the log directory (see class comment)
  ISAAC_PARAM(std::string, base_directory, "/tmp/isaac");
  // A tag used as part of the log directory (see class comment)
  ISAAC_PARAM(std::string, tag, "");
  // If messages can not be written out fast enough they are kept in memory. If more than a certain
  // amount of memory is used no new messages are accepted until all messages are written out to
  // the disk.
  ISAAC_PARAM(int, max_buffer_size_mb, 4096);
  // If true, the recording will automatically start when this Codelet starts.
  ISAAC_PARAM(bool, start_recording_automatically, true);

 private:
  // Called whenever a new message arrives in the codelet
  void onRequestRecord(CaskMessageEntry&& item);

  // Writes a message to the log and updates the channel index
  void record(const CaskMessageEntry& item);

  // Writes an index with all channels being logged to the log container
  void writeChannelIndex();

  // This mutex protects all access to the cask_, the items_mutex_ can't be locked after this one to
  // avoid dead lock.
  std::mutex cask_mutex_;
  // This mutex protects all member variables with prefix items_ from concurrent access.
  std::mutex items_mutex_;
  // A conditional variable to wake up tick when new messages arrive
  std::condition_variable items_queue_cv_;
  // A queue of messages which need to be written to the log file
  std::vector<CaskMessageEntry> items_queue_;
  // The total size of all messages in the `work_items_` queue
  size_t items_queue_memory_size_;
  // The total size of all messages which are currently being processed by the tick function
  size_t items_in_process_memory_size_;
  // The maximum total allowed size of items kept in the item queue.
  size_t items_max_memory_size_;
  // If this flag is enabled new messages are rejected until all items in the current queue are
  // written to the log.
  bool items_queue_locked_;

  // The latest write speed in MB/s
  double write_speed_mb_per_s_;

  std::atomic<bool> recording_enabled_;

  // The casks to which messages are written
  std::unique_ptr<cask::Cask> cask_;
  // A mapping from component channels to series uuids used to store the messages
  MessageChannelIndexMap component_key_to_uuid_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::Recorder)
