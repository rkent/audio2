#ifndef AUDIO2_STREAM_STREAM_POLICY_HPP
#define AUDIO2_STREAM_STREAM_POLICY_HPP

// Decisions for StreamHeader.handle: how a new stream is treated relative to existing streams.

#include <cstdint>
#include <vector>

#include "audio2_stream/AudioStream.hpp"
#include "audio2_stream_msgs/msg/stream_header.hpp"

// What to do with a new stream
enum class Admission
{
  StartNow,  // Start playing immediately
  Enqueue,   // Hold until all existing streams have finished
  Drop       // Do not play
};

struct HandleDecision
{
  Admission admission;
  bool cancel_others;  // Stop all existing (active and pending) streams first
  bool unknown_handle;  // The handle value was not recognized and was treated as MIX
};

// Decide how to admit a new stream with the given handle. others_exist is true when any
// stream is active or waiting in the queue.
inline HandleDecision decide_handle(uint8_t handle, bool others_exist)
{
  using SH = audio2_stream_msgs::msg::StreamHeader;
  switch (handle) {
    case SH::HANDLE_MIX:
    case SH::HANDLE_MUTE:  // Muting of the other streams is done by update_mutes()
      return {Admission::StartNow, false, false};
    case SH::HANDLE_QUEUE:
      return {others_exist ? Admission::Enqueue : Admission::StartNow, false, false};
    case SH::HANDLE_CANCEL:
      return {Admission::StartNow, others_exist, false};
    case SH::HANDLE_IGNORE:
      return {others_exist ? Admission::Drop : Admission::StartNow, false, false};
    default:
      return {Admission::StartNow, false, true};
  }
}

// Set muted_ on each stream: a stream is muted when a newer stream with HANDLE_MUTE is still
// running. Streams are given oldest first.
inline void update_mutes(const std::vector<AudioStream *> & streams_in_start_order)
{
  bool newer_mute_running = false;
  for (auto it = streams_in_start_order.rbegin(); it != streams_in_start_order.rend(); ++it) {
    AudioStream * stream = *it;
    stream->muted_.store(newer_mute_running);
    if (stream->handle_ == audio2_stream_msgs::msg::StreamHeader::HANDLE_MUTE &&
      !stream->shutdown_flag_.load() && !stream->shutdown_complete_.load())
    {
      newer_mute_running = true;
    }
  }
}

#endif  // AUDIO2_STREAM_STREAM_POLICY_HPP
