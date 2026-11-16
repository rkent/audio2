#include <csignal>
#include <cstdio>
#include <cstring>
#include <sndfile.h>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <fstream>
#include <memory>
#include <functional>
#include <array>
#include <deque>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "audio2_stream/buffer_file.hpp"
#include "audio2_stream/AudioStream.hpp"
#include "audio2_stream/stream_policy.hpp"
#include "boost/lockfree/spsc_queue.hpp"

#include "audio2_stream/IAudioDeviceProxy.hpp"
#include "audio2_stream_msgs/msg/play_file.hpp"
#include "audio2_stream_msgs/msg/audio_chunk.hpp"
#include "audio2_stream_msgs/msg/tts_provider.hpp"
#include "audio2_stream_msgs/msg/tts_request.hpp"

static auto rcl_logger = rclcpp::get_logger("audio2_play");

using AudioChunk = audio2_stream_msgs::msg::AudioChunk;
using StreamHeader = audio2_stream_msgs::msg::StreamHeader;
// Key for maps indexed by stream UUID
using UuidKey = std::array<uint8_t, 16>;

// Chunks of a queued or draining chunk stream are fed to its audio queue only while it has at
// least this many free entries, so they are never dropped for lack of space.
static constexpr std::size_t BACKLOG_QUEUE_MARGIN = 10;
// Chunks of an ignored or cancelled stream are discarded until its end-of-stream chunk, or until
// no chunk has arrived for this long.
static constexpr double DROPPED_UUID_TIMEOUT_S = 10.0;

class Audio2PlayNode : public rclcpp::Node
{
public:
  Audio2PlayNode()
  : Node("audio2_play_node")
  {
        // Parameters
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.description = "Audio device name for playback";
    param_desc.additional_constraints =
      "An ALSA device name on Linux; on Windows \"default\", or part of a device's name or its ID.";
    param_desc.read_only = false;
    this->declare_parameter<std::string>("device_name", DEFAULT_DEVICE_NAME, param_desc);

    param_desc.description = "Audio sample format for playback";
    param_desc.additional_constraints =
      "A SampleFormat integer value (same numbering as ALSA snd_pcm_format_t; 14 = float).";
    param_desc.read_only = false;
    this->declare_parameter<int>("sample_format", static_cast<int>(DEFAULT_SAMPLE_FORMAT),
      param_desc);

    param_desc.description = "Number of frames per audio chunk in the stream";
    param_desc.additional_constraints = "Must be a positive integer.";
    param_desc.read_only = false;
    this->declare_parameter<int>("stream_queue_frames", STREAM_QUEUE_FRAMES, param_desc);

        // Subscriber for TtsRequest messages
    tts_request_subscriber_ = this->create_subscription<audio2_stream_msgs::msg::TtsRequest>(
            "tts_request", 10,
            std::bind(&Audio2PlayNode::tts_request_callback, this, std::placeholders::_1));

        // Subscriber for local PlayFile messages
    play_file_local_subscriber_ = this->create_subscription<audio2_stream_msgs::msg::PlayFile>(
            "play_file_local", 10,
            std::bind(&Audio2PlayNode::play_file_local_callback, this, std::placeholders::_1));

        // Streams and subscriber for audio chunks
    message_source_ = std::make_unique<MessageSource>(
            "audio_stream_chunks"
    );

        // TODO: Figure out best QOS settings for audio chunk subscription
    chunk_subscriber_ = this->create_subscription<audio2_stream_msgs::msg::AudioChunk>(
            "audio_stream_chunks", AUDIO_CHUNK_QOS,
            std::bind(&Audio2PlayNode::audio_chunk_callback, this, std::placeholders::_1));

        // Register shutdown callback
    rclcpp::on_shutdown(
            std::bind(&Audio2PlayNode::stop_streams_callback, this));

        // Timer to check and clean up finished streams
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&Audio2PlayNode::check_streams_callback, this));

    RCLCPP_INFO(rcl_logger, "Audio2PlayNode initialized.");
  }

  ~Audio2PlayNode()
  {
    RCLCPP_INFO(rcl_logger, "Audio2PlayNode shutting down.");
  }

  void tts_request_callback(const audio2_stream_msgs::msg::TtsRequest::SharedPtr msg)
  {
    RCLCPP_INFO(rcl_logger, "Received TtsRequest message, text: %s",
            msg->text.c_str());
    std::unique_ptr<TtsSource> tts_source = std::make_unique<TtsSource>(
            msg->provider.name,
            msg->text,
            msg->provider.voice,
            msg->provider.model,
            msg->provider.format
    );

    RCLCPP_INFO(rcl_logger, "Opening audio device");

        // Use pre-opened device proxy if possible
    std::unique_ptr<DeviceSink> device_sink = std::make_unique<DeviceSink>(
            get_parameter("device_name").as_string(),
            get_device_proxy(get_parameter("device_name").as_string())
    );

    auto audio_stream = std::make_unique<AudioStream>(
      std::move(tts_source),
      std::move(device_sink),
      std::string("tts request with text ") + msg->text.substr(0, 20) + "...",
      get_parameter("stream_queue_frames").as_int()
    );
    audio_stream->handle_ = msg->handle;
    admit_stream(std::move(audio_stream));
  }

  void check_streams_callback()
  {
        // Clean up any streams that have completed shutdown
    std::erase_if(audio_streams_, [](const std::unique_ptr<AudioStream> & stream) {
        return stream->shutdown_complete_.load();
    });
    // Drop backlogs of streams that no longer exist
    std::erase_if(backlog_, [this](const auto & entry) {
        return !find_active(entry.first) && !find_pending(entry.first);
    });

        // Start queued streams once all earlier streams have finished
    while (!pending_.empty() && !any_running()) {
      auto stream = std::move(pending_.front());
      pending_.pop_front();
      RCLCPP_INFO(rcl_logger, "Starting queued stream <%s>", stream->description_.c_str());
      start_stream(std::move(stream));
    }

    drain_backlogs();

    auto now = this->now();
    std::erase_if(dropped_uuids_, [&now](const auto & entry) {
        return (now - entry.second).seconds() > DROPPED_UUID_TIMEOUT_S;
    });

    refresh_mutes();

        // Pre-open the playback device to reduce latency on first use.
    if (!preopened_proxy_) {
      auto device_name = get_parameter("device_name").as_string();
      auto proxy = make_default_device_proxy();
      auto preopen_result = proxy->preopen(device_name, AudioDirection::Playback);
      if (preopen_result.has_value()) {
        // Retried every timer tick, so only warn on the first failure
        if (!preopen_failed_) {
          RCLCPP_WARN(rcl_logger, "Cannot pre-open audio device for playback: %s",
            preopen_result->c_str());
        }
        preopen_failed_ = true;
      } else {
        RCLCPP_INFO(rcl_logger, "Pre-opened audio device for playback: %s", device_name.c_str());
        preopened_proxy_ = std::move(proxy);
        preopen_failed_ = false;
      }
    }
  }

  void stop_streams_callback()
  {
    pending_.clear();
    backlog_.clear();
    for (auto & stream : audio_streams_) {
      stream->shutdown();
    }
    audio_streams_.clear();
  }

  void audio_chunk_callback(const audio2_stream_msgs::msg::AudioChunk::SharedPtr msg)
  {
    std::size_t hash_id = std::hash<std::thread::id>{}(std::this_thread::get_id()) % 10000;
    printf(
      "audio_chunk_callback: thread %zu Received audio chunk with %zu bytes sequence %u at %s\n",
      hash_id, msg->data.size(), msg->header.sequence, format_timestamp().c_str());
        // Here you can handle incoming audio chunks if needed
    const auto & uuid = msg->header.uuid;
    const UuidKey & key = uuid.uuid;
    const bool eos = msg->header.sequence == StreamHeader::SEQUENCE_EOS;

        // Discard chunks of ignored or cancelled streams
    if (auto dropped = dropped_uuids_.find(key); dropped != dropped_uuids_.end()) {
      if (eos) {
        dropped_uuids_.erase(dropped);
      } else {
        dropped->second = this->now();
      }
      return;
    }

        // Hold chunks of a queued stream until it starts
    if (find_pending(key)) {
      backlog_[key].push_back(msg);
      return;
    }

    if (AudioStream * audio_stream_raw = find_active(key)) {
          // Keep chunks in order behind any backlog still being fed to the stream
      if (auto backlog = backlog_.find(key);
        backlog != backlog_.end() && !backlog->second.empty())
      {
        backlog->second.push_back(msg);
        return;
      }
      // todo: why isn't this owned by the audio stream?
      message_source_->callback(msg, audio_stream_raw);
      return;
    }

    if (eos) {
      // End of a stream that is not playing; nothing to do
      return;
    }

        // We need to do an initial open of the source to get the format
    VIO_SOUNDFILE_HANDLE vio_handle;
    if (auto err = ropen_vio_from_vector(msg->data, vio_handle)) {
      RCLCPP_ERROR(rcl_logger, "Failed to open sound file for reading from buffer: %s",
        err->c_str());
      return;
    }

        // Use pre-opened device proxy if possible. The device is opened by the sink.
    std::unique_ptr<DeviceSink> device_sink = std::make_unique<DeviceSink>(
              get_parameter("device_name").as_string(),
              get_device_proxy(get_parameter("device_name").as_string())
    );
        // The stream has no source to negotiate with, so restrict the sink to the format
        // of the incoming audio. Otherwise the device opens at the sink's preferred rate
        // and channel count, and the audio plays at the wrong speed and pitch.
    device_sink->config_ranges_->samplerate_values = {vio_handle.fileh.samplerate()};
    device_sink->config_ranges_->channel_values = {vio_handle.fileh.channels()};
    RCLCPP_INFO(rcl_logger, "New audio stream: %d Hz, %d channels, handle %u",
      vio_handle.fileh.samplerate(), vio_handle.fileh.channels(), msg->header.handle);
    auto audio_stream = std::make_unique<AudioStream>(
              nullptr,
              std::move(device_sink),
              std::string("Stream for topic callback: ") + msg->header.description,
              get_parameter("stream_queue_frames").as_int()
    );
    audio_stream->stream_uuid_ = uuid;
    audio_stream->handle_ = msg->header.handle;

    switch (admit_stream(std::move(audio_stream))) {
      case Admission::StartNow:
        if (AudioStream * started = find_active(key)) {
          message_source_->callback(msg, started);
        }
        break;
      case Admission::Enqueue:
        backlog_[key].push_back(msg);
        break;
      case Admission::Drop:
        dropped_uuids_[key] = this->now();
        break;
    }
  }

  void play_file_local_callback(const audio2_stream_msgs::msg::PlayFile::SharedPtr msg)
  {
    RCLCPP_INFO(rcl_logger, "Received PlayFile message for local: path=%s, play_type=%d",
            msg->path.c_str(), msg->play_type);

    std::unique_ptr<SndFileSource> snd_file_source = std::make_unique<SndFileSource>(msg->path);

        // Use pre-opened device proxy if possible
    std::unique_ptr<DeviceSink> device_sink = std::make_unique<DeviceSink>(
            get_parameter("device_name").as_string(),
            get_device_proxy(get_parameter("device_name").as_string())
    );

    auto audio_stream = std::make_unique<AudioStream>(
            std::move(snd_file_source),
            std::move(device_sink),
            std::string("Local playback of ") + msg->path,
            get_parameter("stream_queue_frames").as_int()
    );
    audio_stream->handle_ = msg->handle;
    admit_stream(std::move(audio_stream));
  }

  // Apply the stream's handle (StreamHeader::HANDLE_*): start it, queue it, or drop it,
  // cancelling the other streams first for HANDLE_CANCEL. Returns what was done.
  Admission admit_stream(std::unique_ptr<AudioStream> audio_stream)
  {
    const bool others_exist = any_running() || !pending_.empty();
    auto decision = decide_handle(audio_stream->handle_, others_exist);
    if (decision.unknown_handle) {
      RCLCPP_WARN(rcl_logger, "Unknown stream handle %u for <%s>, mixing with other streams",
        audio_stream->handle_, audio_stream->description_.c_str());
    }

    if (decision.cancel_others) {
      RCLCPP_INFO(rcl_logger, "Stream <%s> cancels %zu active and %zu queued streams",
        audio_stream->description_.c_str(), audio_streams_.size(), pending_.size());
      auto now = this->now();
      for (auto & stream : audio_streams_) {
        if (!stream->source_) {
          // A chunk stream: discard its remaining chunks
          dropped_uuids_[stream->stream_uuid_.uuid] = now;
        }
        stream->request_shutdown();
      }
      for (auto & stream : pending_) {
        if (!stream->source_) {
          dropped_uuids_[stream->stream_uuid_.uuid] = now;
        }
      }
      pending_.clear();
      backlog_.clear();
    }

    Admission admission = decision.admission;
    switch (admission) {
      case Admission::StartNow:
        RCLCPP_INFO(rcl_logger, "Starting stream <%s> with handle %u",
          audio_stream->description_.c_str(), audio_stream->handle_);
        if (!start_stream(std::move(audio_stream))) {
          admission = Admission::Drop;
        }
        break;
      case Admission::Enqueue:
        RCLCPP_INFO(rcl_logger, "Queueing stream <%s> behind %zu active and %zu queued streams",
          audio_stream->description_.c_str(), audio_streams_.size(), pending_.size());
        pending_.push_back(std::move(audio_stream));
        break;
      case Admission::Drop:
        RCLCPP_INFO(rcl_logger, "Ignoring stream <%s> because other streams are playing",
          audio_stream->description_.c_str());
        break;
    }
    refresh_mutes();
    return admission;
  }

  // Start a stream and add it to the active streams. Returns false if it could not start.
  bool start_stream(std::unique_ptr<AudioStream> audio_stream)
  {
    auto start_result = audio_stream->start();
    if (start_result.has_value()) {
      RCLCPP_ERROR(rcl_logger, "Cannot start audio stream: %s", start_result->c_str());
      backlog_.erase(audio_stream->stream_uuid_.uuid);
      return false;
    }
    audio_streams_.push_back(std::move(audio_stream));
    return true;
  }

  // Feed buffered chunks to started streams, as fast as their audio queues have room.
  void drain_backlogs()
  {
    for (auto & [key, chunks] : backlog_) {
      AudioStream * stream = find_active(key);
      if (!stream) {
        continue;  // Still queued
      }
      while (!chunks.empty() && !stream->shutdown_flag_.load() &&
        stream->queue_.write_available() > BACKLOG_QUEUE_MARGIN)
      {
        auto chunk = std::move(chunks.front());
        chunks.pop_front();
        message_source_->callback(chunk, stream);
      }
    }
  }

  void refresh_mutes()
  {
    std::vector<AudioStream *> streams;
    streams.reserve(audio_streams_.size());
    for (auto & stream : audio_streams_) {
      streams.push_back(stream.get());
    }
    update_mutes(streams);
  }

  // Whether any started stream is still playing (not finished or stopping)
  bool any_running()
  {
    for (auto & stream : audio_streams_) {
      if (!stream->shutdown_flag_.load() && !stream->shutdown_complete_.load()) {
        return true;
      }
    }
    return false;
  }

  // Active (started) stream with the given UUID, or nullptr
  AudioStream * find_active(const UuidKey & key)
  {
    for (auto & stream : audio_streams_) {
      if (stream->stream_uuid_.uuid == key) {
        return stream.get();
      }
    }
    return nullptr;
  }

  // Queued stream with the given UUID, or nullptr
  AudioStream * find_pending(const UuidKey & key)
  {
    for (auto & stream : pending_) {
      if (stream->stream_uuid_.uuid == key) {
        return stream.get();
      }
    }
    return nullptr;
  }

  // Get a device proxy for playback, preferring the pre-opened one.
  std::unique_ptr<IAudioDeviceProxy> get_device_proxy(const std::string & device_name)
  {
    if (preopened_proxy_ && preopened_proxy_->device_name() == device_name) {
      printf("get_device_proxy: Using pre-opened audio device\n");
      return std::move(preopened_proxy_);
    }
    auto proxy = make_default_device_proxy();
    auto preopen_result = proxy->preopen(device_name, AudioDirection::Playback);
    if (preopen_result.has_value()) {
      RCLCPP_WARN(rcl_logger, "Cannot pre-open audio device for playback: %s",
        preopen_result->c_str());
      // A fresh proxy without the pre-open error; the sink reports any error when it opens
      proxy = make_default_device_proxy();
    }
    return proxy;
  }

private:
  rclcpp::Subscription<audio2_stream_msgs::msg::PlayFile>::SharedPtr play_file_local_subscriber_;
  rclcpp::Subscription<audio2_stream_msgs::msg::AudioChunk>::SharedPtr chunk_subscriber_;
  rclcpp::Subscription<audio2_stream_msgs::msg::TtsRequest>::SharedPtr tts_request_subscriber_;
  // Started streams, oldest first
  std::vector<std::unique_ptr<AudioStream>> audio_streams_;
  // Streams with HANDLE_QUEUE waiting for the active streams to finish, oldest first
  std::deque<std::unique_ptr<AudioStream>> pending_;
  // Chunks received for queued chunk streams, or not yet fed to a started one
  std::map<UuidKey, std::deque<AudioChunk::SharedPtr>> backlog_;
  // UUIDs of ignored or cancelled chunk streams, with the time their last chunk arrived
  std::map<UuidKey, rclcpp::Time> dropped_uuids_;
  std::unique_ptr<MessageSource> message_source_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<std::jthread> check_thread_;
  std::unique_ptr<IAudioDeviceProxy> preopened_proxy_;
  bool preopen_failed_ = false;
};


int main(int argc, [[maybe_unused]] char ** argv)
{
  RCLCPP_INFO(rcl_logger, "Starting Audio2PlayNode thread %zu at %s",
                std::hash<std::thread::id>{}(std::this_thread::get_id()) % 10000,
                format_timestamp().c_str());
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Audio2PlayNode>();
  rclcpp::spin(node);
  RCLCPP_INFO(rcl_logger, "audio2_play_node shutting down...");
  // Ctrl-C under ros2 launch sends SIGINT twice (from the terminal and from launch), and a
  // second one must not kill the process during cleanup. Release the node (closing streams and
  // the audio device) while rclcpp's handler still absorbs SIGINT, then ignore SIGINT once
  // rclcpp::shutdown() restores the default action, for the remaining middleware teardown.
  node.reset();
  rclcpp::shutdown();
  std::signal(SIGINT, SIG_IGN);

  return 0;
}
