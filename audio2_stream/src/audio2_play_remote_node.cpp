#include <csignal>
#include <cstdio>
#include <cstring>
#include <sndfile.h>
#include <alsa/asoundlib.h>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <fstream>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "audio2_stream/alsaops.hpp"
#include "audio2_stream/buffer_file.hpp"
#include "audio2_stream/AudioStream.hpp"
#include "boost/lockfree/spsc_queue.hpp"

#include "audio2_stream_msgs/msg/play_file.hpp"
#include "audio2_stream_msgs/msg/audio_chunk.hpp"

static auto rcl_logger = rclcpp::get_logger("audio2_stream");

class Audio2PlayRemoteNode : public rclcpp::Node {
public:
  Audio2PlayRemoteNode()
  : Node("audio2_play_remote_node")
  {
        // Parameters
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.description = "Number of frames per audio chunk in the stream";
    param_desc.additional_constraints = "Must be a positive integer.";
    param_desc.read_only = false;
    this->declare_parameter<int>("stream_queue_frames", STREAM_QUEUE_FRAMES, param_desc);

        // Subscriber for local PlayFile messages

    play_file_remote_subscriber_ = this->create_subscription<audio2_stream_msgs::msg::PlayFile>(
            "play_file_remote", 10,
            std::bind(&Audio2PlayRemoteNode::play_file_remote_callback, this,
      std::placeholders::_1));

        // Register shutdown callback
    rclcpp::on_shutdown(
            std::bind(&Audio2PlayRemoteNode::stop_streams_callback, this));

        // Timer to check and clean up finished streams
        // Timer to check and clean up finished streams
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&Audio2PlayRemoteNode::check_streams_callback, this));

    RCLCPP_INFO(rcl_logger, "Audio2PlayRemoteNode initialized.");
  }

  void check_streams_callback()
  {
    std::erase_if(audio_streams_, [](const std::unique_ptr<AudioStream> & stream) {
        return stream->shutdown_complete_.load();
        });
  }

  void stop_streams_callback()
  {
    for (auto & stream : audio_streams_) {
      stream->shutdown();
    }
    audio_streams_.clear();
  }

  void play_file_remote_callback(const audio2_stream_msgs::msg::PlayFile::SharedPtr msg)
  {
    RCLCPP_INFO(rcl_logger, "Received PlayFile message for remote: path=%s, play_type=%d",
            msg->path.c_str(), msg->play_type);
    auto file_path = msg->path;

        // Open the local file
    std::unique_ptr<SndFileSource> snd_file_source = std::make_unique<SndFileSource>(file_path);
    auto open_result = snd_file_source->open();
    if (open_result.has_value()) {
      RCLCPP_ERROR(rcl_logger, "Cannot open file <%s>: %s", file_path.c_str(),
        open_result.value().c_str());
      return;
    }

        // Create the message publisher
    auto publisher = this->create_publisher<audio2_stream_msgs::msg::AudioChunk>(msg->topic,
      AUDIO_CHUNK_QOS);
    int channels = snd_file_source->sndfileh_.channels();
    int samplerate = snd_file_source->sndfileh_.samplerate();
    int sndfile_format = snd_file_source->sndfileh_.format();
    RCLCPP_INFO(rcl_logger, "Opened file %s: channels=%d, samplerate=%d, format=0x%X",
            file_path.c_str(), channels, samplerate, sndfile_format);

        // Open the playback device
    std::unique_ptr<MessageSink> message_sink = std::make_unique<MessageSink>(
            msg->topic,
            channels,
            samplerate,
            SF_FORMAT_DEFAULT,
            publisher,
            file_path
    );

    auto audio_stream = std::make_unique<AudioStream>(
            SFG_RW_FORMAT,
            std::move(snd_file_source),
            std::move(message_sink),
            std::string("Remote playback of ") + file_path,
            get_parameter("stream_queue_frames").as_int()
    );

    audio_stream->start();
    audio_streams_.push_back(std::move(audio_stream));

    RCLCPP_INFO(rcl_logger, "Enqueued file for remote playback '%s'", file_path.c_str());
  }

private:
  rclcpp::Subscription<audio2_stream_msgs::msg::PlayFile>::SharedPtr play_file_remote_subscriber_;
  std::vector<std::unique_ptr<AudioStream>> audio_streams_;
  std::unique_ptr<MessageSource> message_source_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<std::jthread> check_thread_;
};


int main(int argc, [[maybe_unused]] char ** argv)
{
  RCLCPP_INFO(rcl_logger, "Starting Audio2PlayRemoteNode thread %zu at %s",
                std::hash<std::thread::id>{}(std::this_thread::get_id()) % 10000,
                format_timestamp().c_str());
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Audio2PlayRemoteNode>();
  rclcpp::spin(node);
  RCLCPP_INFO(rcl_logger, "Audio2PlayRemoteNode shutting down...");
  rclcpp::shutdown();
  return 0;
}
