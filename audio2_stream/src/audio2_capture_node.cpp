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

#include "rclcpp/rclcpp.hpp"
#include "audio2_stream/buffer_file.hpp"
#include "audio2_stream/AudioStream.hpp"
#include "boost/lockfree/spsc_queue.hpp"

#include "audio2_stream_msgs/msg/audio_chunk.hpp"

static auto rcl_logger = rclcpp::get_logger("audio2_capture");

static std::vector<std::unique_ptr<AudioStream>> audio_streams_;

class Audio2CaptureNode : public rclcpp::Node {
public:
  Audio2CaptureNode()
  : Node("audio2_capture_node")
  {
        // Parameters

    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.description = "Audio device name for capture";
    param_desc.additional_constraints =
      "An ALSA device name on Linux; on Windows \"default\", or part of a device's name or its ID.";
    param_desc.read_only = false;
    this->declare_parameter<std::string>("device_name", DEFAULT_DEVICE_NAME, param_desc);
    printf("Audio device name: %s\n", get_parameter("device_name").as_string().c_str());

    param_desc.description = "Audio sample format for capture";
    param_desc.additional_constraints =
      "A SampleFormat integer value (same numbering as ALSA snd_pcm_format_t; 14 = float).";
    param_desc.read_only = false;
    this->declare_parameter<int>("sample_format", static_cast<int>(DEFAULT_SAMPLE_FORMAT),
      param_desc);

    param_desc.description = "Number of audio channels for capture";
    param_desc.additional_constraints = "Must be a positive integer (typically 1 or 2).";
    param_desc.read_only = false;
    this->declare_parameter<int>("channels", 2, param_desc);

    param_desc.description = "Sample rate for audio capture";
    param_desc.additional_constraints = "Must be a positive integer (e.g., 44100, 48000).";
    param_desc.read_only = false;
    this->declare_parameter<int>("samplerate", 48000, param_desc);

    param_desc.description = "Number of frames per audio chunk in the stream";
    param_desc.additional_constraints = "Must be a positive integer.";
    param_desc.read_only = false;
    this->declare_parameter<int>("stream_queue_frames", STREAM_QUEUE_FRAMES, param_desc);

    param_desc.description = "Topic name for publishing captured audio chunks";
    param_desc.additional_constraints = "Must be a valid ROS topic name.";
    param_desc.read_only = false;
    this->declare_parameter<std::string>("audio_topic", "audio_stream_chunks", param_desc);

    param_desc.description = "Auto-start capturing on node initialization";
    param_desc.additional_constraints = "Boolean value (true/false).";
    param_desc.read_only = false;
    this->declare_parameter<bool>("auto_start", true, param_desc);

        // Register shutdown callback
    using rclcpp::contexts::get_global_default_context;
    get_global_default_context()->add_pre_shutdown_callback(
      [this]() {
        this->stop_streams_callback();
            });

        // Timer to check and clean up finished streams
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&Audio2CaptureNode::check_streams_callback, this));

        // Auto-start if configured
    if (get_parameter("auto_start").as_bool()) {
      start_capture();
    }

    RCLCPP_INFO(rcl_logger, "Audio2CaptureNode initialized.");
  }

  void check_streams_callback()
  {
    std::erase_if(audio_streams_, [](const std::unique_ptr<AudioStream> & stream) {
        return stream->shutdown_complete_.load();
        });
  }

  void stop_streams_callback()
  {
    printf("Audio2CaptureNode: Stopping all audio streams...\n");
    for (auto & stream : audio_streams_) {
      stream->shutdown();
    }
    printf("Audio2CaptureNode: Clearing streams\n");
    audio_streams_.clear();
    printf("Audio2CaptureNode: All audio streams stopped.\n");
  }

  void start_capture()
  {
    int channels = get_parameter("channels").as_int();
    int samplerate = get_parameter("samplerate").as_int();
    std::string audio_topic = get_parameter("audio_topic").as_string();

    RCLCPP_INFO(rcl_logger, "Starting audio capture: channels=%d, samplerate=%d, topic=%s",
            channels, samplerate, audio_topic.c_str());

        // Create the DeviceSource for capturing audio, using the platform's default backend
    std::unique_ptr<DeviceSource> device_source = std::make_unique<DeviceSource>(
            get_parameter("device_name").as_string(),
            nullptr
    );
        // Capture with the requested parameters
    device_source->config_ranges_->samplerate_values = {samplerate};
    device_source->config_ranges_->channel_values = {channels};
    DeviceSource * device_source_raw = device_source.get();  // still used after the move below

        // Create the message publisher
    auto publisher = this->create_publisher<audio2_stream_msgs::msg::AudioChunk>(audio_topic,
      AUDIO_CHUNK_QOS);

        // Create the MessageSink for publishing captured audio chunks
    std::unique_ptr<MessageSink> message_sink = std::make_unique<MessageSink>(
            audio_topic,
            SF_FORMAT_DEFAULT,
            publisher,
            std::string("Audio capture from ") + get_parameter("device_name").as_string()
    );

        // Create the AudioStream to connect source and sink
    auto audio_stream = std::make_unique<AudioStream>(
            std::move(device_source),
            std::move(message_sink),
            std::string("Audio capture stream on ") + audio_topic,
            get_parameter("stream_queue_frames").as_int()
    );

    auto open_result = device_source_raw->open(AudioDirection::Capture, audio_stream.get());
    if (open_result.has_value()) {
      RCLCPP_ERROR(rcl_logger, "Cannot open audio device for capture: %s",
        open_result->c_str());
      return;
    }

    auto start_result = audio_stream->start();
    if (start_result.has_value()) {
      RCLCPP_ERROR(rcl_logger, "Cannot start audio stream: %s", start_result->c_str());
      return;
    }
    audio_streams_.push_back(std::move(audio_stream));

    RCLCPP_INFO(rcl_logger, "Audio capture started on topic '%s'", audio_topic.c_str());
  }

private:
    //std::vector<std::unique_ptr<AudioStream>> audio_streams_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{
  printf("argc: %d\n", argc);
  for (int i = 0; i < argc; i++) {
    printf("argv[%d]: %s\n", i, argv[i]);
  }

  RCLCPP_INFO(rcl_logger, "Starting Audio2CaptureNode thread %zu at %s",
                std::hash<std::thread::id>{}(std::this_thread::get_id()) % 10000,
                format_timestamp().c_str());
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Audio2CaptureNode>();
  rclcpp::spin(node);
  printf("audio2_capture_node shutting down.\n");
  // Ctrl-C under ros2 launch sends SIGINT twice (from the terminal and from launch), and a
  // second one must not kill the process during cleanup. Release the node (closing streams and
  // the audio device) while rclcpp's handler still absorbs SIGINT, then ignore SIGINT once
  // rclcpp::shutdown() restores the default action, for the remaining middleware teardown.
  node.reset();
  rclcpp::shutdown();
  std::signal(SIGINT, SIG_IGN);
  return 0;
}
