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

static auto rcl_logger = rclcpp::get_logger("audio2_play");

class Audio2PlayNode : public rclcpp::Node {
public:
  Audio2PlayNode()
  : Node("audio2_play_node")
  {
        // Parameters

    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.description = "ALSA device name for audio playback";
    param_desc.additional_constraints = "Must be a valid ALSA device name (a string).";
    param_desc.read_only = false;
    this->declare_parameter<std::string>("alsa_device_name", ALSA_DEVICE_NAME, param_desc);

    param_desc.description = "ALSA audio format for playback";
    param_desc.additional_constraints = "Must be a valid snd_pcm_format_t integer value.";
    param_desc.read_only = false;
    this->declare_parameter<int>("alsa_format", static_cast<int>(ALSA_FORMAT), param_desc);

    param_desc.description = "Number of frames per audio chunk in the stream";
    param_desc.additional_constraints = "Must be a positive integer.";
    param_desc.read_only = false;
    this->declare_parameter<int>("stream_queue_frames", STREAM_QUEUE_FRAMES, param_desc);

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

  void check_streams_callback()
  {
        //static int count = 0;
        //count++;
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

  void audio_chunk_callback(const audio2_stream_msgs::msg::AudioChunk::SharedPtr msg)
  {
    std::size_t hash_id = std::hash<std::thread::id>{}(std::this_thread::get_id()) % 10000;
    printf(
      "audio_chunk_callback: thread %zu Received audio chunk with %zu bytes sequence %u at %s\n",
      hash_id, msg->data.size(), msg->header.sequence, format_timestamp().c_str());
        // Here you can handle incoming audio chunks if needed
    auto uuid = msg->header.uuid;
    AudioStream * audio_stream_raw = nullptr;
        // Locate the audio_stream with the matching UUID
    for (auto & stream : audio_streams_) {
      if (stream->stream_uuid_ == uuid) {
                // Pass the chunk to the stream's source
                // message_source_->callback(msg, stream.get());
        audio_stream_raw = stream.get();
        break;
      }
    }
    if (!audio_stream_raw) {

            // We need to do an initial open of the source to get the format
      VIO_SOUNDFILE_HANDLE vio_handle;
      if (auto err = ropen_vio_from_vector(msg->data, vio_handle)) {
        RCLCPP_ERROR(rcl_logger, "Failed to open sound file for reading from buffer: %s",
          err->c_str());
        return;
      }

            // Create a matching audio stream
            // Open the playback device
      std::unique_ptr<AlsaSink> alsa_sink = std::make_unique<AlsaSink>(
                get_parameter("alsa_device_name").as_string(),
                vio_handle.fileh.channels(),
                vio_handle.fileh.samplerate(),
                static_cast<snd_pcm_format_t>(get_parameter("alsa_format").as_int())
      );
      auto alsa_open_result = alsa_sink->open(SND_PCM_STREAM_PLAYBACK);
      if (alsa_open_result.has_value()) {
        RCLCPP_ERROR(rcl_logger, "Cannot open ALSA device for new stream: %s",
          alsa_open_result->c_str());
        return;
      }
      auto audio_stream = std::make_unique<AudioStream>(
                sfg_format_from_alsa_format(alsa_sink->format_),
                nullptr,
                std::move(alsa_sink),
                std::string("Stream for topic callback"),
                get_parameter("stream_queue_frames").as_int()
      );
      audio_stream->stream_uuid_ = uuid;
      audio_stream->start();
      audio_stream_raw = audio_stream.get();
      audio_streams_.push_back(std::move(audio_stream));
      RCLCPP_INFO_STREAM(rcl_logger, "\n\nCreated new audio stream for UUID");
    }
    message_source_->callback(msg, audio_stream_raw);
  }

  void play_file_local_callback(const audio2_stream_msgs::msg::PlayFile::SharedPtr msg)
  {
    RCLCPP_INFO(rcl_logger, "Received PlayFile message for local: path=%s, play_type=%d",
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

    int channels = snd_file_source->sndfileh_.channels();
    int samplerate = snd_file_source->sndfileh_.samplerate();
    int sndfile_format = snd_file_source->sndfileh_.format();
    RCLCPP_INFO(rcl_logger, "Opened file %s: channels=%d, samplerate=%d, format=0x%X",
            file_path.c_str(), channels, samplerate, sndfile_format);

        // Open the playback device
    std::unique_ptr<AlsaSink> alsa_sink = std::make_unique<AlsaSink>(
            get_parameter("alsa_device_name").as_string(),
            channels,
            samplerate,
            static_cast<snd_pcm_format_t>(get_parameter("alsa_format").as_int())
    );
    auto alsa_open_result = alsa_sink->open(SND_PCM_STREAM_PLAYBACK);
    if (alsa_open_result.has_value()) {
      RCLCPP_ERROR(rcl_logger, "Cannot open ALSA device: %s", alsa_open_result->c_str());
      return;
    }

        // during open, alsa may change the format if the original is unsupported.
    SfgRwFormat rw_format = sfg_format_from_alsa_format(alsa_sink->format_);
    auto audio_stream = std::make_unique<AudioStream>(
            rw_format,
            std::move(snd_file_source),
            std::move(alsa_sink),
            std::string("Local playback of ") + file_path,
            get_parameter("stream_queue_frames").as_int()
    );

    audio_stream->start();
    audio_streams_.push_back(std::move(audio_stream));

    RCLCPP_INFO(rcl_logger, "Enqueued file %s", file_path.c_str());

  }

private:
  rclcpp::Subscription<audio2_stream_msgs::msg::PlayFile>::SharedPtr play_file_local_subscriber_;
  rclcpp::Subscription<audio2_stream_msgs::msg::AudioChunk>::SharedPtr chunk_subscriber_;
  std::vector<std::unique_ptr<AudioStream>> audio_streams_;
  std::unique_ptr<MessageSource> message_source_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<std::jthread> check_thread_;
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
  rclcpp::shutdown();
  return 0;
}
