#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "audio2_stream/AudioStream.hpp"
#include "audio2_stream_msgs/msg/play_file.hpp"

static auto rcl_logger = rclcpp::get_logger("ra_audio2_play");

class RaAudio2PlayNode : public rclcpp::Node
{
public:
  RaAudio2PlayNode()
  : Node("ra_audio2_play_node")
  {
    // Subscriber for local PlayFile messages
    play_file_local_subscriber_ = this->create_subscription<audio2_stream_msgs::msg::PlayFile>(
      "play_file_local", 10,
      std::bind(&RaAudio2PlayNode::play_file_local_callback, this, std::placeholders::_1));

    // Register shutdown callback
    rclcpp::on_shutdown(
      std::bind(&RaAudio2PlayNode::stop_streams_callback, this));

    // Timer to check and clean up finished streams
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&RaAudio2PlayNode::check_streams_callback, this));

    RCLCPP_INFO(rcl_logger, "RaAudio2PlayNode initialized.");
  }

  ~RaAudio2PlayNode()
  {
    RCLCPP_INFO(rcl_logger, "RaAudio2PlayNode shutting down.");
    stop_streams_callback();
  }

  void play_file_local_callback(const audio2_stream_msgs::msg::PlayFile::SharedPtr msg)
  {
    RCLCPP_INFO(
      rcl_logger, "Received PlayFile message for local: path=%s, play_type=%d",
      msg->path.c_str(), msg->play_type);

    if (msg->play_type == audio2_stream_msgs::msg::PlayFile::PLAY_STOP) {
      stop_playback(msg->path);
      return;
    }

    auto snd_file_source = std::make_unique<SndFileSource>(msg->path);
    auto ra_sink = std::make_unique<RaSink>(RTAUDIO_FLOAT32);

    auto audio_stream = std::make_unique<AudioStream>(
      std::move(snd_file_source),
      std::move(ra_sink),
      std::string("Local playback of ") + msg->path,
      STREAM_QUEUE_FRAMES
    );

    auto start_result = audio_stream->start();
    if (start_result.has_value()) {
      RCLCPP_ERROR(rcl_logger, "Cannot start audio stream: %s", start_result->c_str());
      return;
    }

    audio_streams_.push_back(std::move(audio_stream));
    RCLCPP_INFO(rcl_logger, "Enqueued file %s", msg->path.c_str());
  }

  void stop_playback(const std::string & path)
  {
    for (auto & stream : audio_streams_) {
      if (path.empty() || stream->description_.find(path) != std::string::npos) {
        stream->shutdown();
      }
    }
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

private:
  rclcpp::Subscription<audio2_stream_msgs::msg::PlayFile>::SharedPtr play_file_local_subscriber_;
  std::vector<std::unique_ptr<AudioStream>> audio_streams_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RaAudio2PlayNode>();
  rclcpp::spin(node);
  RCLCPP_INFO(rcl_logger, "ra_audio2_play_node shutting down...");
  rclcpp::shutdown();
  return 0;
}
