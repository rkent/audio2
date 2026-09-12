#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <RtAudio.h>
#include <sndfile.h>
#include <sndfile.hh>

#include "audio2_stream/ra_buffer_file.hpp"
#include "audio2_stream_msgs/msg/play_file.hpp"
#include "boost/lockfree/spsc_queue.hpp"
#include "rclcpp/rclcpp.hpp"

static auto rcl_logger = rclcpp::get_logger("ra_audio2_play");

class RaAudio2PlayNode : public rclcpp::Node
{
private:
  struct ActivePlayback
  {
    std::string path;
    bool loop{false};
    std::atomic<bool> stop_requested{false};
    std::atomic<bool> finished{false};
    std::unique_ptr<std::thread> worker_thread;
  };

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

    start_playback(msg->path, msg->play_type == audio2_stream_msgs::msg::PlayFile::PLAY_START);
  }

  void stop_playback(const std::string & path)
  {
    for (auto & active : active_files_) {
      if (path.empty() || active->path == path) {
        active->stop_requested.store(true);
      }
    }
  }

  void start_playback(const std::string & path, bool loop)
  {
    auto active_stream = std::make_unique<ActivePlayback>();
    active_stream->path = path;
    active_stream->loop = loop;
    active_stream->stop_requested.store(false);
    active_stream->finished.store(false);

    ActivePlayback * stream_ptr = active_stream.get();

    active_stream->worker_thread = std::make_unique<std::thread>(
      [this, stream_ptr]() {
        this->playback_worker(stream_ptr);
      });

    active_files_.push_back(std::move(active_stream));
    RCLCPP_INFO(rcl_logger, "Enqueued file %s (loop=%s)", path.c_str(), loop ? "true" : "false");
  }

  void playback_worker(ActivePlayback * stream_info)
  {
    SndfileHandle fileh(stream_info->path);
    if (fileh.error()) {
      RCLCPP_ERROR(
        rcl_logger, "Cannot open file <%s>: %s",
        stream_info->path.c_str(), fileh.strError());
      stream_info->finished.store(true);
      return;
    }

    // Explicitly disable libsndfile normalization to handle integer scaling manually
    fileh.command(SFC_SET_NORM_FLOAT, NULL, SF_FALSE);
    fileh.command(SFC_SET_NORM_DOUBLE, NULL, SF_FALSE);

    const int queue_capacity = 100;
    boost::lockfree::spsc_queue<std::vector<uint8_t>> audio_queue(queue_capacity);
    std::atomic<bool> data_available(false);

    RaWriteThread writer(
      static_cast<unsigned int>(fileh.channels()),
      static_cast<unsigned int>(fileh.samplerate()),
      RTAUDIO_FLOAT32,
      &audio_queue,
      &stream_info->stop_requested,
      &data_available);

    if (!writer.get_error().empty()) {
      RCLCPP_ERROR(
        rcl_logger, "Failed to start RtAudio writer for %s: %s",
        stream_info->path.c_str(), writer.get_error().c_str());
      stream_info->finished.store(true);
      return;
    }

    const int frames_per_chunk = 1024;
    int channels = fileh.channels();
    int samples_per_chunk = frames_per_chunk * channels;
    SfgRwFormat r_format = sfg_format_from_sndfile_format(fileh.format());
    SfgRwFormat w_format = SFG_FLOAT;

    std::vector<uint8_t> r_buffer;
    std::vector<uint8_t> w_buffer;
    auto create_res = create_convert_vectors(r_format, w_format, samples_per_chunk, r_buffer,
      w_buffer);
    if (create_res.has_value()) {
      RCLCPP_ERROR(rcl_logger, "Failed to create convert buffers: %s", create_res.value().c_str());
      stream_info->finished.store(true);
      return;
    }

    do {
      fileh.seek(0, SEEK_SET);

      while (!stream_info->stop_requested.load() && rclcpp::ok()) {
        int samples_read = sfg_read(fileh, r_format, r_buffer.data(), samples_per_chunk);
        if (samples_read <= 0) {
          break;
        }

        int samples_converted = convert_types(
          r_format, w_format, r_buffer.data(), w_buffer.data(), samples_read);
        if (samples_converted <= 0) {
          RCLCPP_ERROR(rcl_logger, "Error converting samples from format %d to float", r_format);
          break;
        }

        std::vector<uint8_t> byte_chunk(samples_converted * sizeof(float));
        std::memcpy(byte_chunk.data(), w_buffer.data(), byte_chunk.size());

        while (!audio_queue.push(byte_chunk) && !stream_info->stop_requested.load() &&
          rclcpp::ok())
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        data_available.store(true);
      }
    } while (stream_info->loop && !stream_info->stop_requested.load() && rclcpp::ok());

    writer.drain_and_close();
    stream_info->finished.store(true);
  }

  void check_streams_callback()
  {
    for (auto it = active_files_.begin(); it != active_files_.end(); ) {
      if ((*it)->finished.load()) {
        if ((*it)->worker_thread && (*it)->worker_thread->joinable()) {
          (*it)->worker_thread->join();
        }
        it = active_files_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void stop_streams_callback()
  {
    for (auto & active : active_files_) {
      active->stop_requested.store(true);
    }
    for (auto & active : active_files_) {
      if (active->worker_thread && active->worker_thread->joinable()) {
        active->worker_thread->join();
      }
    }
    active_files_.clear();
  }

private:
  rclcpp::Subscription<audio2_stream_msgs::msg::PlayFile>::SharedPtr play_file_local_subscriber_;
  std::vector<std::unique_ptr<ActivePlayback>> active_files_;
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
