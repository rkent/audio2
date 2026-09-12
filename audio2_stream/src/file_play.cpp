/**
 * @file file_play.cpp
 * @brief Play an audio file using RtAudio.
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>

#include <RtAudio.h>
#include <sndfile.hh>

#include "rclcpp/rclcpp.hpp"

// Global flag to signal thread shutdown
std::atomic<bool> shutdown_flag(false);

static auto rcl_logger = rclcpp::get_logger("audio2_stream/file_play");

void signal_handler(int signal)
{
  if (signal == SIGINT) {
    RCLCPP_INFO(rcl_logger, "Keyboard interrupt received. Shutting down.");
    shutdown_flag.store(true);
    rclcpp::shutdown();
  }
}

struct PlaybackData
{
  SndfileHandle fileh;
  std::atomic<bool> * shutdown_flag{nullptr};
  std::atomic<bool> finished{false};
  int channels{0};
};

static int audioCallback(
  void * outputBuffer,
  void * /*inputBuffer*/,
  unsigned int nBufferFrames,
  double /*streamTime*/,
  RtAudioStreamStatus status,
  void * userData)
{
  if (status) {
    RCLCPP_WARN(rcl_logger, "Stream underflow/overflow detected: %u", status);
  }

  auto * data = static_cast<PlaybackData *>(userData);
  if (data->shutdown_flag && data->shutdown_flag->load()) {
    return 2;  // Abort immediately
  }

  float * out = static_cast<float *>(outputBuffer);
  sf_count_t frames_read = data->fileh.readf(out, nBufferFrames);

  if (frames_read < static_cast<sf_count_t>(nBufferFrames)) {
    // Zero-fill the remaining frames
    sf_count_t remaining_frames = static_cast<sf_count_t>(nBufferFrames) - frames_read;
    std::memset(
      out + (frames_read * data->channels),
      0,
      remaining_frames * data->channels * sizeof(float));
    data->finished.store(true);
    return 1;  // Stop and drain stream
  }

  return 0;  // Continue playback
}

class FilePlayerNode : public rclcpp::Node {
public:
  FilePlayerNode()
  : Node("file_play")
  {
  }

  void play_file_data(const std::string & file_path)
  {
    RCLCPP_INFO(rcl_logger, "Playing audio data from file: %s", file_path.c_str());

    // Open the sound file
    SndfileHandle fileh(file_path);
    if (fileh.error()) {
      RCLCPP_ERROR(rcl_logger, "Cannot open file <%s>: %s", file_path.c_str(), fileh.strError());
      return;
    }

    int channels = fileh.channels();
    unsigned int sample_rate = static_cast<unsigned int>(fileh.samplerate());
    RCLCPP_INFO(
      rcl_logger,
      "File opened: %s, Sample Rate: %u, Channels: %d, Format: 0x%X",
      file_path.c_str(), sample_rate, channels, fileh.format());

    RtAudio dac;
    if (dac.getDeviceIds().empty()) {
      RCLCPP_ERROR(rcl_logger, "No audio output devices found by RtAudio.");
      return;
    }

    PlaybackData playback_data;
    playback_data.fileh = fileh;
    playback_data.shutdown_flag = &shutdown_flag;
    playback_data.finished.store(false);
    playback_data.channels = channels;

    RtAudio::StreamParameters oParams;
    oParams.deviceId = dac.getDefaultOutputDevice();
    oParams.nChannels = static_cast<unsigned int>(channels);
    oParams.firstChannel = 0;

    unsigned int bufferFrames = 512;
    RtAudio::StreamOptions options;
    options.flags = RTAUDIO_SCHEDULE_REALTIME;

    RtAudioErrorType err = dac.openStream(
      &oParams,
      nullptr,
      RTAUDIO_FLOAT32,
      sample_rate,
      &bufferFrames,
      &audioCallback,
      &playback_data,
      &options);

    if (err != RTAUDIO_NO_ERROR) {
      RCLCPP_ERROR(rcl_logger, "Failed to open RtAudio stream: %s", dac.getErrorText().c_str());
      return;
    }

    err = dac.startStream();
    if (err != RTAUDIO_NO_ERROR) {
      RCLCPP_ERROR(rcl_logger, "Failed to start RtAudio stream: %s", dac.getErrorText().c_str());
      dac.closeStream();
      return;
    }

    RCLCPP_INFO(rcl_logger, "Playback started (buffer frames: %u).", bufferFrames);

    while (dac.isStreamRunning() && rclcpp::ok() && !shutdown_flag.load() &&
      !playback_data.finished.load())
    {
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }

    // Wait for the stream to drain if we reached EOF
    if (playback_data.finished.load() && !shutdown_flag.load() && rclcpp::ok()) {
      while (dac.isStreamRunning() && rclcpp::ok() && !shutdown_flag.load()) {
        rclcpp::sleep_for(std::chrono::milliseconds(20));
      }
    }

    if (dac.isStreamOpen()) {
      if (dac.isStreamRunning()) {
        dac.stopStream();
      }
      dac.closeStream();
    }

    RCLCPP_INFO(rcl_logger, "Playback finished.");
  }
};

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: file_play <file_path>\n";
    return 1;
  }

  std::signal(SIGINT, signal_handler);
  rclcpp::init(argc, argv);

  auto file_player = std::make_shared<FilePlayerNode>();
  file_player->play_file_data(argv[1]);

  rclcpp::shutdown();
  return 0;
}
