/**
 * @file file_play.cpp
 * @brief Test file read, loop through snd_file buffers to the audio device.
 */

#include <sndfile.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <sndfile.hh>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "audio2_stream/buffer_file.hpp"

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
    SndfileHandle fileh = SndfileHandle(file_path);
    if (fileh.error()) {
      RCLCPP_ERROR(rcl_logger, "Cannot open file <%s>: %s", file_path.c_str(), fileh.strError());
      return;
    }
    RCLCPP_INFO(rcl_logger, "File opened: %s, Sample Rate: %d, Format %X", file_path.c_str(),
      fileh.samplerate(), fileh.format());
    printf("channels: %d\n", fileh.channels());

    AudioHwParams hw_vals;
    hw_vals.channels = fileh.channels();
    hw_vals.samplerate = fileh.samplerate();
    hw_vals.format = ALSA_FORMAT;
    AudioSwParams sw_vals;
    AudioDevice * alsa_dev = nullptr;
    auto open_result = audio_open(hw_vals, sw_vals, alsa_dev);
    if (open_result.has_value()) {
      RCLCPP_ERROR(rcl_logger, "Failed to open audio device for file play: %s",
        open_result->c_str());
      return;
    }

        // Disable normalization since we are handling scaling ourselves.
    fileh.command(SFC_SET_NORM_FLOAT, NULL, SF_FALSE);
    fileh.command(SFC_SET_NORM_DOUBLE, NULL, SF_FALSE);

    auto err = audio_play(fileh, alsa_dev, hw_vals.format, &shutdown_flag);
    if (err) {
      RCLCPP_ERROR(rcl_logger, "Audio play error: %s", err->c_str());
    }
    audio_drain(alsa_dev);
    audio_close(alsa_dev);
    return;
  }
};

//#define ALSA_PERIOD_SIZE 1024
//#define ALSA_BUFFER_PERIODS 4
//#define ALSA_START_PERIODS 2

int main(int argc, char ** argv)
{
  std::signal(SIGINT, signal_handler);   // Register the signal handler
  rclcpp::init(argc, argv);

  auto file_player = std::make_shared<FilePlayerNode>();
    // Enqueue all files from command line arguments
  file_player->play_file_data(argv[1]);
  rclcpp::spin(file_player);
  rclcpp::shutdown();
  return 0;
}
