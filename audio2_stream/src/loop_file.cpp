/**
 * @file loop_file.cpp
 * @brief Test file read, loop through snd_file buffers and play with RtAudio.
 */

#include <atomic>
#include <csignal>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <RtAudio.h>
#include <sndfile.h>
#include <sndfile.hh>

#include "rclcpp/rclcpp.hpp"
#include "audio2_stream/ra_ops.hpp"

#define TOPIC_FORMAT (SF_FORMAT_WAV | SF_FORMAT_PCM_32)
//#define TOPIC_FORMAT (SF_FORMAT_OGG | SF_FORMAT_VORBIS)
//#define TOPIC_FORMAT (SF_FORMAT_OGG | SF_FORMAT_OPUS)
//#define TOPIC_FORMAT (SF_FORMAT_MPEG | SF_FORMAT_MPEG_LAYER_III)

// Global flag to signal thread shutdown
std::atomic<bool> shutdown_flag(false);
// Global flag to signify that new data is available in the queue
std::atomic<bool> data_available(false);

static auto rcl_logger = rclcpp::get_logger("audio2_stream/loop_file");

void signal_handler(int signal)
{
  if (signal == SIGINT) {
    RCLCPP_INFO(rcl_logger, "Keyboard interrupt received. Shutting down.");
    shutdown_flag.store(true);
    rclcpp::shutdown();
  }
}

class FileLooperNode : public rclcpp::Node
{
public:
  FileLooperNode()
  : Node("file_looper")
  {
  }

  void publish_file_data(const std::string & file_path)
  {
    // Open the sound file
    RCLCPP_INFO(rcl_logger, "Streaming audio data from file: %s", file_path.c_str());
    fileh_ = SndfileHandle(file_path);
    if (fileh_.error()) {
      RCLCPP_ERROR(rcl_logger, "Cannot open file <%s>: %s", file_path.c_str(), fileh_.strError());
      return;
    }
    RCLCPP_INFO(
      rcl_logger, "File opened: %s, Sample Rate: %d, Format 0x%X, Channels: %d",
      file_path.c_str(), fileh_.samplerate(), fileh_.format(), fileh_.channels());

    // Disable normalization since we are handling scaling ourselves.
    fileh_.command(SFC_SET_NORM_FLOAT, NULL, SF_FALSE);
    fileh_.command(SFC_SET_NORM_DOUBLE, NULL, SF_FALSE);

    // Initialize lock-free queue and continuous RtAudio writer thread
    const int queue_capacity = 100;
    boost::lockfree::spsc_queue<std::vector<uint8_t>> audio_queue(queue_capacity);

    RaWriteThread writer(
      static_cast<unsigned int>(fileh_.channels()),
      static_cast<unsigned int>(fileh_.samplerate()),
      RTAUDIO_FLOAT32,
      &audio_queue,
      &shutdown_flag,
      &data_available);

    if (!writer.get_error().empty()) {
      RCLCPP_ERROR(rcl_logger, "Failed to start audio writer: %s", writer.get_error().c_str());
      return;
    }

    SfgRwFormat file_rw_format = sfg_format_from_sndfile_format(fileh_.format());
    int file_sample_size = sample_size_from_sfg_format(file_rw_format);
    const int MAX_HEADER = 128;
    const int read_frames = 1024 * 16;
    int file_buffer_size = read_frames * fileh_.channels() * file_sample_size;
    std::vector<char> file_buffer(file_buffer_size);

    SfgRwFormat topic_rw_format = sfg_format_from_sndfile_format(TOPIC_FORMAT);
    int topic_sample_size = sample_size_from_sfg_format(topic_rw_format);
    int topic_file_size = read_frames * fileh_.channels() * topic_sample_size + MAX_HEADER;
    std::vector<char> topic_buffer(topic_file_size);

    int samples_read = 0;
    do {
      samples_read = sfg_read(
        fileh_, file_rw_format, file_buffer.data(),
        read_frames * fileh_.channels());
      RCLCPP_INFO(
        rcl_logger, "Read %d samples from file using format %s", samples_read,
        sfg_format_to_string(file_rw_format));
      if (samples_read <= 0) {
        if (samples_read < 0) {
          RCLCPP_ERROR(
            rcl_logger, "Error reading from file: %s %d", fileh_.strError(),
            samples_read);
        }
        break;
      }

      // At this point, we have read samples_read samples into file_buffer
      // Create a virtual sound file in memory for topic publish
      RCLCPP_INFO(rcl_logger, "Topic publish format: %s", format_to_string(TOPIC_FORMAT).c_str());

      VIO_SOUNDFILE_HANDLE tw_vio_sndfileh;
      tw_vio_sndfileh.vio_data.data = topic_buffer.data();
      tw_vio_sndfileh.vio_data.length = 0;
      tw_vio_sndfileh.vio_data.offset = 0;
      tw_vio_sndfileh.vio_data.capacity = static_cast<sf_count_t>(topic_buffer.size());
      if (auto err = open_sndfile_from_buffer(
          tw_vio_sndfileh, SFM_WRITE, TOPIC_FORMAT,
          fileh_.channels(), fileh_.samplerate()))
      {
        RCLCPP_ERROR(
          rcl_logger, "Failed to open sound file for writing to buffer: %s",
          err->c_str());
        return;
      }

      int samples_written = sfg_write_convert(
        tw_vio_sndfileh.fileh, file_rw_format,
        topic_rw_format, file_buffer.data(), samples_read);
      if (samples_written != samples_read) {
        RCLCPP_ERROR(
          rcl_logger, "samples_written %d does not match expected %d", samples_written,
          samples_read);
        break;
      }
      RCLCPP_INFO(
        rcl_logger, "Wrote %d samples to virtual topic buffer using format %s", samples_written,
        sfg_format_to_string(topic_rw_format));

      // Reopen the virtual file for reading and decode into playback queue
      VIO_SOUNDFILE_HANDLE tr_handle;
      tr_handle.vio_data.data = topic_buffer.data();
      tr_handle.vio_data.length = tw_vio_sndfileh.vio_data.length;
      tr_handle.vio_data.offset = 0;
      tr_handle.vio_data.capacity = static_cast<sf_count_t>(topic_buffer.size());

      if (auto err = open_sndfile_from_buffer(tr_handle, SFM_READ)) {
        RCLCPP_ERROR(
          rcl_logger, "Failed to open sound file for reading from buffer: %s",
          err->c_str());
        return;
      }

      // Decode into float samples (RTAUDIO_FLOAT32) and push to the continuous playback queue
      std::vector<uint8_t> pcm_chunk(samples_written * sizeof(float));
      float * float_buffer = reinterpret_cast<float *>(pcm_chunk.data());
      sf_count_t frames_decoded = tr_handle.fileh.readf(
        float_buffer,
        samples_written / fileh_.channels());

      if (frames_decoded > 0) {
        pcm_chunk.resize(frames_decoded * fileh_.channels() * sizeof(float));

        // Push to lockfree queue, backpressuring if full
        while (!audio_queue.push(pcm_chunk) && !shutdown_flag.load() && rclcpp::ok()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        data_available.store(true);
      }

    } while (!shutdown_flag.load());

    // Drain remaining audio before closing
    writer.drain_and_close();
    return;
  }

private:
  SndfileHandle fileh_;
};

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: loop_file <file_path>\n";
    return 1;
  }

  std::signal(SIGINT, signal_handler);   // Register the signal handler
  rclcpp::init(argc, argv);

  auto file_publisher = std::make_shared<FileLooperNode>();
  file_publisher->publish_file_data(argv[1]);
  rclcpp::shutdown();
  return 0;
}
