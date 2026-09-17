#ifndef AUDIO2_STREAM_RA_OPS_HPP
#define AUDIO2_STREAM_RA_OPS_HPP

#include "audio2_stream/buffer_file.hpp"
#include <RtAudio.h>
#include <sndfile.hh>
#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "boost/lockfree/spsc_queue.hpp"

/**
 * Convert RtAudio format to human-readable string.
 * \param format The RtAudioFormat integer.
 * \return       A string representing the format.
 */
std::string format_to_string(RtAudioFormat format);

/**
 * Get the sample size in bytes for a given RtAudio format.
 * \param format The RtAudio format.
 * \return       The sample size in bytes, or -1 if unsupported.
 */
int sample_size_from_rtaudio_format(RtAudioFormat format);

/**
 * Read/write type to use for different RtAudio formats
 * \param ra_format The RtAudioFormat enum value.
 * \return          Corresponding SfgRwFormat enum value.
 */
SfgRwFormat sfg_format_from_rtaudio_format(RtAudioFormat ra_format);

/**
 * Convert SfgRwFormat enum to RtAudioFormat
 * \param sfg_format The SfgRwFormat enum value.
 * \return           Corresponding RtAudioFormat value.
 */
RtAudioFormat rtaudio_format_from_sfg_format(SfgRwFormat sfg_format);

/**
 * ra_play: Play audio from a SNDFILE using RtAudio
 * @param fileh         SndfileHandle to read audio data from
 * @param dac           Reference to RtAudio instance
 * @param ra_format     RtAudio format to use for playback
 * @param shutdown_flag optional atomic boolean flag to signal shutdown
 * @return Optional error message string if an error occurs, std::nullopt on success
 */
std::optional<std::string>
ra_play(
  SndfileHandle fileh, RtAudio & dac, RtAudioFormat ra_format = RTAUDIO_FLOAT32,
  std::atomic<bool> * shutdown_flag = nullptr);

/**
 * ra_play: Play audio from a SNDFILE using RtAudio (creates a local RtAudio instance)
 * @param fileh         SndfileHandle to read audio data from
 * @param ra_format     RtAudio format to use for playback
 * @param shutdown_flag optional atomic boolean flag to signal shutdown
 * @return Optional error message string if an error occurs, std::nullopt on success
 */
std::optional<std::string>
ra_play(
  SndfileHandle fileh, RtAudioFormat ra_format = RTAUDIO_FLOAT32,
  std::atomic<bool> * shutdown_flag = nullptr);

/**
 * Thread-safe RtAudio writer class that streams continuous audio from a lock-free queue.
 */
class RaWriteThread
{
public:
  RaWriteThread(
    unsigned int channels,
    unsigned int sample_rate,
    RtAudioFormat format,
    boost::lockfree::spsc_queue<std::vector<uint8_t>> * queue,
    std::atomic<bool> * shutdown_flag = nullptr,
    std::atomic<bool> * data_available = nullptr,
    unsigned int buffer_frames = 512);

  ~RaWriteThread();

  std::string get_error() const;
  bool is_open() const;
  void close();
  void drain_and_close();

private:
  static int audio_callback(
    void * output_buffer,
    void * input_buffer,
    unsigned int n_buffer_frames,
    double stream_time,
    RtAudioStreamStatus status,
    void * user_data);

  std::unique_ptr<RtAudio> dac_;
  unsigned int channels_{2};
  unsigned int sample_rate_{48000};
  RtAudioFormat format_{RTAUDIO_FLOAT32};
  int sample_bytes_{4};
  boost::lockfree::spsc_queue<std::vector<uint8_t>> * queue_{nullptr};
  std::atomic<bool> * shutdown_flag_{nullptr};
  std::atomic<bool> * data_available_{nullptr};
  std::vector<uint8_t> current_chunk_;
  size_t current_chunk_offset_{0};
  std::string error_str_;
};

#endif // AUDIO2_STREAM_RA_OPS_HPP
