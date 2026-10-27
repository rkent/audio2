#ifndef AUDIO2_STREAM_AUDIOOPS_HPP
#define AUDIO2_STREAM_AUDIOOPS_HPP

#include "audio2_stream/config.hpp"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <tuple>
#include <optional>
#include <string>
#include <vector>
#include <chrono>

// utility function to format timestamps for debugging
inline std::string format_timestamp()
{
  auto time_point = std::chrono::steady_clock::now();
  auto time_since_epoch = time_point.time_since_epoch();
  auto hours = std::chrono::duration_cast<std::chrono::hours>(time_since_epoch) % 24;
  auto minutes = std::chrono::duration_cast<std::chrono::minutes>(time_since_epoch) % 60;
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch) % 60;
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch) %
    1000;

  char buffer[256];
  snprintf(buffer, sizeof(buffer), "%02lld:%02lld:%02lld:%03lld",
    static_cast<long long>(hours.count()), static_cast<long long>(minutes.count()),
    static_cast<long long>(seconds.count()), static_cast<long long>(milliseconds.count()));
  return std::string(buffer);
}

// Opaque handle to an open audio device stream (implemented with RtAudio)
struct AudioDevice;

// Audio device hardware parameters values
typedef struct AudioHwParams
{
  const char *device = ALSA_DEVICE_NAME;
  unsigned int channels = 1;
  unsigned long buffer_size = ALSA_BUFFER_SIZE;
  unsigned long period_size = ALSA_PERIOD_SIZE;
  AudioFormat format = ALSA_FORMAT;
  unsigned int samplerate = ALSA_SAMPLERATE;
  AudioDirection direction = AUDIO_PLAYBACK;
  bool operator!=(const AudioHwParams & p_rhs) const
  {
    return std::tie(
                channels,
                buffer_size,
                period_size,
                format,
                samplerate,
                direction) !=
           std::tie(
                p_rhs.channels,
                p_rhs.buffer_size,
                p_rhs.period_size,
                p_rhs.format,
                p_rhs.samplerate,
                p_rhs.direction) ||
           strncmp(device, p_rhs.device, 256) != 0;
  }
} AudioHwParams;

// Audio device software parameters values
typedef struct AudioSwParams
{
  unsigned long start_threshold = ALSA_START_THRESHOLD;

  bool operator!=(const AudioSwParams & p_rhs) const
  {
    return start_threshold != p_rhs.start_threshold;
  }
} AudioSwParams;

// Function declarations

/**
 * Sample width in bits of an AudioFormat, or -1 if unknown.
 */
int audio_format_width(AudioFormat format);

/**
 * Debug string of an AudioFormat.
 */
const char * audio_format_to_string(AudioFormat format);

/**
 * Names of the devices available for a direction, for diagnostics.
 */
std::vector<std::string> audio_device_names(AudioDirection direction);

/**
 * Open an audio device with specified hardware and software parameters.
 * \param hw_vals  Hardware parameters for the device. samplerate and period_size are
 *                 updated to the values actually used.
 * \param sw_vals  Software parameters for the device.
 * \param device   Pointer to the opened device, or nullptr on failure.
 * \return         Optional error string if opening fails.
 */
std::optional<std::string>
audio_open(AudioHwParams & hw_vals, AudioSwParams & sw_vals, AudioDevice *& device);

/**
 * Block until all buffered playback data has been played, then stop the device.
 */
void audio_drain(AudioDevice * audio_dev);

/**
 * Stop and close the device, and set audio_dev to nullptr.
 */
void audio_close(AudioDevice *& audio_dev);

/**
 * Write audio data to an audio device, blocking until it has all been buffered.
 * \param samples   Number of samples to write.
 * \param audio_dev Pointer to the audio device.
 * \param data      Pointer to the audio data buffer.
 * \param channels  Number of audio channels.
 * \param format    Format of the audio data, which must match the opened device.
 * \param shutdown_flag Pointer to atomic boolean to signal shutdown.
 * \return          Number of samples written, or negative error code.
 */
int audio_write(
  int samples, AudioDevice * audio_dev, const void * data, int channels,
  AudioFormat format, std::atomic<bool> * shutdown_flag);

/**
 * Read audio data from an audio device, blocking until samples have been read.
 * \return          Number of samples read, or negative error code.
 */
int audio_read(
  int samples, AudioDevice * audio_dev, void * data, int channels,
  AudioFormat format, std::atomic<bool> * shutdown_flag);

#endif
