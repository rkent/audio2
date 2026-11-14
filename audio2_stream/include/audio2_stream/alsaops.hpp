#ifndef SNDPLAY_ALSAOPS_HPP
#define SNDPLAY_ALSAOPS_HPP

// ALSA device operations. Linux only: neither this header nor alsaops.cpp is part of the
// Windows build, where audio devices use WasapiProxyImpl instead.

#include "audio2_stream/config.hpp"
#include "audio2_stream/visibility_control.hpp"
#include "audio2_stream/timestamp.hpp"

#include <alsa/asoundlib.h>
#include <sndfile.hh>

#include <atomic>
#include <cstring>
#include <limits>
#include <tuple>
#include <optional>
#include <string>

// ALSA software parameter defaults
const int ALSA_START_PERIODS = DEVICE_BUFFER_PERIODS / 2;
const long unsigned int ALSA_START_THRESHOLD = DEVICE_PERIOD_SIZE * ALSA_START_PERIODS;
// Never automatically stop the alsa device
const long unsigned int ALSA_STOP_THRESHOLD = std::numeric_limits<snd_pcm_uframes_t>::max();
// settings for "samples are zeroed after they are played back"
const long unsigned int ALSA_SILENCE_SIZE = std::numeric_limits<snd_pcm_uframes_t>::max();
const long unsigned int ALSA_SILENCE_THRESHOLD = 0;

// SampleFormat values are ALSA's, so conversion is a cast. Keep them in sync.
static_assert(static_cast<int>(SampleFormat::Unknown) == SND_PCM_FORMAT_UNKNOWN);
static_assert(static_cast<int>(SampleFormat::S8) == SND_PCM_FORMAT_S8);
static_assert(static_cast<int>(SampleFormat::U8) == SND_PCM_FORMAT_U8);
static_assert(static_cast<int>(SampleFormat::S16) == SND_PCM_FORMAT_S16_LE);
static_assert(static_cast<int>(SampleFormat::U16) == SND_PCM_FORMAT_U16_LE);
static_assert(static_cast<int>(SampleFormat::S24) == SND_PCM_FORMAT_S24_LE);
static_assert(static_cast<int>(SampleFormat::U24) == SND_PCM_FORMAT_U24_LE);
static_assert(static_cast<int>(SampleFormat::S32) == SND_PCM_FORMAT_S32_LE);
static_assert(static_cast<int>(SampleFormat::U32) == SND_PCM_FORMAT_U32_LE);
static_assert(static_cast<int>(SampleFormat::Float) == SND_PCM_FORMAT_FLOAT_LE);
static_assert(static_cast<int>(SampleFormat::Float64) == SND_PCM_FORMAT_FLOAT64_LE);

constexpr snd_pcm_format_t to_alsa_format(SampleFormat format)
{
  return static_cast<snd_pcm_format_t>(format);
}

constexpr SampleFormat from_alsa_format(snd_pcm_format_t format)
{
  return static_cast<SampleFormat>(format);
}

// ALSA hardware parameters values
typedef struct AlsaHwParams
{
  const char *device = DEFAULT_DEVICE_NAME;
  unsigned int channels = 1;
  snd_pcm_uframes_t buffer_size = DEVICE_BUFFER_SIZE;
  snd_pcm_uframes_t period_size = DEVICE_PERIOD_SIZE;
  snd_pcm_access_t access = SND_PCM_ACCESS_RW_INTERLEAVED;
  snd_pcm_format_t format = to_alsa_format(DEFAULT_SAMPLE_FORMAT);
  unsigned int samplerate = DEFAULT_SAMPLERATE;
  snd_pcm_stream_t direction = SND_PCM_STREAM_PLAYBACK;
  bool operator!=(const AlsaHwParams & p_rhs) const
  {
    return std::tie(
                channels,
                buffer_size,
                period_size,
                access,
                format,
                samplerate) !=
           std::tie(
                p_rhs.channels,
                p_rhs.buffer_size,
                p_rhs.period_size,
                p_rhs.access,
                p_rhs.format,
                p_rhs.samplerate) ||
           strncmp(device, p_rhs.device, 256) != 0;
  }
} AlsaHwParams;

// ALSA software parameters values
typedef struct AlsaSwParams
{
  snd_pcm_uframes_t start_threshold = ALSA_START_THRESHOLD;
  snd_pcm_uframes_t stop_threshold = ALSA_STOP_THRESHOLD;
  snd_pcm_uframes_t silence_size = ALSA_SILENCE_SIZE;
  snd_pcm_uframes_t silence_threshold = ALSA_SILENCE_THRESHOLD;

  bool operator!=(const AlsaSwParams & p_rhs) const
  {
    return std::tie(
                start_threshold,
                stop_threshold,
                silence_threshold,
                silence_size) !=
           std::tie(
                p_rhs.start_threshold,
                p_rhs.stop_threshold,
                p_rhs.silence_threshold,
                p_rhs.silence_size);
  }
} AlsaSwParams;

// Function declarations

/**
 * Open an ALSA PCM device with specified hardware and software parameters.
 * \param hw_vals  Hardware parameters for the ALSA device.
 * \param sw_vals  Software parameters for the ALSA device.
 * \param device   Pointer to the opened ALSA PCM device, or nullptr on failure.
 * \return         Optional error string if opening fails.
 */
AUDIO2_STREAM_PUBLIC
std::optional<std::string>
alsa_open(AlsaHwParams & hw_vals, AlsaSwParams & sw_vals, snd_pcm_t *& device);

/**
 * Write audio data to ALSA device.
 * \param samples Number of samples to write.
 * \param alsa_dev  Pointer to the ALSA PCM device.
 * \param data      Pointer to the audio data buffer.
 * \param channels  Number of audio channels.
 * \param alsa_format ALSA format of the audio data.
 * \param shutdown_flag Pointer to atomic boolean to signal shutdown.
 * \param epipe_count Score of recent xruns on this stream, owned by the caller and starting at 0.
 *                    It decays by 1 per call and rises by 100 per xrun; a further xrun while it
 *                    is above 140 makes the call give up and return -EPIPE.
 * \return          Number of samples written, or negative error code.
 */
AUDIO2_STREAM_PUBLIC
int alsa_write(
  int samples, snd_pcm_t * alsa_dev, void * data, int channels,
  snd_pcm_format_t alsa_format, std::atomic<bool> * shutdown_flag, int & epipe_count);

/**
 * Read audio data from an ALSA device. Parameters are as for alsa_write.
 * \return Number of samples read, or negative error code.
 */
AUDIO2_STREAM_PUBLIC
int alsa_read(
  int samples, snd_pcm_t * alsa_dev, void * data, int channels,
  snd_pcm_format_t alsa_format, std::atomic<bool> * shutdown_flag, int & epipe_count);

/**
 * alsa_play: Play audio from a SNDFILE using ALSA
 * @param fileh SndfileHandle to read audio data from
 * @param alsa_dev Pointer to the opened ALSA PCM device
 * @param alsa_format ALSA format to use for playback
 * @param shutdown_flag optional atomic boolean flag to signal shutdown
 * @return Optional error message string if an error occurs, std::nullopt on success
 */
AUDIO2_STREAM_PUBLIC
std::optional<std::string>
alsa_play(
  SndfileHandle fileh, snd_pcm_t * alsa_dev, snd_pcm_format_t alsa_format,
  std::atomic<bool> * shutdown_flag);

#endif
