#ifndef SNDPLAY_BUFFER_FILE_HPP
#define SNDPLAY_BUFFER_FILE_HPP

#include "audio2_stream/config.hpp"
#include <sndfile.hh>
#include <chrono>
#include <thread>
#include <cstring>
#include <vector>
#include <memory>
#include <atomic>
#include <optional>
#include "boost/lockfree/spsc_queue.hpp"

#include "audio2_stream/alsaops.hpp"

/**
 * Convert sndfile format to human-readable string.
 * \param format The sndfile format integer.
 * \return       A string representing the format.
 */
std::string format_to_string(int format);

// Virtual I/O context for reading/writing from/to memory
typedef struct
{
  char *data = nullptr;
  sf_count_t length = 0;
  sf_count_t offset = 0;
  sf_count_t capacity = 0;
} VIO_DATA;

typedef struct
{
  SndfileHandle fileh;
  VIO_DATA vio_data;
} VIO_SOUNDFILE_HANDLE;

/**
 * \param vio_sndfileh The VIO_SOUNDFILE_HANDLE to open.
 * \param mode         The mode to open the file in (SFM_READ, SFM_WRITE).
 * \param format       The sndfile format (only for write mode).
 * \param channels     The number of channels (only for write mode).
 * \param samplerate   The sample rate (only for write mode).
 */
std::optional<std::string>
open_sndfile_from_buffer(
  VIO_SOUNDFILE_HANDLE & vio_sndfileh, int mode = SFM_READ,
  int format = 0, int channels = 0, int samplerate = 0);

/**
 * Read samples from a SNDFILE into a buffer.
 * \param sndfile The SndfileHandle to read from.
 * \param format  The buffer format.
 * \param buffer  The buffer to read samples into.
 * \param samples The number of samples to read.
 * \return        The number of samples read, or a negative error code.
 */
int sfg_read(SndfileHandle & sndfileh, SfgRwFormat format, void * buffer, int samples);

/**
 * Write samples from a buffer to a SNDFILE.
 * \param sndfile The SNDFILE to write to
 * \param format  The buffer format
 * \param buffer  The buffer containing samples to write
 * \param samples The number of samples to write
 */
int sfg_write(SNDFILE * sndfile, SfgRwFormat format, void * buffer, int samples);

/**
 * Get the sample size in bytes for a given ALSA format.
 * \param format The ALSA format.
 * \return       The sample size in bytes, or -1 if unsupported.
 */
int sample_size_from_alsa_format(snd_pcm_format_t format);

/**
 * Get sample size from our SfgRwFormat enum
 * \param format The SfgRwFormat enum value.
 * \return       The sample size in bytes.
 */
int sample_size_from_sfg_format(SfgRwFormat format);

/**
 * Read/write type to use for different sndfile formats
 * \param sf_format The sndfile format integer.
 * \return          Corresponding SfgRwFormat enum value.
 */
SfgRwFormat sfg_format_from_sndfile_format(int sf_format);

/**
 * Read/write type to use for different ALSA formats
 * \param alsa_format The ALSA format enum value.
 * \return            Corresponding SfgRwFormat enum value.
 */
SfgRwFormat sfg_format_from_alsa_format(snd_pcm_format_t alsa_format);

/**
 * String representation of SfgRwFormat
 * \param format The SfgRwFormat enum value.
 * \return       A string representing the format.
 */
const char * sfg_format_to_string(SfgRwFormat format);

/**
 * Convert between different sample formats.
 * \param from_format The source format.
 * \param to_format   The destination format.
 * \param in_buffer   Pointer to the input buffer.
 * \param out_buffer  Pointer to the output buffer.
 * \param samples     Number of samples to convert.
 * \return            Number of samples converted, or negative error code.
 */
int convert_types(
  SfgRwFormat from_format, SfgRwFormat to_format, const void * in_buffer,
  void * out_buffer, int samples);

/**
 * Write samples from a buffer to a SNDFILE with scaling and conversion.
 * \param sndfile The SNDFILE to write to.
 * \param from_format The source format.
 * \param to_format   The destination format.
 * \param buffer  The buffer containing samples to write.
 * \param samples The number of samples to write.
 * \return        The number of samples written, or a negative error code.
 */
int sfg_write_convert(
  SndfileHandle & fileh, SfgRwFormat from_format, SfgRwFormat to_format,
  char * buffer, int samples);

/**
 * alsa_play: Play audio from a SNDFILE using ALSA
 * @param fileh SndfileHandle to read audio data from
 * @param alsa_dev Pointer to the opened ALSA PCM device
 * @param alsa_format ALSA format to use for playback
 * @param shutdown_flag optional atomic boolean flag to signal shutdown
 * @return Optional error message string if an error occurs, std::nullopt on success
 */
std::optional<std::string>
alsa_play(
  SndfileHandle fileh, snd_pcm_t * alsa_dev, snd_pcm_format_t alsa_format,
  std::atomic<bool> * shutdown_flag);

/**
 * Create read and write buffers for format conversion.
 * \param from_format The source format.
 * \param to_format   The destination format.
 * \param samples     Number of samples to convert.
 * \param r_buffer    Reference to the read buffer vector.
 * \param w_buffer    Reference to the write buffer vector.
 */
void create_convert_vectors(
  SfgRwFormat from_format, SfgRwFormat to_format, int samples,
  std::vector<uint8_t> & r_buffer, std::vector<uint8_t> & w_buffer);

/**
 * Open a virtual sound file for reading from a vector buffer.
 *
 * \param vector       The vector containing the audio data.
 * \param vio_sndfileh The VIO_SOUNDFILE_HANDLE to initialize.
 * \return             Optional error string if an error occurs.
 */
std::optional<std::string>
ropen_vio_from_vector(
  const std::vector<unsigned char> & vector,
  VIO_SOUNDFILE_HANDLE & vio_sndfileh);

/**
 * Open a virtual sound file for writing to a vector buffer.
 *
 * \param vector       The vector to contain the audio data.
 * \param vio_sndfileh The VIO_SOUNDFILE_HANDLE to initialize.
 * \param sfg_format   The SfgRwFormat to use (e.g. SFG_SHORT).
 * \param sf_format    The sndfile format to use (e.g. SF_FORMAT_WAV | SF_FORMAT_PCM_16).
 * \param channels     Number of audio channels.
 * \param sample_rate  Sample rate of the audio data in frames per second.
 * \param frames       Number of frames to allocate space for.
 * \return             Optional error string if an error occurs.
 */
std::optional<std::string>
wopen_vio_to_vector(
  std::vector<unsigned char> & vector,
  VIO_SOUNDFILE_HANDLE & vio_sndfileh,
  SfgRwFormat sfg_format,
  int sf_format,
  int channels,
  int sample_rate,
  int frames);

#endif // SNDPLAY_BUFFER_FILE_HPP
