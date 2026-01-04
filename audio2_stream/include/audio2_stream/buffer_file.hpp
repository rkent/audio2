#ifndef SNDPLAY_BUFFER_FILE_HPP
#define SNDPLAY_BUFFER_FILE_HPP
#include <sndfile.hh>
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
typedef struct {
    char *data = nullptr;
    sf_count_t length = 0;
    sf_count_t offset = 0;
    sf_count_t capacity = 0;
} VIO_DATA;

typedef struct {
    VIO_DATA vio_data;
    SNDFILE * sndfile = nullptr;
    SF_INFO sfinfo;
} VIO_SOUNDFILE;

typedef struct {
    SndfileHandle fileh;
    VIO_DATA vio_data;
} VIO_SOUNDFILE_HANDLE;

typedef struct {
    std::shared_ptr<std::vector<char>> file_data;
    AlsaHwParams hw_vals;
    AlsaSwParams sw_vals;
} PlayBufferParams;

typedef enum {
    SFG_INVALID = -1,
    SFG_BYTE,
    SFG_SHORT,
    SFG_INT,
    SFG_FLOAT,
    SFG_DOUBLE
} SfgRwFormat;

std::optional<std::string> open_sndfile_from_buffer(VIO_SOUNDFILE & vio_sndfile, int mode);

std::optional<std::string> open_sndfile_from_buffer2(VIO_SOUNDFILE_HANDLE & vio_sndfileh, int mode = SFM_READ,
    int format = 0, int channels = 0, int samplerate = 0);

// int write_buffer(void* buffer, int format, int frames, int channels, int sample_rate);

/**
 * Read an entire binary file into memory.
 * \param file_path The path to the file to read.
 * \param out_data  Shared pointer to store the file data.
 * \return          Optional error string if an error occurs.
*/
std::optional<std::string> get_file(const char * file_path, std::shared_ptr<std::vector<char>> & out_data);

void play_buffer_thread(boost::lockfree::spsc_queue<PlayBufferParams>* audio_queue, std::atomic<bool>* shutdown_flag, std::atomic<bool>* data_available);

/**
 * Read samples from a SNDFILE into a buffer.
 * \param sndfile The SNDFILE to read from.
 * \param format  The buffer format.
 * \param buffer  The buffer to read samples into.
 * \param samples The number of samples to read.
 * \return        The number of samples read, or a negative error code.
 */
int sfg_read(SNDFILE * sndfile, SfgRwFormat format, void * buffer, int samples);

/**
 * Read samples from a SNDFILE into a buffer.
 * \param sndfile The SndfileHandle to read from.
 * \param format  The buffer format.
 * \param buffer  The buffer to read samples into.
 * \param samples The number of samples to read.
 * \return        The number of samples read, or a negative error code.
 */
int sfg_read2(SndfileHandle& sndfileh, SfgRwFormat format, void * buffer, int samples);

/**
 * Write samples from a buffer to a SNDFILE.
 * \param sndfile The SNDFILE to write to
 * \param format  The buffer format
 * \param buffer  The buffer containing samples to write
 * \param samples The number of samples to write
 */
int sfg_write2(SNDFILE * sndfile, SfgRwFormat format, void * buffer, int samples);

/**
 * Get the sample size in bytes for a given ALSA format.
 * \param format The ALSA format.
 * \return       The sample size in bytes, or -1 if unsupported.
 */
int sample_size_from_format(snd_pcm_format_t format);

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
int convert_types(SfgRwFormat from_format, SfgRwFormat to_format, const void* in_buffer, void* out_buffer, int samples);

/**
 * Write samples from a buffer to a SNDFILE.
 * \param sndfile The SNDFILE to write to.
 * \param buffer  The buffer containing samples to write.
 * \param format  The buffer format using ALSA format enums.
 * \param samples The number of samples to write.
 * \return        The number of samples written, or a negative error code.
 */
int sfg_write(SNDFILE * sndfile, void * buffer, snd_pcm_format_t format, int samples);

/**
 * Write samples from a buffer to a SNDFILE with scaling and conversion.
 * \param sndfile The SNDFILE to write to.
 * \param from_format The source format.
 * \param to_format   The destination format.
 * \param buffer  The buffer containing samples to write.
 * \param samples The number of samples to write.
 * \return        The number of samples written, or a negative error code.
 */
int sfg_write_convert(SNDFILE * sndfile, SfgRwFormat from_format, SfgRwFormat to_format, const char * buffer, int samples);

/**
 * alsa_play: Play audio from a SNDFILE using ALSA
 * @param sndfile Pointer to the SNDFILE to read audio data from
 * @param sfinfo SF_INFO structure containing information about the audio file
 * @param alsa_dev Pointer to the opened ALSA PCM device
 * @param alsa_format ALSA format to use for playback
 * @param shutdown_flag optional atomic boolean flag to signal shutdown
 * @return Optional error message string if an error occurs, std::nullopt on success
 */
std::optional<std::string>
alsa_play (SNDFILE *sndfile, SF_INFO sfinfo, snd_pcm_t* alsa_dev, snd_pcm_format_t alsa_format, std::atomic<bool>* shutdown_flag) ;

std::optional<std::string>
alsa_play (SNDFILE *sndfile, int format, int channels, snd_pcm_t* alsa_dev, snd_pcm_format_t alsa_format, std::atomic<bool>* shutdown_flag);

std::optional<std::string>
alsa_play (SndfileHandle fileh, snd_pcm_t* alsa_dev, snd_pcm_format_t alsa_format, std::atomic<bool>* shutdown_flag);

#endif // SNDPLAY_BUFFER_FILE_HPP
