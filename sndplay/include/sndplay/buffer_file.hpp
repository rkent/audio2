#ifndef SNDPLAY_BUFFER_FILE_HPP
#define SNDPLAY_BUFFER_FILE_HPP
#include <sndfile.h>
#include <vector>
#include <memory>
#include <atomic>
#include <optional>
#include "boost/lockfree/spsc_queue.hpp"

#include "sndplay/alsaops.hpp"

/**
 * Convert sndfile format to human-readable string.
 * \param format The sndfile format integer.
 * \return       A string representing the format.
 */
std::string format_to_string(int format);

// Virtual I/O context for reading/writing from/to memory
typedef struct {
    const unsigned char *data = nullptr;
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
    std::shared_ptr<std::vector<unsigned char>> file_data;
    AlsaHwParams hw_vals;
    AlsaSwParams sw_vals;
} PlayBufferParams;

std::optional<std::string> open_sndfile_from_buffer(VIO_SOUNDFILE & vio_sndfile, int mode);

// int write_buffer(void* buffer, int format, int frames, int channels, int sample_rate);

/**
 * Read an entire binary file into memory.
 * \param file_path The path to the file to read.
 * \param out_data  Shared pointer to store the file data.
 * \return          Optional error string if an error occurs.
*/
std::optional<std::string> get_file(const char * file_path, std::shared_ptr<std::vector<unsigned char>> & out_data);

void play_buffer_thread(boost::lockfree::spsc_queue<PlayBufferParams>* audio_queue, std::atomic<bool>* shutdown_flag, std::atomic<bool>* data_available);

/**
 * Read samples from a SNDFILE into a buffer.
 * \param sndfile The SNDFILE to read from.
 * \param format  The buffer format using ALSA format enums.
 * \param buffer  The buffer to read samples into.
 * \param samples The number of samples to read.
 * \return        The number of samples read, or a negative error code.
 */
int sfg_read(SNDFILE * sndfile, snd_pcm_format_t format, void * buffer, int samples);

/**
 * Get the sample size in bytes for a given ALSA format.
 * \param format The ALSA format.
 * \return       The sample size in bytes, or -1 if unsupported.
 */
int sample_size_from_format(snd_pcm_format_t format);

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

#endif // SNDPLAY_BUFFER_FILE_HPP
