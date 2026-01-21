#ifndef SNDPLAY_ALSAOPS_HPP
#define SNDPLAY_ALSAOPS_HPP

#include "audio2_stream/config.hpp"
#include <alsa/asoundlib.h>

#include <atomic>
#include <tuple>
#include <optional>
#include <string>
#include <chrono>
#include "boost/lockfree/spsc_queue.hpp"

// utility function to format timestamps for debugging
inline std::string format_timestamp() {
    auto time_point = std::chrono::steady_clock::now();
    auto time_since_epoch = time_point.time_since_epoch();
    auto hours = std::chrono::duration_cast<std::chrono::hours>(time_since_epoch) % 24;
    auto minutes = std::chrono::duration_cast<std::chrono::minutes>(time_since_epoch) % 60;
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch) % 60;
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch) % 1000;
    
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "%02ld:%02ld:%02ld:%03ld",
                hours.count(), minutes.count(), seconds.count(), milliseconds.count());
    return std::string(buffer);
}

// ALSA hardware parameters values
typedef struct AlsaHwParams {
    const char *device = ALSA_DEVICE_NAME;
    unsigned int channels = 1;
    snd_pcm_uframes_t buffer_size = ALSA_BUFFER_SIZE;
    snd_pcm_uframes_t period_size = ALSA_PERIOD_SIZE;
    snd_pcm_access_t access = SND_PCM_ACCESS_RW_INTERLEAVED;
    snd_pcm_format_t format = ALSA_FORMAT;
    unsigned int samplerate = ALSA_SAMPLERATE;
    snd_pcm_stream_t direction = SND_PCM_STREAM_PLAYBACK;
    bool operator!=(const AlsaHwParams& p_rhs) const
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
typedef struct AlsaSwParams {
    snd_pcm_uframes_t start_threshold = ALSA_START_THRESHOLD;
    snd_pcm_uframes_t stop_threshold = ALSA_STOP_THRESHOLD;
    snd_pcm_uframes_t silence_size = ALSA_SILENCE_SIZE;
    snd_pcm_uframes_t silence_threshold = ALSA_SILENCE_THRESHOLD;

    bool operator!=(const AlsaSwParams& p_rhs) const
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
 * \return          Number of samples written, or negative error code.
 */
int alsa_write(int samples, snd_pcm_t* alsa_dev, void* data, int channels, snd_pcm_format_t alsa_format, std::atomic<bool>* shutdown_flag);

/**
 * Thread-safe ALSA writer class that consumes audio data from a lock-free queue.
 */
class AlsaWriteThread
{
public:
    AlsaWriteThread(
        AlsaHwParams hw_vals,
        AlsaSwParams sw_vals,
        boost::lockfree::spsc_queue<std::vector<uint8_t>>* queue,
        std::atomic<bool>* shutdown_flag = nullptr,
        std::atomic<bool>* data_available = nullptr);
    ~AlsaWriteThread() = default;
    snd_pcm_t * get_alsa_dev();
    std::string get_error();
    void close();
    void run();
private:
    snd_pcm_t * alsa_dev_ = nullptr;
    std::string error_str_;
    AlsaHwParams hw_vals_;
    boost::lockfree::spsc_queue<std::vector<uint8_t>>* queue_;
    std::atomic<bool>* shutdown_flag_;
    std::atomic<bool>* data_available_;
};

#endif
