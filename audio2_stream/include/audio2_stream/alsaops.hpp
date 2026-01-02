#ifndef SNDPLAY_ALSAOPS_HPP
#define SNDPLAY_ALSAOPS_HPP

#include <alsa/asoundlib.h>
#include <sndfile.h>
#include <atomic>
#include <string>
#include <optional>
#include <cstring>

#define ALSA_PERIOD_SIZE 1024
#define ALSA_BUFFER_PERIODS 8
// ALSA hardware parameters values
typedef struct AlsaHwParams {
    const char *device = "default";
    unsigned int channels = 1;
    snd_pcm_uframes_t buffer_size = ALSA_PERIOD_SIZE * ALSA_BUFFER_PERIODS;
    snd_pcm_uframes_t period_size = ALSA_PERIOD_SIZE;
    snd_pcm_access_t access = SND_PCM_ACCESS_RW_INTERLEAVED;
    snd_pcm_format_t format = SND_PCM_FORMAT_S16;
    unsigned int samplerate = 48000;
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
    snd_pcm_uframes_t start_threshold = ALSA_PERIOD_SIZE * ALSA_BUFFER_PERIODS;
    snd_pcm_uframes_t stop_threshold = ALSA_PERIOD_SIZE * ALSA_BUFFER_PERIODS;

    bool operator!=(const AlsaSwParams& p_rhs) const
    {
        return std::tie(
                start_threshold,
                stop_threshold) !=
            std::tie(
                p_rhs.start_threshold,
                p_rhs.stop_threshold);
    }
} AlsaSwParams;

// Function declarations
snd_pcm_t * alsa_open (AlsaHwParams hw_vals, AlsaSwParams sw_vals);

/**
 * Write audio data to ALSA device.
 * \param samples Number of samples to write.
 * \param alsa_dev  Pointer to the ALSA PCM device.
 * \param data      Pointer to the audio data buffer.
 * \param channels  Number of audio channels.
 * \param alsa_format ALSA format of the audio data.
 * \return          Number of samples written, or negative error code.
 */
int alsa_write(int samples, snd_pcm_t* alsa_dev, void* data, int channels, snd_pcm_format_t alsa_format);

#endif