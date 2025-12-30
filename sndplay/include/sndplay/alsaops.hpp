#ifndef SNDPLAY_ALSAOPS_HPP
#define SNDPLAY_ALSAOPS_HPP

#include <alsa/asoundlib.h>
#include <sndfile.h>
#include <atomic>
#include <string>
#include <optional>
#include <cstring>

#define ALSA_PERIOD_SIZE 2048
#define ALSA_BUFFER_PERIODS 4
// ALSA hardware parameters values
typedef struct AlsaHwParams {
    const char *device = "default";
    unsigned int channels = 2;
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

#endif