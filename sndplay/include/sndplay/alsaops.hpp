#ifndef SNDPLAY_ALSAOPS_HPP
#define SNDPLAY_ALSAOPS_HPP

#include <alsa/asoundlib.h>
#include <sndfile.h>
#include <atomic>
#include <string>
#include <tuple>
#include <cstring>

// Playback result structure
typedef struct {
    bool success = true;
    const char * error_message = "";
} PlaybackResult;

#define ALSA_PERIOD_SIZE 1024
#define ALSA_BUFFER_PERIODS 4
// ALSA hardware parameters values
typedef struct AlsaHwParams {
    const char *device = "default";
    unsigned int channels = 1;
    snd_pcm_uframes_t buffer_size = ALSA_PERIOD_SIZE * ALSA_BUFFER_PERIODS;
    snd_pcm_uframes_t period_size = ALSA_PERIOD_SIZE;
    snd_pcm_access_t access = SND_PCM_ACCESS_RW_INTERLEAVED;
    snd_pcm_format_t format = SND_PCM_FORMAT_S16;
    unsigned int samplerate = 0;
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
    snd_pcm_uframes_t stop_threshold = 0;
    snd_pcm_uframes_t silence_size = 0;

    bool operator!=(const AlsaSwParams& p_rhs) const
    {
        return std::tie(
                start_threshold,
                stop_threshold,
                silence_size) !=
            std::tie(
                p_rhs.start_threshold,
                p_rhs.stop_threshold,
                p_rhs.silence_size);
    }
} AlsaSwParams;

// Function declarations
snd_pcm_t * alsa_open (AlsaHwParams hw_vals, AlsaSwParams sw_vals);

PlaybackResult
alsa_play (SNDFILE *sndfile, SF_INFO sfinfo, snd_pcm_t* alsa_dev, snd_pcm_format_t alsa_format, std::atomic<bool>* shutdown_flag) ;

#endif