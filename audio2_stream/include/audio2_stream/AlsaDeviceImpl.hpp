#ifndef AUDIO2_STREAM_ALSADEVICEIMPL_HPP
#define AUDIO2_STREAM_ALSADEVICEIMPL_HPP

#include "audio2_stream/IAlsaDevice.hpp"
#include <memory>

/**
 * Real implementation of IAlsaDevice that wraps actual ALSA hardware operations.
 * This is used in production code for actual audio I/O.
 */
class AlsaDeviceImpl : public IAlsaDevice
{
public:
    AlsaDeviceImpl() : alsa_dev_(nullptr), error_str_(""), format_(SND_PCM_FORMAT_UNKNOWN) {}
    
    virtual ~AlsaDeviceImpl() {
        close();
    }

    std::optional<std::string> open(
        AlsaHwParams & hw_vals,
        AlsaSwParams & sw_vals,
        snd_pcm_stream_t direction
    ) override;

    void close() override;

    int write(
        int samples,
        void* data,
        int channels,
        snd_pcm_format_t format,
        std::atomic<bool>* shutdown_flag = nullptr
    ) override;

    int read(
        int samples,
        void* data,
        int channels,
        snd_pcm_format_t format,
        std::atomic<bool>* shutdown_flag = nullptr
    ) override;

    snd_pcm_t* get_handle() override {
        return alsa_dev_;
    }

    std::string get_error() const override {
        return error_str_;
    }

    snd_pcm_format_t get_format() const override {
        return format_;
    }

private:
    snd_pcm_t* alsa_dev_;
    std::string error_str_;
    snd_pcm_format_t format_;
};

#endif // AUDIO2_STREAM_ALSADEVICEIMPL_HPP
