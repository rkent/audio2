#ifndef AUDIO2_STREAM_TEST_MOCKALSADEVICE_HPP
#define AUDIO2_STREAM_TEST_MOCKALSADEVICE_HPP

#include <gmock/gmock.h>
#include "audio2_stream/IAlsaDevice.hpp"

/**
 * Mock implementation of IAlsaDevice for testing.
 * Uses Google Mock to allow setting expectations on ALSA operations.
 */
class MockAlsaDevice : public IAlsaDevice
{
public:
    MOCK_METHOD(
        std::optional<std::string>,
        open,
        (AlsaHwParams & hw_vals, AlsaSwParams & sw_vals, snd_pcm_stream_t direction),
        (override)
    );

    MOCK_METHOD(void, close, (), (override));

    MOCK_METHOD(
        int,
        write,
        (int samples, void* data, int channels, snd_pcm_format_t format, std::atomic<bool>* shutdown_flag),
        (override)
    );

    MOCK_METHOD(
        int,
        read,
        (int samples, void* data, int channels, snd_pcm_format_t format, std::atomic<bool>* shutdown_flag),
        (override)
    );

    MOCK_METHOD(snd_pcm_t*, get_handle, (), (override));

    MOCK_METHOD(std::string, get_error, (), (const, override));

    MOCK_METHOD(snd_pcm_format_t, get_format, (), (const, override));
};

#endif // AUDIO2_STREAM_TEST_MOCKALSADEVICE_HPP
