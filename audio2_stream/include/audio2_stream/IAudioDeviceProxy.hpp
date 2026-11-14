#ifndef AUDIO2_STREAM_IAUDIODEVICEPROXY_HPP
#define AUDIO2_STREAM_IAUDIODEVICEPROXY_HPP

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include "audio2_stream/config.hpp"

enum class AudioDirection
{
  Playback,
  Capture
};

/**
 * Requested audio device configuration. open() may adjust these to what the device accepted.
 * Sample formats use the snd_pcm_format_t values on all platforms (see alsa_compat.hpp).
 */
struct DeviceParams
{
  std::string device = DEFAULT_DEVICE_NAME;
  unsigned int channels = 1;
  unsigned int samplerate = DEFAULT_SAMPLERATE;
  snd_pcm_format_t format = DEFAULT_SAMPLE_FORMAT;
  unsigned long period_frames = DEVICE_PERIOD_SIZE;
  unsigned long buffer_frames = DEVICE_BUFFER_SIZE;
};

/**
 * Interface for audio device operations, implemented per platform backend
 * (AlsaProxyImpl on Linux, WasapiProxyImpl on Windows).
 * This abstraction also allows for mocking and testing of device-dependent code.
 */
class IAudioDeviceProxy
{
public:
  virtual ~IAudioDeviceProxy() = default;

    /**
     * Acquire the device early, before its parameters are known, to reduce latency of open().
     * \param device    Device name.
     * \param direction Playback or capture.
     * \return          Optional error string if the device cannot be acquired.
     */
  virtual std::optional<std::string> preopen(const std::string & device, AudioDirection direction)
  {
    (void)device;
    (void)direction;
    return std::nullopt;
  }

    /**
     * Open the audio device with specified parameters.
     * \param params    Device parameters (may be modified by implementation).
     * \param direction Playback or capture.
     * \return          Optional error string if opening fails.
     */
  virtual std::optional<std::string> open(DeviceParams & params, AudioDirection direction) = 0;

    /**
     * Close the device, draining any queued playback.
     */
  virtual void close() = 0;

    /**
     * Write audio data to the device.
     * \param samples      Number of samples (frames * channels) to write.
     * \param data         Pointer to audio data buffer.
     * \param channels     Number of audio channels.
     * \param format       Sample format of the audio data.
     * \param shutdown_flag Pointer to shutdown flag (optional).
     * \return             Number of samples written, or negative error code.
     */
  virtual int write(
    int samples,
    void * data,
    int channels,
    snd_pcm_format_t format,
    std::atomic<bool> * shutdown_flag = nullptr
  ) = 0;

    /**
     * Read audio data from the device.
     * \param samples      Number of samples (frames * channels) to read.
     * \param data         Pointer to buffer for audio data.
     * \param channels     Number of audio channels.
     * \param format       Sample format of the audio data.
     * \param shutdown_flag Pointer to shutdown flag (optional).
     * \return             Number of samples read, or negative error code.
     */
  virtual int read(
    int samples,
    void * data,
    int channels,
    snd_pcm_format_t format,
    std::atomic<bool> * shutdown_flag = nullptr
  ) = 0;

    /**
     * \return true if the device is open (or pre-opened) and usable.
     */
  virtual bool is_open() const = 0;

    /**
     * \return Name of the device this proxy was opened or pre-opened with.
     */
  virtual std::string device_name() const = 0;

    /**
     * Get error string from last operation.
     * \return Error string, or empty string if no error.
     */
  virtual std::string get_error() const = 0;

    /**
     * Get the current format being used.
     * \return Sample format.
     */
  virtual snd_pcm_format_t get_format() const = 0;
};

/**
 * Create the audio device proxy for this platform's backend.
 * Defined by the compiled backend: AlsaProxyImpl.cpp (Linux) or WasapiProxyImpl.cpp (Windows).
 */
std::unique_ptr<IAudioDeviceProxy> make_default_device_proxy();

#endif // AUDIO2_STREAM_IAUDIODEVICEPROXY_HPP
