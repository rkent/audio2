#ifndef AUDIO2_STREAM_IALSADEVICE_HPP
#define AUDIO2_STREAM_IALSADEVICE_HPP

#include <alsa/asoundlib.h>
#include <optional>
#include <string>
#include <atomic>
#include "audio2_stream/alsaops.hpp"

/**
 * Interface for ALSA device operations.
 * This abstraction allows for mocking and testing of ALSA-dependent code
 * without requiring actual hardware devices.
 */
class IAlsaDevice
{
public:
  virtual ~IAlsaDevice() = default;

    /**
     * Open an ALSA PCM device with specified parameters.
     * \param hw_vals  Hardware parameters (may be modified by implementation).
     * \param sw_vals  Software parameters.
     * \return         Optional error string if opening fails.
     */
  virtual std::optional<std::string> open(
    AlsaHwParams & hw_vals,
    AlsaSwParams & sw_vals,
    snd_pcm_stream_t direction
  ) = 0;

    /**
     * Close the ALSA device.
     */
  virtual void close() = 0;

    /**
     * Write audio data to the device.
     * \param samples      Number of samples to write.
     * \param data         Pointer to audio data buffer.
     * \param channels     Number of audio channels.
     * \param format       ALSA format of the audio data.
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
     * \param samples      Number of samples to read.
     * \param data         Pointer to buffer for audio data.
     * \param channels     Number of audio channels.
     * \param format       ALSA format of the audio data.
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
     * Get the underlying ALSA device handle (for status operations).
     * \return Pointer to snd_pcm_t, or nullptr if not open.
     */
  virtual snd_pcm_t * get_handle() = 0;

    /**
     * Get error string from last operation.
     * \return Error string, or empty string if no error.
     */
  virtual std::string get_error() const = 0;

    /**
     * Get the current format being used.
     * \return ALSA PCM format.
     */
  virtual snd_pcm_format_t get_format() const = 0;
};

#endif // AUDIO2_STREAM_IALSADEVICE_HPP
