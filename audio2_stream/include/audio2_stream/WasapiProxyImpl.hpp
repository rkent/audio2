#ifndef AUDIO2_STREAM_WASAPIPROXYIMPL_HPP
#define AUDIO2_STREAM_WASAPIPROXYIMPL_HPP

#include "audio2_stream/IAudioDeviceProxy.hpp"
#include "audio2_stream/visibility_control.hpp"
#include <memory>
#include <optional>
#include <string>
#include <vector>

/**
 * Layout of a sample format as presented to WASAPI.
 */
struct WasapiSampleFormat
{
  bool is_float;                 // IEEE float, otherwise integer PCM
  unsigned short container_bits; // bits per sample in memory
  unsigned short valid_bits;     // significant bits within the container
};

/**
 * Map a sample format to its WASAPI layout.
 * \return The layout, or std::nullopt if the format is not supported by this backend.
 */
AUDIO2_STREAM_PUBLIC
std::optional<WasapiSampleFormat> wasapi_sample_format(SampleFormat format);

/**
 * WASAPI (Windows Audio Session API) implementation of IAudioDeviceProxy.
 * Uses shared mode with automatic sample rate, channel and format conversion,
 * so any requested stream parameters are accepted, like ALSA's plug devices.
 *
 * Device names: "default" selects the default endpoint for the direction. Any other
 * name matches an endpoint ID exactly, or a case-insensitive substring of an endpoint's
 * friendly name (e.g. "Speakers", "USB Audio").
 */
class AUDIO2_STREAM_PUBLIC WasapiProxyImpl : public IAudioDeviceProxy
{
public:
  WasapiProxyImpl();
  virtual ~WasapiProxyImpl();

  std::optional<std::string> preopen(
    const std::string & device,
    AudioDirection direction
  ) override;

  std::optional<std::string> open(
    DeviceParams & params,
    AudioDirection direction
  ) override;

  void close() override;

  int write(
    int samples,
    void * data,
    int channels,
    SampleFormat format,
    std::atomic<bool> * shutdown_flag = nullptr
  ) override;

  int read(
    int samples,
    void * data,
    int channels,
    SampleFormat format,
    std::atomic<bool> * shutdown_flag = nullptr
  ) override;

  bool is_open() const override;

  std::string device_name() const override
  {
    return device_name_;
  }

  std::string get_error() const override
  {
    return error_str_;
  }

  SampleFormat get_format() const override
  {
    return format_;
  }

  /**
   * List the friendly names of the active endpoints for a direction.
   */
  static std::vector<std::string> list_device_names(AudioDirection direction);

private:
  struct Impl;  // Holds the COM objects, keeping Windows headers out of this header
  std::unique_ptr<Impl> impl_;
  std::string device_name_;
  std::string error_str_;
  SampleFormat format_;

  std::optional<std::string> resolve_device(const std::string & device, AudioDirection direction);
  int fail(const char * context, long hr);
};

#endif // AUDIO2_STREAM_WASAPIPROXYIMPL_HPP
