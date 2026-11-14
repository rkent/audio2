#ifndef AUDIO2_STREAM_ALSAPROXYIMPL_HPP
#define AUDIO2_STREAM_ALSAPROXYIMPL_HPP

#include "audio2_stream/IAudioDeviceProxy.hpp"
#include "audio2_stream/alsaops.hpp"
#include "audio2_stream/visibility_control.hpp"
#include <memory>

/**
 * ALSA implementation of IAudioDeviceProxy that wraps actual ALSA hardware operations.
 * This is used in production code for actual audio I/O on Linux.
 */
class AUDIO2_STREAM_PUBLIC AlsaProxyImpl : public IAudioDeviceProxy
{
public:
  AlsaProxyImpl()
  : alsa_dev_(nullptr), error_str_(""), format_(SND_PCM_FORMAT_UNKNOWN) {}

  virtual ~AlsaProxyImpl()
  {
    close();
  }

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
    snd_pcm_format_t format,
    std::atomic<bool> * shutdown_flag = nullptr
  ) override;

  int read(
    int samples,
    void * data,
    int channels,
    snd_pcm_format_t format,
    std::atomic<bool> * shutdown_flag = nullptr
  ) override;

  bool is_open() const override
  {
    return alsa_dev_ != nullptr;
  }

  std::string device_name() const override
  {
    return device_name_;
  }

  std::string get_error() const override
  {
    return error_str_;
  }

  snd_pcm_format_t get_format() const override
  {
    return format_;
  }

  snd_pcm_t * get_handle()
  {
    return alsa_dev_;
  }

private:
  snd_pcm_t * alsa_dev_;
  std::string device_name_;
  std::string error_str_;
  snd_pcm_format_t format_;
};

#endif // AUDIO2_STREAM_ALSAPROXYIMPL_HPP
