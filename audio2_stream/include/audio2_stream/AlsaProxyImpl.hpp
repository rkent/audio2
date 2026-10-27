#ifndef AUDIO2_STREAM_ALSAPROXYIMPL_HPP
#define AUDIO2_STREAM_ALSAPROXYIMPL_HPP

#include "audio2_stream/IAlsaProxy.hpp"
#include <memory>

/**
 * Real implementation of IAlsaProxy that wraps actual ALSA hardware operations.
 * This is used in production code for actual audio I/O.
 */
class AlsaProxyImpl : public IAlsaProxy
{
public:
  AlsaProxyImpl(AudioDevice * alsa_dev = nullptr)
  : alsa_dev_(alsa_dev), error_str_(""), format_(AUDIO_FORMAT_UNKNOWN) {}

  virtual ~AlsaProxyImpl()
  {
    close();
  }

  std::optional<std::string> open(
    AudioHwParams & hw_vals,
    AudioSwParams & sw_vals,
    AudioDirection direction
  ) override;

  void close() override;

  int write(
    int samples,
    void * data,
    int channels,
    AudioFormat format,
    std::atomic<bool> * shutdown_flag = nullptr
  ) override;

  int read(
    int samples,
    void * data,
    int channels,
    AudioFormat format,
    std::atomic<bool> * shutdown_flag = nullptr
  ) override;

  AudioDevice * get_handle() override
  {
    return alsa_dev_;
  }

  std::string get_error() const override
  {
    return error_str_;
  }

  AudioFormat get_format() const override
  {
    return format_;
  }

private:
  AudioDevice * alsa_dev_;
  std::string error_str_;
  AudioFormat format_;
};

#endif // AUDIO2_STREAM_ALSAPROXYIMPL_HPP
