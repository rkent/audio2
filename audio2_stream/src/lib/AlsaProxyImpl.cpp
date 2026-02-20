#include "audio2_stream/AlsaProxyImpl.hpp"
#include "audio2_stream/alsaops.hpp"
#include <cstdio>

std::optional<std::string> AlsaProxyImpl::open(
  AlsaHwParams & hw_vals,
  AlsaSwParams & sw_vals,
  snd_pcm_stream_t direction)
{
  printf("AlsaProxyImpl::open called at %s\n", format_timestamp().c_str());
  printf(
    "AlsaProxyImpl::open called with hw_params: device=%s, channels=%u, samplerate=%u, format=%d, direction=%d\n",
           hw_vals.device,
           hw_vals.channels,
           hw_vals.samplerate,
           hw_vals.format,
           direction);

    // Set direction in hw_vals
  hw_vals.direction = direction;

    // Open the ALSA device
  auto result = alsa_open(hw_vals, sw_vals, alsa_dev_);
  printf("AlsaProxyImpl::open completed at %s\n", format_timestamp().c_str());

  if (result.has_value()) {
    error_str_ = result.value();
    return error_str_;
  }

    // Store the format that was actually opened
  format_ = hw_vals.format;

  return std::nullopt;
}

void AlsaProxyImpl::close()
{
  printf("AlsaProxyImpl::close called\n");
  if (alsa_dev_) {
    snd_pcm_drain(alsa_dev_);
    snd_pcm_close(alsa_dev_);
    alsa_dev_ = nullptr;
  }
}

int AlsaProxyImpl::write(
  int samples,
  void * data,
  int channels,
  snd_pcm_format_t format,
  std::atomic<bool> * shutdown_flag
)
{
  if (!alsa_dev_) {
    error_str_ = "ALSA device is not opened";
    return -1;
  }

  int result = alsa_write(samples, alsa_dev_, data, channels, format, shutdown_flag);
  if (result < 0) {
    error_str_ = "ALSA write failed";
  }
  return result;
}

int AlsaProxyImpl::read(
  int samples,
  void * data,
  int channels,
  snd_pcm_format_t format,
  std::atomic<bool> * shutdown_flag
)
{
  if (!alsa_dev_) {
    error_str_ = "ALSA device is not opened";
    return -1;
  }

  int result = alsa_read(samples, alsa_dev_, data, channels, format, shutdown_flag);
  if (result < 0) {
    error_str_ = "ALSA read failed";
  }
  printf("AlsaProxyImpl::read requested %d samples, got %d samples\n", samples, result);
  return result;
}
