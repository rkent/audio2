#include "audio2_stream/AlsaProxyImpl.hpp"
#include "audio2_stream/alsaops.hpp"
#include <cstdio>

std::unique_ptr<IAudioDeviceProxy> make_default_device_proxy()
{
  return std::make_unique<AlsaProxyImpl>();
}

namespace
{

snd_pcm_stream_t to_alsa_stream(AudioDirection direction)
{
  return direction == AudioDirection::Capture ? SND_PCM_STREAM_CAPTURE : SND_PCM_STREAM_PLAYBACK;
}

}  // namespace

std::optional<std::string> AlsaProxyImpl::preopen(
  const std::string & device,
  AudioDirection direction)
{
  if (alsa_dev_) {
    return std::nullopt;
  }
  device_name_ = device;
  int open_result = snd_pcm_open(&alsa_dev_, device.c_str(), to_alsa_stream(direction), 0);
  if (open_result < 0) {
    alsa_dev_ = nullptr;
    error_str_ = snd_strerror(open_result);
    return error_str_;
  }
  return std::nullopt;
}

std::optional<std::string> AlsaProxyImpl::open(
  DeviceParams & params,
  AudioDirection direction)
{
  printf("AlsaProxyImpl::open called at %s\n", format_timestamp().c_str());
  printf(
    "AlsaProxyImpl::open called with params: device=%s, channels=%u, samplerate=%u, format=%d, direction=%d\n",
           params.device.c_str(),
           params.channels,
           params.samplerate,
           static_cast<int>(params.format),
           static_cast<int>(direction));

  if (alsa_dev_ && device_name_ != params.device) {
    // Pre-opened for a different device
    snd_pcm_close(alsa_dev_);
    alsa_dev_ = nullptr;
  }
  device_name_ = params.device;

  AlsaHwParams hw_vals;
  hw_vals.device = device_name_.c_str();
  hw_vals.channels = params.channels;
  hw_vals.samplerate = params.samplerate;
  hw_vals.format = to_alsa_format(params.format);
  hw_vals.period_size = params.period_frames;
  hw_vals.buffer_size = params.buffer_frames;
  hw_vals.direction = to_alsa_stream(direction);
  AlsaSwParams sw_vals;

    // Open the ALSA device, reusing a pre-opened handle if present
  auto result = alsa_open(hw_vals, sw_vals, alsa_dev_);
  printf("AlsaProxyImpl::open completed at %s\n", format_timestamp().c_str());

  if (result.has_value()) {
    alsa_dev_ = nullptr;  // alsa_open closes the device on failure
    error_str_ = result.value();
    return error_str_;
  }

    // Report the values that were actually opened
  params.channels = hw_vals.channels;
  params.samplerate = hw_vals.samplerate;
  params.format = from_alsa_format(hw_vals.format);
  params.period_frames = hw_vals.period_size;
  params.buffer_frames = hw_vals.buffer_size;
  format_ = from_alsa_format(hw_vals.format);
  epipe_count_ = 0;

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
  std::atomic<bool> * shutdown_flag
)
{
  if (!alsa_dev_) {
    error_str_ = "ALSA device is not opened";
    return -1;
  }
  if (format_ == SampleFormat::Unknown) {
    error_str_ = "ALSA device is pre-opened but not configured; call open() first";
    return -1;
  }

  int result = alsa_write(
    samples, alsa_dev_, data, channels, to_alsa_format(format_), shutdown_flag, epipe_count_);
  if (result < 0) {
    error_str_ = std::string("ALSA write failed: ") + snd_strerror(result);
  }
  return result;
}

int AlsaProxyImpl::read(
  int samples,
  void * data,
  int channels,
  std::atomic<bool> * shutdown_flag
)
{
  if (!alsa_dev_) {
    error_str_ = "ALSA device is not opened";
    return -1;
  }
  if (format_ == SampleFormat::Unknown) {
    error_str_ = "ALSA device is pre-opened but not configured; call open() first";
    return -1;
  }

  int result = alsa_read(
    samples, alsa_dev_, data, channels, to_alsa_format(format_), shutdown_flag, epipe_count_);
  if (result < 0) {
    error_str_ = std::string("ALSA read failed: ") + snd_strerror(result);
  }
  printf("AlsaProxyImpl::read requested %d samples, got %d samples\n", samples, result);
  return result;
}
