#include "audio2_stream/audioops.hpp"

#include <RtAudio.h>

#include <algorithm>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <memory>
#include <mutex>
#include <thread>

// Device name that discards playback and captures silence, paced in real time.
static const char * NULL_DEVICE_NAME = "null";

// How long a blocked read/write waits before rechecking shutdown and stream state
static const auto WAIT_SLICE = std::chrono::milliseconds(20);

// An open device stream. RtAudio delivers audio through a callback on its own thread,
// so a ring buffer (guarded by mutex) adapts that to the blocking read/write API.
struct AudioDevice
{
  ~AudioDevice()
  {
    // Close before the members used by the callback are destroyed.
    if (rtaudio && rtaudio->isStreamOpen()) {
      rtaudio->closeStream();
    }
  }

  std::unique_ptr<RtAudio> rtaudio;  // nullptr for the null device
  AudioDirection direction = AUDIO_PLAYBACK;
  AudioFormat format = AUDIO_FORMAT_UNKNOWN;
  unsigned int channels = 0;         // channels seen by audio_read/audio_write
  unsigned int device_channels = 0;  // channels of the RtAudio stream
  unsigned int samplerate = 0;
  std::size_t sample_bytes = 0;
  std::size_t frame_bytes = 0;  // sample_bytes * channels
  std::size_t start_frames = 0;
  unsigned int period_frames = 0;

  std::mutex mutex;
  std::condition_variable cv;
  std::vector<uint8_t> ring;  // holds frames of channels * sample_bytes
  std::size_t head = 0;       // read position in ring, bytes
  std::size_t count = 0;      // bytes stored in ring
  std::vector<uint8_t> scratch;  // callback buffer for channel conversion
  bool running = false;
  unsigned long xruns = 0;

  std::chrono::steady_clock::time_point null_clock;  // null device pacing
};

int audio_format_width(AudioFormat format)
{
  switch (format) {
    case AUDIO_FORMAT_S8:
      return 8;
    case AUDIO_FORMAT_S16:
      return 16;
    case AUDIO_FORMAT_S32:
    case AUDIO_FORMAT_FLOAT:
      return 32;
    case AUDIO_FORMAT_FLOAT64:
      return 64;
    default:
      return -1;
  }
}

const char * audio_format_to_string(AudioFormat format)
{
  switch (format) {
    case AUDIO_FORMAT_S8:
      return "S8";
    case AUDIO_FORMAT_S16:
      return "S16";
    case AUDIO_FORMAT_S32:
      return "S32";
    case AUDIO_FORMAT_FLOAT:
      return "FLOAT";
    case AUDIO_FORMAT_FLOAT64:
      return "FLOAT64";
    default:
      return "UNKNOWN";
  }
}

static RtAudioFormat rtaudio_format(AudioFormat format)
{
  switch (format) {
    case AUDIO_FORMAT_S8:
      return RTAUDIO_SINT8;
    case AUDIO_FORMAT_S16:
      return RTAUDIO_SINT16;
    case AUDIO_FORMAT_S32:
      return RTAUDIO_SINT32;
    case AUDIO_FORMAT_FLOAT64:
      return RTAUDIO_FLOAT64;
    case AUDIO_FORMAT_FLOAT:
    default:
      return RTAUDIO_FLOAT32;
  }
}

static std::unique_ptr<RtAudio> make_rtaudio()
{
  auto rtaudio = std::make_unique<RtAudio>(
    RtAudio::UNSPECIFIED,
    [](RtAudioErrorType type, const std::string & error_text) {
      if (type != RTAUDIO_WARNING) {
        printf("RtAudio error: %s\n", error_text.c_str());
      }
    });
  rtaudio->showWarnings(false);
  return rtaudio;
}

static unsigned int direction_channels(const RtAudio::DeviceInfo & info, AudioDirection direction)
{
  return direction == AUDIO_PLAYBACK ? info.outputChannels : info.inputChannels;
}

std::vector<std::string> audio_device_names(AudioDirection direction)
{
  std::vector<std::string> names;
  auto rtaudio = make_rtaudio();
  for (unsigned int id : rtaudio->getDeviceIds()) {
    auto info = rtaudio->getDeviceInfo(id);
    if (direction_channels(info, direction) > 0) {
      names.push_back(info.name);
    }
  }
  return names;
}

// Find the RtAudio device id for a device name, or 0 if none matches.
static unsigned int find_device_id(RtAudio & rtaudio, const std::string & name, AudioDirection direction)
{
  if (name.empty() || name == "default") {
    return direction == AUDIO_PLAYBACK ?
           rtaudio.getDefaultOutputDevice() : rtaudio.getDefaultInputDevice();
  }
  unsigned int partial_match = 0;
  for (unsigned int id : rtaudio.getDeviceIds()) {
    auto info = rtaudio.getDeviceInfo(id);
    if (direction_channels(info, direction) == 0) {
      continue;
    }
    if (info.name == name) {
      return id;
    }
    if (!partial_match && info.name.find(name) != std::string::npos) {
      partial_match = id;
    }
  }
  return partial_match;
}

// Ring buffer helpers, called with mutex held. Sizes are in bytes.
static void ring_push(AudioDevice * dev, const uint8_t * src, std::size_t bytes)
{
  std::size_t tail = (dev->head + dev->count) % dev->ring.size();
  std::size_t first = std::min(bytes, dev->ring.size() - tail);
  std::memcpy(dev->ring.data() + tail, src, first);
  std::memcpy(dev->ring.data(), src + first, bytes - first);
  dev->count += bytes;
}

static void ring_pop(AudioDevice * dev, uint8_t * dst, std::size_t bytes)
{
  std::size_t first = std::min(bytes, dev->ring.size() - dev->head);
  if (dst) {
    std::memcpy(dst, dev->ring.data() + dev->head, first);
    std::memcpy(dst + first, dev->ring.data(), bytes - first);
  }
  dev->head = (dev->head + bytes) % dev->ring.size();
  dev->count -= bytes;
}

// Copy frames between channel counts. Only from_channels == 1 (duplicate to every channel)
// or to_channels <= from_channels (keep the first channels) are used.
static void convert_channels(
  const uint8_t * src, unsigned int from_channels, uint8_t * dst, unsigned int to_channels,
  unsigned int frames, std::size_t sample_bytes)
{
  for (unsigned int f = 0; f < frames; ++f) {
    for (unsigned int c = 0; c < to_channels; ++c) {
      unsigned int src_c = from_channels == 1 ? 0 : c;
      std::memcpy(dst + (f * to_channels + c) * sample_bytes,
        src + (f * from_channels + src_c) * sample_bytes, sample_bytes);
    }
  }
}

static int rtaudio_callback(
  void * output, void * input, unsigned int frames, double /*stream_time*/,
  RtAudioStreamStatus status, void * user_data)
{
  auto dev = static_cast<AudioDevice *>(user_data);
  std::size_t bytes = frames * dev->frame_bytes;
  bool convert = dev->channels != dev->device_channels;
  {
    std::lock_guard<std::mutex> lock(dev->mutex);
    if (status) {
      dev->xruns++;
    }
    if (convert && dev->scratch.size() < bytes) {
      dev->scratch.resize(bytes);
    }
    if (dev->direction == AUDIO_PLAYBACK) {
      uint8_t * out = convert ? dev->scratch.data() : static_cast<uint8_t *>(output);
      std::size_t available = std::min(bytes, dev->count);
      ring_pop(dev, out, available);
      // On underrun, play silence (all supported formats are signed, so zero is silence).
      std::memset(out + available, 0, bytes - available);
      if (convert) {
        convert_channels(out, dev->channels, static_cast<uint8_t *>(output),
          dev->device_channels, frames, dev->sample_bytes);
      }
    } else {
      const uint8_t * in = static_cast<const uint8_t *>(input);
      if (convert) {
        convert_channels(in, dev->device_channels, dev->scratch.data(), dev->channels,
          frames, dev->sample_bytes);
        in = dev->scratch.data();
      }
      if (bytes > dev->ring.size()) {
        in += bytes - dev->ring.size();
        bytes = dev->ring.size();
      }
      std::size_t space = dev->ring.size() - dev->count;
      if (bytes > space) {
        // Overrun, drop the oldest captured audio.
        ring_pop(dev, nullptr, bytes - space);
        dev->xruns++;
      }
      ring_push(dev, in, bytes);
    }
  }
  dev->cv.notify_all();
  return 0;
}

// Start the stream. Called with lock held, which is released while starting.
static bool start_stream(AudioDevice * dev, std::unique_lock<std::mutex> & lock)
{
  dev->running = true;
  lock.unlock();
  auto err = dev->rtaudio->startStream();
  lock.lock();
  if (err != RTAUDIO_NO_ERROR) {
    printf("audio device: cannot start stream: %s\n", dev->rtaudio->getErrorText().c_str());
    dev->running = false;
  }
  return dev->running;
}

// Stream stopped unexpectedly (for example, the device was disconnected)?
static bool stream_failed(AudioDevice * dev)
{
  return dev->running && !dev->rtaudio->isStreamRunning();
}

// Simulate a device consuming or producing frames in real time.
static void null_device_wait(AudioDevice * dev, int frames, std::atomic<bool> * shutdown_flag)
{
  auto now = std::chrono::steady_clock::now();
  if (dev->null_clock < now) {
    dev->null_clock = now;
  }
  dev->null_clock += std::chrono::microseconds(1000000LL * frames / dev->samplerate);
  while (std::chrono::steady_clock::now() < dev->null_clock) {
    if (shutdown_flag && shutdown_flag->load()) {
      break;
    }
    std::this_thread::sleep_until(
      std::min(dev->null_clock, std::chrono::steady_clock::now() + WAIT_SLICE));
  }
}

std::optional<std::string>
audio_open(AudioHwParams & hw_vals, AudioSwParams & sw_vals, AudioDevice *& audio_dev)
{
  audio_dev = nullptr;
  std::string device_name = hw_vals.device ? hw_vals.device : "";
  int width = audio_format_width(hw_vals.format);
  if (width <= 0) {
    return std::format("audio_open error: unsupported format {}", static_cast<int>(hw_vals.format));
  }
  if (hw_vals.channels == 0 || hw_vals.samplerate == 0) {
    return std::format("audio_open error: invalid channels {} or samplerate {}",
      hw_vals.channels, hw_vals.samplerate);
  }

  auto dev = std::make_unique<AudioDevice>();
  dev->direction = hw_vals.direction;
  dev->format = hw_vals.format;
  dev->channels = hw_vals.channels;
  dev->device_channels = hw_vals.channels;
  dev->samplerate = hw_vals.samplerate;
  dev->sample_bytes = width / 8;
  dev->frame_bytes = dev->sample_bytes * dev->channels;

  if (device_name == NULL_DEVICE_NAME) {
    audio_dev = dev.release();
    return std::nullopt;
  }

  auto rtaudio = make_rtaudio();
  std::string api_name = RtAudio::getApiDisplayName(rtaudio->getCurrentApi());
  if (rtaudio->getDeviceCount() == 0) {
    return std::format("audio_open error: no audio devices found (RtAudio API {})", api_name);
  }
  unsigned int device_id = find_device_id(*rtaudio, device_name, hw_vals.direction);
  if (device_id == 0) {
    std::string names;
    for (auto & name : audio_device_names(hw_vals.direction)) {
      names += "\n  " + name;
    }
    return std::format("audio_open error: cannot find {} device '{}' (RtAudio API {}). "
             "Available devices:{}", hw_vals.direction == AUDIO_PLAYBACK ? "playback" : "capture",
             device_name, api_name, names);
  }
  auto info = rtaudio->getDeviceInfo(device_id);
  printf("audio_open: using %s device '%s' (RtAudio API %s)\n",
    hw_vals.direction == AUDIO_PLAYBACK ? "playback" : "capture", info.name.c_str(),
    api_name.c_str());

  // Mono to a multichannel output, or a mono input to multichannel, is duplicated across
  // channels. Otherwise the device must have at least the requested channels.
  unsigned int max_channels = direction_channels(info, hw_vals.direction);
  if (hw_vals.channels > max_channels) {
    if (max_channels != 1) {
      return std::format("audio_open error: device '{}' supports {} channels, {} requested",
               info.name, max_channels, hw_vals.channels);
    }
    dev->device_channels = 1;
  } else if (hw_vals.channels == 1 && hw_vals.direction == AUDIO_PLAYBACK && max_channels >= 2) {
    dev->device_channels = 2;
  }

  // Use the closest supported rate, like ALSA's set_rate_near.
  if (!info.sampleRates.empty() &&
    std::find(info.sampleRates.begin(), info.sampleRates.end(), hw_vals.samplerate) ==
    info.sampleRates.end())
  {
    hw_vals.samplerate = *std::min_element(info.sampleRates.begin(), info.sampleRates.end(),
        [&](unsigned int a, unsigned int b) {
          return std::abs(static_cast<long>(a) - static_cast<long>(hw_vals.samplerate)) <
          std::abs(static_cast<long>(b) - static_cast<long>(hw_vals.samplerate));
        });
  }

  RtAudio::StreamParameters stream_params;
  stream_params.deviceId = device_id;
  stream_params.nChannels = dev->device_channels;
  stream_params.firstChannel = 0;
  RtAudio::StreamOptions options;
  if (hw_vals.period_size > 0 && hw_vals.buffer_size >= hw_vals.period_size) {
    options.numberOfBuffers = static_cast<unsigned int>(hw_vals.buffer_size / hw_vals.period_size);
  }
  options.streamName = "audio2_stream";
  unsigned int buffer_frames = static_cast<unsigned int>(hw_vals.period_size);

  auto err = rtaudio->openStream(
    hw_vals.direction == AUDIO_PLAYBACK ? &stream_params : nullptr,
    hw_vals.direction == AUDIO_CAPTURE ? &stream_params : nullptr,
    rtaudio_format(hw_vals.format), hw_vals.samplerate, &buffer_frames, &rtaudio_callback,
    dev.get(), &options);
  if (err != RTAUDIO_NO_ERROR) {
    return std::format("audio_open error: cannot open device '{}': {}", info.name,
             rtaudio->getErrorText());
  }

  hw_vals.samplerate = rtaudio->getStreamSampleRate();
  hw_vals.period_size = buffer_frames;
  hw_vals.buffer_size = std::max<unsigned long>(hw_vals.buffer_size, 2ul * buffer_frames);
  dev->samplerate = hw_vals.samplerate;
  dev->period_frames = buffer_frames;
  // Capture gets extra room since it is drained by a thread that may be slow to wake.
  std::size_t ring_frames = hw_vals.buffer_size * (hw_vals.direction == AUDIO_CAPTURE ? 2 : 1);
  dev->ring.resize(ring_frames * dev->frame_bytes);
  dev->scratch.resize(buffer_frames * dev->frame_bytes *
    std::max(dev->channels, dev->device_channels));
  dev->start_frames = std::min<std::size_t>(sw_vals.start_threshold, ring_frames);
  dev->rtaudio = std::move(rtaudio);
  audio_dev = dev.release();
  return std::nullopt;
} /* audio_open */

void audio_drain(AudioDevice * audio_dev)
{
  if (!audio_dev || !audio_dev->rtaudio) {
    return;
  }
  std::unique_lock<std::mutex> lock(audio_dev->mutex);
  if (audio_dev->direction == AUDIO_PLAYBACK) {
    if (!audio_dev->running && audio_dev->count > 0) {
      start_stream(audio_dev, lock);
    }
    // Wait for the buffered audio to be handed to the device, with a time limit in case
    // the device has stopped consuming.
    auto buffered_ms = 1000 * audio_dev->count / audio_dev->frame_bytes / audio_dev->samplerate;
    auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(buffered_ms + 1000);
    while (audio_dev->running && audio_dev->count > 0 && !stream_failed(audio_dev)) {
      if (audio_dev->cv.wait_until(lock, deadline) == std::cv_status::timeout) {
        break;
      }
    }
  }
  bool was_running = audio_dev->running;
  audio_dev->running = false;
  lock.unlock();
  if (was_running && audio_dev->direction == AUDIO_PLAYBACK) {
    // stopStream does not wait for the device to play what it was already given (WASAPI
    // discards it), so keep the stream running, now outputting silence, until it has.
    long latency = std::max(0L, audio_dev->rtaudio->getStreamLatency());
    auto tail_frames = latency + 2L * audio_dev->period_frames;
    std::this_thread::sleep_for(
      std::chrono::milliseconds(1000 * tail_frames / audio_dev->samplerate));
  }
  if (was_running && audio_dev->rtaudio->isStreamRunning()) {
    audio_dev->rtaudio->stopStream();
  }
  if (audio_dev->xruns > 0) {
    printf("audio_drain: %lu xruns occurred\n", audio_dev->xruns);
  }
}

void audio_close(AudioDevice *& audio_dev)
{
  delete audio_dev;
  audio_dev = nullptr;
}

int audio_write(
  int samples, AudioDevice * audio_dev, const void * data, int channels,
  AudioFormat format, std::atomic<bool> * shutdown_flag)
{
  if (!audio_dev) {
    return -1;
  } else if (channels <= 0) {
    return -2;
  } else if (samples <= 0) {
    return 0;
  }
  if (audio_dev->direction != AUDIO_PLAYBACK ||
    static_cast<unsigned int>(channels) != audio_dev->channels || format != audio_dev->format)
  {
    printf("audio_write: device opened for different direction, channels or format\n");
    return -3;
  }
  int frames = samples / channels;
  if (!audio_dev->rtaudio) {
    null_device_wait(audio_dev, frames, shutdown_flag);
    return frames * channels;
  }

  const uint8_t * src = static_cast<const uint8_t *>(data);
  std::size_t total = frames * audio_dev->frame_bytes;
  std::size_t done = 0;
  std::unique_lock<std::mutex> lock(audio_dev->mutex);
  while (done < total) {
    if (shutdown_flag && shutdown_flag->load()) {
      break;
    }
    if (stream_failed(audio_dev)) {
      printf("audio_write: device stream stopped\n");
      return -4;
    }
    std::size_t n = std::min(audio_dev->ring.size() - audio_dev->count, total - done);
    if (n > 0) {
      ring_push(audio_dev, src + done, n);
      done += n;
    }
    if (!audio_dev->running && audio_dev->count >= audio_dev->start_frames * audio_dev->frame_bytes) {
      if (!start_stream(audio_dev, lock)) {
        return -5;
      }
    }
    if (done < total) {
      if (!audio_dev->running) {
        // Buffer is full but below the start threshold, so start now.
        if (!start_stream(audio_dev, lock)) {
          return -5;
        }
      }
      audio_dev->cv.wait_for(lock, WAIT_SLICE);
    }
  }
  return static_cast<int>(done / audio_dev->frame_bytes) * channels;
}

int audio_read(
  int samples, AudioDevice * audio_dev, void * data, int channels,
  AudioFormat format, std::atomic<bool> * shutdown_flag)
{
  if (!audio_dev) {
    return -1;
  } else if (channels <= 0) {
    return -2;
  } else if (samples <= 0) {
    return 0;
  }
  if (audio_dev->direction != AUDIO_CAPTURE ||
    static_cast<unsigned int>(channels) != audio_dev->channels || format != audio_dev->format)
  {
    printf("audio_read: device opened for different direction, channels or format\n");
    return -3;
  }
  int frames = samples / channels;
  if (!audio_dev->rtaudio) {
    null_device_wait(audio_dev, frames, shutdown_flag);
    std::memset(data, 0, frames * audio_dev->frame_bytes);
    return frames * channels;
  }

  uint8_t * dst = static_cast<uint8_t *>(data);
  std::size_t total = frames * audio_dev->frame_bytes;
  std::size_t done = 0;
  std::unique_lock<std::mutex> lock(audio_dev->mutex);
  if (!audio_dev->running && !start_stream(audio_dev, lock)) {
    return -5;
  }
  while (done < total) {
    if (shutdown_flag && shutdown_flag->load()) {
      break;
    }
    if (stream_failed(audio_dev)) {
      printf("audio_read: device stream stopped\n");
      return -4;
    }
    std::size_t n = std::min(audio_dev->count, total - done);
    if (n > 0) {
      ring_pop(audio_dev, dst + done, n);
      done += n;
    } else {
      audio_dev->cv.wait_for(lock, WAIT_SLICE);
    }
  }
  return static_cast<int>(done / audio_dev->frame_bytes) * channels;
}
