#include "audio2_stream/ra_buffer_file.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <RtAudio.h>
#include <sndfile.hh>

// Convert RtAudio format to human-readable string.
std::string format_to_string(RtAudioFormat format)
{
  switch (format) {
    case RTAUDIO_SINT8:
      return "SINT8";
    case RTAUDIO_SINT16:
      return "SINT16";
    case RTAUDIO_SINT24:
      return "SINT24";
    case RTAUDIO_SINT32:
      return "SINT32";
    case RTAUDIO_FLOAT32:
      return "FLOAT32";
    case RTAUDIO_FLOAT64:
      return "FLOAT64";
    default:
      return "UNKNOWN";
  }
}

// Get sample size in bytes from RtAudioFormat
int sample_size_from_rtaudio_format(RtAudioFormat format)
{
  switch (format) {
    case RTAUDIO_SINT8:
      return 1;
    case RTAUDIO_SINT16:
      return 2;
    case RTAUDIO_SINT24:
      return 3;
    case RTAUDIO_SINT32:
      return 4;
    case RTAUDIO_FLOAT32:
      return 4;
    case RTAUDIO_FLOAT64:
      return 8;
    default:
      return -1;
  }
}

SfgRwFormat sfg_format_from_rtaudio_format(RtAudioFormat ra_format)
{
  switch (ra_format) {
    case RTAUDIO_SINT8:
      return SFG_BYTE;
    case RTAUDIO_SINT16:
      return SFG_SHORT;
    case RTAUDIO_SINT24:
    case RTAUDIO_SINT32:
      return SFG_INT;
    case RTAUDIO_FLOAT32:
      return SFG_FLOAT;
    case RTAUDIO_FLOAT64:
      return SFG_DOUBLE;
    default:
      return SFG_INVALID;
  }
}

RtAudioFormat rtaudio_format_from_sfg_format(SfgRwFormat sfg_format)
{
  switch (sfg_format) {
    case SFG_BYTE:
      return RTAUDIO_SINT8;
    case SFG_SHORT:
      return RTAUDIO_SINT16;
    case SFG_INT:
      return RTAUDIO_SINT32;
    case SFG_FLOAT:
      return RTAUDIO_FLOAT32;
    case SFG_DOUBLE:
      return RTAUDIO_FLOAT64;
    default:
      return 0;
  }
}

struct RaPlaybackContext
{
  SndfileHandle fileh;
  std::atomic<bool> * shutdown_flag{nullptr};
  std::atomic<bool> finished{false};
  int channels{0};
  RtAudioFormat ra_format{RTAUDIO_FLOAT32};
};

static int raAudioCallback(
  void * outputBuffer,
  void * /*inputBuffer*/,
  unsigned int nBufferFrames,
  double /*streamTime*/,
  RtAudioStreamStatus /*status*/,
  void * userData)
{
  auto * data = static_cast<RaPlaybackContext *>(userData);
  if (data->shutdown_flag && data->shutdown_flag->load()) {
    return 2;  // Abort immediately
  }

  sf_count_t frames_read = 0;
  int sample_bytes = sample_size_from_rtaudio_format(data->ra_format);
  if (sample_bytes <= 0) {
    return 2;
  }

  switch (data->ra_format) {
    case RTAUDIO_FLOAT32:
      frames_read = data->fileh.readf(static_cast<float *>(outputBuffer), nBufferFrames);
      break;
    case RTAUDIO_SINT16:
      frames_read = data->fileh.readf(static_cast<short *>(outputBuffer), nBufferFrames);
      break;
    case RTAUDIO_SINT32:
      frames_read = data->fileh.readf(static_cast<int *>(outputBuffer), nBufferFrames);
      break;
    case RTAUDIO_FLOAT64:
      frames_read = data->fileh.readf(static_cast<double *>(outputBuffer), nBufferFrames);
      break;
    default:
      return 2;
  }

  if (frames_read < static_cast<sf_count_t>(nBufferFrames)) {
    sf_count_t remaining_frames = static_cast<sf_count_t>(nBufferFrames) - frames_read;
    char * out_char = static_cast<char *>(outputBuffer);
    std::memset(
      out_char + (frames_read * data->channels * sample_bytes),
      0,
      remaining_frames * data->channels * sample_bytes);
    data->finished.store(true);
    return 1;  // Stop and drain stream
  }

  return 0;
}

std::optional<std::string>
ra_play(
  SndfileHandle fileh, RtAudio & dac, RtAudioFormat ra_format,
  std::atomic<bool> * shutdown_flag)
{
  if (fileh.error()) {
    return std::string("Invalid sound file handle: ") + fileh.strError();
  }

  if (dac.getDeviceIds().empty()) {
    return "No audio output devices found by RtAudio";
  }

  // If stream is open, ensure it's closed before opening for this file
  if (dac.isStreamOpen()) {
    if (dac.isStreamRunning()) {
      dac.stopStream();
    }
    dac.closeStream();
  }

  RaPlaybackContext context;
  context.fileh = fileh;
  context.shutdown_flag = shutdown_flag;
  context.finished.store(false);
  context.channels = fileh.channels();
  context.ra_format = ra_format;

  RtAudio::StreamParameters oParams;
  oParams.deviceId = dac.getDefaultOutputDevice();
  oParams.nChannels = static_cast<unsigned int>(fileh.channels());
  oParams.firstChannel = 0;

  unsigned int bufferFrames = 512;
  RtAudio::StreamOptions options;
  // This needs docker support to use.
  // "The container process does not have CAP_SYS_NICE or realtime rtprio limits enabled,
  // "so pthread_setschedparam fails and RtAudio falls back to standard scheduling
  // "with this notice.""
  // options.flags = RTAUDIO_SCHEDULE_REALTIME;

  RtAudioErrorType err = dac.openStream(
    &oParams,
    nullptr,
    ra_format,
    static_cast<unsigned int>(fileh.samplerate()),
    &bufferFrames,
    &raAudioCallback,
    &context,
    &options);

  if (err != RTAUDIO_NO_ERROR) {
    return "Failed to open RtAudio stream: " + dac.getErrorText();
  }

  err = dac.startStream();
  if (err != RTAUDIO_NO_ERROR) {
    std::string err_msg = "Failed to start RtAudio stream: " + dac.getErrorText();
    dac.closeStream();
    return err_msg;
  }

  while (dac.isStreamRunning() && (!shutdown_flag || !shutdown_flag->load()) &&
    !context.finished.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (context.finished.load() && (!shutdown_flag || !shutdown_flag->load())) {
    while (dac.isStreamRunning() && (!shutdown_flag || !shutdown_flag->load())) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  if (dac.isStreamOpen()) {
    if (dac.isStreamRunning()) {
      dac.stopStream();
    }
    dac.closeStream();
  }

  if (shutdown_flag && shutdown_flag->load()) {
    return "Playback interrupted by shutdown signal";
  }

  return std::nullopt;
}

std::optional<std::string>
ra_play(
  SndfileHandle fileh, RtAudioFormat ra_format,
  std::atomic<bool> * shutdown_flag)
{
  RtAudio dac;
  return ra_play(fileh, dac, ra_format, shutdown_flag);
}

RaWriteThread::RaWriteThread(
  unsigned int channels,
  unsigned int sample_rate,
  RtAudioFormat format,
  boost::lockfree::spsc_queue<std::vector<uint8_t>> * queue,
  std::atomic<bool> * shutdown_flag,
  std::atomic<bool> * data_available,
  unsigned int buffer_frames)
: channels_(channels),
  sample_rate_(sample_rate),
  format_(format),
  queue_(queue),
  shutdown_flag_(shutdown_flag),
  data_available_(data_available)
{
  sample_bytes_ = sample_size_from_rtaudio_format(format_);
  if (sample_bytes_ <= 0) {
    error_str_ = "Unsupported RtAudio format";
    return;
  }

  dac_ = std::make_unique<RtAudio>();
  if (dac_->getDeviceIds().empty()) {
    error_str_ = "No audio output devices found by RtAudio";
    return;
  }

  RtAudio::StreamParameters oParams;
  oParams.deviceId = dac_->getDefaultOutputDevice();
  oParams.nChannels = channels_;
  oParams.firstChannel = 0;

  unsigned int frames = buffer_frames;
  RtAudio::StreamOptions options;

  RtAudioErrorType err = dac_->openStream(
    &oParams,
    nullptr,
    format_,
    sample_rate_,
    &frames,
    &RaWriteThread::audio_callback,
    this,
    &options);

  if (err != RTAUDIO_NO_ERROR) {
    error_str_ = "Failed to open RtAudio stream: " + dac_->getErrorText();
    return;
  }

  err = dac_->startStream();
  if (err != RTAUDIO_NO_ERROR) {
    error_str_ = "Failed to start RtAudio stream: " + dac_->getErrorText();
    dac_->closeStream();
    return;
  }
}

RaWriteThread::~RaWriteThread()
{
  close();
}

std::string RaWriteThread::get_error() const
{
  return error_str_;
}

bool RaWriteThread::is_open() const
{
  return dac_ && dac_->isStreamOpen();
}

void RaWriteThread::close()
{
  if (dac_ && dac_->isStreamOpen()) {
    if (dac_->isStreamRunning()) {
      dac_->stopStream();
    }
    dac_->closeStream();
  }
}

void RaWriteThread::drain_and_close()
{
  if (dac_ && dac_->isStreamRunning()) {
    // Wait until remaining items in queue are consumed and current chunk is finished
    while ((!shutdown_flag_ || !shutdown_flag_->load()) &&
      ((queue_ && !queue_->empty()) || current_chunk_offset_ < current_chunk_.size()))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    // Small settling delay for hardware/driver buffer
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  close();
}

int RaWriteThread::audio_callback(
  void * output_buffer,
  void * /*input_buffer*/,
  unsigned int n_buffer_frames,
  double /*stream_time*/,
  RtAudioStreamStatus /*status*/,
  void * user_data)
{
  auto * self = static_cast<RaWriteThread *>(user_data);
  if (self->shutdown_flag_ && self->shutdown_flag_->load()) {
    return 2;  // Abort immediately
  }

  uint8_t * out_bytes = static_cast<uint8_t *>(output_buffer);
  size_t bytes_needed = static_cast<size_t>(n_buffer_frames) * self->channels_ *
    self->sample_bytes_;
  size_t bytes_written = 0;

  while (bytes_written < bytes_needed) {
    if (self->current_chunk_offset_ < self->current_chunk_.size()) {
      size_t available = self->current_chunk_.size() - self->current_chunk_offset_;
      size_t to_copy = std::min(available, bytes_needed - bytes_written);
      std::memcpy(
        out_bytes + bytes_written,
        self->current_chunk_.data() + self->current_chunk_offset_,
        to_copy);
      self->current_chunk_offset_ += to_copy;
      bytes_written += to_copy;
    } else {
      if (self->queue_ && self->queue_->pop(self->current_chunk_)) {
        self->current_chunk_offset_ = 0;
      } else {
        // Queue is empty: zero-fill remaining buffer to avoid stutter/noise
        std::memset(out_bytes + bytes_written, 0, bytes_needed - bytes_written);
        break;
      }
    }
  }

  return 0;  // Keep stream alive
}
