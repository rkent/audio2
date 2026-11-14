#include <algorithm>
#include <sndfile.hh>
#include <cstring>
#include <ios>
#include <iterator>
#include <fstream>
#include <cstdio>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <audio2_stream/IAudioDeviceProxy.hpp>
#include <audio2_stream/AudioStream.hpp>
#include <inja/inja.hpp>
#include "subprocess.hpp"

#ifdef _WIN32
#define WHICH_COMMAND "where "
#define NULL_DEVICE "NUL"
#else
#define WHICH_COMMAND "which "
#define NULL_DEVICE "/dev/null"
#endif

static auto rcl_logger = rclcpp::get_logger("audio2_stream/AudioStream");

static bool isProgramInstalled(const std::string & program)
{
  std::string command = WHICH_COMMAND + program + " > " NULL_DEVICE " 2>&1";
  return system(command.c_str()) == 0;
}

static bool hasEspeak()
{
    // Initialized only the first time this function is called
  static bool installed = isProgramInstalled("espeak");
  return installed;
}

static bool hasEspeakNG()
{
    // Initialized only the first time this function is called
  static bool installed = isProgramInstalled("espeak-ng");
  return installed;
}

std::set<int> AudioStream::combine_samplerates()
{
  if (source_ && !sink_) {
    return source_->config_ranges_->samplerate_values;
  }
  if (sink_ && !source_) {
    return sink_->config_ranges_->samplerate_values;
  }
  if (!source_ && !sink_) {
    printf(
      "AudioStream::combine_samplerates returning empty set because source and sink is missing\n");
    return {};
  }
  std::set<int> intersection;
  std::set_intersection(
    sink_->config_ranges_->samplerate_values.begin(),
    sink_->config_ranges_->samplerate_values.end(),
    source_->config_ranges_->samplerate_values.begin(),
    source_->config_ranges_->samplerate_values.end(),
    std::inserter(intersection, intersection.begin())
  );
  return intersection;
}

std::set<int> AudioStream::combine_channels()
{
  if (!sink_ && source_) {
    return source_->config_ranges_->channel_values;
  }
  if (!source_ && sink_) {
    return sink_->config_ranges_->channel_values;
  }
  if (!sink_ && !source_) {
    printf(
      "AudioStream::combine_channels returning empty set because source and sink is missing\n");
    return {};
  }
  std::set<int> intersection;
  std::set_intersection(
    sink_->config_ranges_->channel_values.begin(), sink_->config_ranges_->channel_values.end(),
    source_->config_ranges_->channel_values.begin(), source_->config_ranges_->channel_values.end(),
    std::inserter(intersection, intersection.begin())
  );
  return intersection;
}

std::optional<std::string> AudioStream::merge_parms()
{
  auto samplerates = combine_samplerates();
  auto channels = combine_channels();
  if (samplerates.empty()) {
    return std::string("No compatible samplerate found between source and sink");
  }
  if (channels.empty()) {
    return std::string("No compatible channel count found between source and sink");
  }
  return std::nullopt;
}

bool AudioStream::parms_fixed()
{
  return samplerate_ > 0 && channels_ > 0;
}

std::optional<std::string> AudioStream::fix_parms()
{
  if (source_ && !source_->are_parms_bound_) {
    return std::string("Source parameters not bound");
  }
  if (sink_ && !sink_->are_parms_bound_) {
    return std::string("Sink parameters not bound");
  }
  auto samplerates = combine_samplerates();
  auto channels = combine_channels();
  if (samplerates.empty()) {
    return std::string("No compatible samplerate found between source and sink");
  }
  if (channels.empty()) {
    return std::string("No compatible channel count found between source and sink");
  }

      // Prefer the lower value if more than one.
  channels_ = *channels.begin();

  if (samplerates.size() == 1) {
    samplerate_ = *samplerates.begin();
  } else {
    if (samplerates.contains(PREFERRED_RATE)) {
      samplerate_ = PREFERRED_RATE;
    } else {
          // If multiple compatible rates, just pick the middle one.
      samplerate_ = *std::next(samplerates.begin(), samplerates.size() / 2);
    }
  }
  printf("AudioStream::fix_parms selected samplerate %u, channels %u\n",
    samplerate_, channels_);
  return std::nullopt;
}

std::optional<std::string> DeviceTerminal::open(
  AudioDirection direction,
  AudioStream * audio_stream)
{
    // Create the platform's default proxy implementation if none provided
  printf("DeviceTerminal::open called\n");
  if (!proxy_) {
    proxy_ = make_default_device_proxy();
  }

  if (!audio_stream->parms_fixed()) {
    auto fix_result = audio_stream->fix_parms();
    if (fix_result.has_value()) {
      RCLCPP_ERROR(rcl_logger, "Error fixing audio parameters for DeviceTerminal: %s",
        fix_result->c_str());
      return fix_result;
    }
  }

  DeviceParams params;
  params.device = device_name_;
  params.channels = audio_stream->channels_;
  params.samplerate = audio_stream->samplerate_;
  params.format = sample_format_;

  printf(
    "DeviceTerminal::open called with params: device=%s, channels=%u, samplerate=%u, format=%d, direction=%d\n",
           params.device.c_str(),
           params.channels,
           params.samplerate,
           static_cast<int>(params.format),
           static_cast<int>(direction));

  auto result = proxy_->open(params, direction);
  if (result.has_value()) {
    return result.value();
  }
    // proxy_->open may change the format if original is not supported.
  sample_format_ = proxy_->get_format();
  if (audio_stream->samplerate_ != static_cast<int>(params.samplerate)) {
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "Error: Requested samplerate %u, got %u from audio device.\n",
      audio_stream->samplerate_, params.samplerate);
    return std::string(buffer);
  }
  are_parms_fixed_ = true;
  return std::nullopt;
}

void DeviceTerminal::close()
{
  printf("DeviceTerminal::close called\n");
  if (proxy_) {
    proxy_->close();
  }
  return;
}

void DeviceSink::run(AudioStream * audio_stream)
{
  assert(audio_stream);
  assert(proxy_);
  printf("DeviceSink::run started\n");
  if (audio_stream->shutdown_flag_.load()) {
    printf("DeviceSink::run exiting immediately due to shutdown flag\n");
    return;
  }
  bool done = false;
  while (!done && !audio_stream->shutdown_flag_.load()) {
    std::string error_str = proxy_->get_error();
    if (error_str.length() > 0) {
      printf("DeviceSink::run exiting due to device error: %s\n", error_str.c_str());
      break;
    }
    std::vector<uint8_t> audio_data;

        // Wait to pop from queue
    printf("Waiting to pop from queue in DeviceSink::run at %s\n", format_timestamp().c_str());
    audio_stream->data_available_.wait(false);     // Wait until there's something to process
    std::size_t hash_id = std::hash<std::thread::id>{}(std::this_thread::get_id()) % 10000;
    printf("\nDeviceSink::run thread %zu woke up to process audio data at %s\n", hash_id,
      format_timestamp().c_str());
    audio_stream->data_available_.store(false);

    if (!are_parms_fixed_) {
      printf("DeviceSink::run fixing parameters before processing audio data\n");
      if (!audio_stream->parms_fixed()) {
        auto fix_result = audio_stream->fix_parms();
        if (fix_result.has_value()) {
          RCLCPP_ERROR(rcl_logger, "Error fixing audio parameters for DeviceSink: %s",
            fix_result->c_str());
          break;
        }
      }
      if (!audio_stream->parms_fixed()) {
        RCLCPP_ERROR(rcl_logger, "Audio parameters still not fixed");
        break;
      }
          // ToDo: vary format depending on sfg format
      sample_format_ = DEFAULT_SAMPLE_FORMAT;

      printf("DeviceSink::run opening audio device\n");
      auto open_result = open(AudioDirection::Playback, audio_stream);
      if (open_result.has_value()) {
        RCLCPP_ERROR(rcl_logger, "Failed to open audio device in DeviceSink: %s",
          open_result->c_str());
        return;
      }
      printf("DeviceSink::run audio device opened successfully\n");
    }

        // empty the queue
    do {
      printf("DeviceSink: popping audio data from queue ra: %zu wa: %zu \n",
        audio_stream->queue_.read_available(), audio_stream->queue_.write_available());
      auto pop_result = audio_stream->queue_.pop(audio_data);
      if (!pop_result) {
        break;
      }
      if (audio_data.empty()) {
          // This is the signal to close
        done = true;
        break;
      }
      int bytes_per_sample = sample_format_bytes(sample_format_);
      int write_result = proxy_->write(
                static_cast<int>(audio_data.size() / bytes_per_sample),
                audio_data.data(),
                audio_stream->channels_, // channels
                sample_format_,
                nullptr  // shutdown flag
      );
      if (write_result < 0) {
        printf("Error writing to audio device: %s\n", proxy_->get_error().c_str());
      }
    }  while (true);
  }
  printf("DeviceSink::run exiting, pushing silence.\n");
    // Push silence to fill the buffer before closing
  const int bytes_per_sample = sample_format_bytes(sample_format_);
  int silence_frames = DEVICE_BUFFER_SIZE;
  const int silence_size = silence_frames * audio_stream->channels_ * bytes_per_sample;
  std::vector<uint8_t> silence_buffer(silence_size, 0);
  while (silence_frames > 0) {
    int written = proxy_->write(
            silence_frames * audio_stream->channels_,
            silence_buffer.data(),
            audio_stream->channels_,
            sample_format_,
            nullptr  // no shutdown flag so we can finish writing silence
    );
    if (written < 0) {
      printf("Error writing silence to audio device: %s\n", proxy_->get_error().c_str());
      break;
    }
    printf("Wrote %d silence frames to audio device of %d\n", written / audio_stream->channels_,
      silence_frames);
    silence_frames -= written / audio_stream->channels_;
  }

  close();
  printf("DeviceSink::run final exiting\n");
  audio_stream->shutdown();
  return;
}

void DeviceSource::run(AudioStream * audio_stream)
{
  assert(audio_stream);
  printf("DeviceSource::run started\n");
  std::vector<uint8_t> audio_data;
    // Add capacity to audio_data to hold one queue chunk of audio.
  auto bytes_per_chunk = audio_stream->queue_frames_ * audio_stream->channels_ *
    sample_size_from_sfg_format(SFG_RW_FORMAT);

  while (!audio_stream->shutdown_flag_.load()) {
    printf("DeviceSource::run loop started\n");
    audio_data.resize(bytes_per_chunk);
    if (!proxy_ || !proxy_->is_open()) {
      printf("DeviceSource::run audio device is not open\n");
      break;
    }
    if (audio_stream->queue_.write_available() == 0) {
            // Queue is full, wait briefly
            // TODO: How to handle this?
      printf("Audio queue is full, waiting...");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    printf("DeviceSource: Attempting to read from audio device\n");
    auto read_result = proxy_->read(
            audio_stream->queue_frames_ * audio_stream->channels_,
            audio_data.data(),
            audio_stream->channels_,
            sample_format_,
            &audio_stream->shutdown_flag_);
    if (read_result < 0) {
      printf("Error reading from audio device: %s\n", proxy_->get_error().c_str());
            // TODO: handle error. Stop rather than resize the buffer to a negative size.
      break;
    }
    printf("DeviceSource: Read %d samples from audio device at %s\n", read_result,
      format_timestamp().c_str());
    audio_data.resize(read_result * sample_size_from_sfg_format(SFG_RW_FORMAT));
    if (!audio_stream->queue_.push(audio_data)) {
            // We should not reach here since we checked write_available above
      printf("Audio queue is full, waiting...");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    audio_stream->data_available_.store(true);
    audio_stream->data_available_.notify_one();
    printf("DeviceSource: Pushed %d samples to audio queue at %s\n", read_result,
      format_timestamp().c_str());
  }
  printf("DeviceSource::run exiting.\n");

  close();
  printf("DeviceSource::run final exiting\n");
  return;
}

std::optional<std::string> SndFileSource::open()
{
  sndfileh_ = SndfileHandle(file_path_.c_str());
  if (sndfileh_.error()) {
    return sndfileh_.strError();
  }

  int sf_channels = sndfileh_.channels();
  int sf_samplerate = sndfileh_.samplerate();
  int sndfile_format = sndfileh_.format();
  config_ranges_->samplerate_values = {sf_samplerate};
  config_ranges_->channel_values = {sf_channels};
  are_parms_bound_ = true;
      // TODO: no need to limit the format here.
  //config_ranges_->format_values = {sfg_format_from_sndfile_format(sndfile_format)};
  printf("Opened file %s: channels=%d, samplerate=%d, sndfile_format=0x%X\n",
    file_path_.c_str(), sf_channels, sf_samplerate, sndfile_format);

  return std::nullopt;
}

void SndFileSource::run(AudioStream * audio_stream)
{
  assert(audio_stream);
  printf("SndFileSource::run started\n");
  do {
    auto open_result = open();
    if (open_result.has_value()) {
      RCLCPP_ERROR(rcl_logger, "Cannot open file <%s>: %s", file_path_.c_str(),
          open_result.value().c_str());
      break;
    }
    auto fix_result = audio_stream->fix_parms();
    if (fix_result.has_value()) {
      RCLCPP_ERROR(rcl_logger, "Error fixing audio parameters for file <%s>: %s",
        file_path_.c_str(),
          fix_result.value().c_str());
      break;
    }
    audio_stream->process_fileh(sndfileh_);
  } while (false);

  printf("SndFileSource::run exiting\n");
      // push to queue one last time to signal end of stream
  audio_stream->push_empty_to_queue();
}

void MessageSink::run(AudioStream * audio_stream)
{
  assert(audio_stream);
  printf("MessageSink::run started\n");
  uint32_t sequence_number = 0;
  auto uuid = generate_uuid();

  std::vector<uint8_t> audio_data;    // to hold popped audio data

    // snd_file_write is used to hold the serialized audio chunk including header
    // TODO: determine appropriate buffer size
  std::vector<unsigned char> snd_file_write;
  auto bytes_per_chunk = audio_stream->queue_frames_ * audio_stream->channels_ *
    sample_size_from_sfg_format(SFG_RW_FORMAT);
  size_t file_size = bytes_per_chunk + MAX_HEADER;
  snd_file_write.reserve(file_size);

    // Messages will be sent at a constant rate based on samplerate and buffer size.
  auto duration = std::chrono::microseconds(static_cast<int64_t>(1'000'000.0 *
      audio_stream->queue_frames_ / (audio_stream->samplerate_)));
  auto next_time = std::chrono::steady_clock::now();

    // Wait to pop from queue. After the first, we rely on time only.
  audio_stream->data_available_.wait(false);   // Wait until there's something to process
  audio_stream->data_available_.store(false);

  bool done = false;
  while (!done) {
        // Send a single chunk as a message
    if (audio_stream->queue_.pop(audio_data)) {

            // Create and populate the AudioChunk message
      auto message = std::make_unique<audio2_stream_msgs::msg::AudioChunk>();
      message->header.handle = 0;       // Placeholder handle
      message->header.description = description_;
      message->header.volume = 1.0f;       // Placeholder volume
      message->header.handle = 0;       // Placeholder handle
      message->header.sequence = ++sequence_number;
      message->header.chunk_start_time = rclcpp::Clock().now();
      message->header.uuid = uuid;

            // Create a virtual sound file in memory for topic publish
      auto data_size = audio_data.size();

      VIO_SOUNDFILE_HANDLE m_vio_sndfileh;
      if (auto err = wopen_vio_to_vector(snd_file_write, m_vio_sndfileh, SFG_RW_FORMAT, sfFormat_,
        audio_stream->channels_, audio_stream->samplerate_, audio_stream->queue_frames_))
      {
        printf("Failed to open sound file for writing to buffer: %s\n", err->c_str());
        return;
      }

            //if (data_size > snd_file_write.capacity() - MAX_HEADER) {
            //    RCLCPP_WARN(rcl_logger, "Audio data size %zu exceeds buffer capacity %zu, resizing", data_size, snd_file_write.capacity() - MAX_HEADER);
            //    snd_file_write.reserve(data_size + MAX_HEADER);
            //}

      auto data_samples = static_cast<int>(data_size) / sample_size_from_sfg_format(SFG_RW_FORMAT);
      int samples_written = sfg_write_convert(m_vio_sndfileh.fileh, SFG_RW_FORMAT, SFG_RW_FORMAT,
                reinterpret_cast<char *>(audio_data.data()), data_samples);
      if (samples_written != data_samples) {
        RCLCPP_ERROR(rcl_logger,
          "Error writing audio data to virtual sound file: expected %d samples, wrote %d samples",
          data_samples, samples_written);
        return;
      }

            // Copy the serialized data into message.data
      message->data.reserve(samples_written * sample_size_from_sfg_format(SFG_RW_FORMAT) +
        MAX_HEADER);
      std::copy(snd_file_write.data(), snd_file_write.data() + m_vio_sndfileh.vio_data.length,
                      std::back_inserter(message->data));

            // Publish the message
      publisher_->publish(std::move(message));

            // print info about the data
      auto time_since_epoch = next_time.time_since_epoch();
      auto hours = std::chrono::duration_cast<std::chrono::hours>(time_since_epoch) % 24;
      auto minutes = std::chrono::duration_cast<std::chrono::minutes>(time_since_epoch) % 60;
      auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch) % 60;
      auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch) %
        1000;
      std::size_t hash_id = std::hash<std::thread::id>{}(std::this_thread::get_id()) % 10000;
      printf("MessageSink thread %zu sending audio chunk of %zu bytes sequence %d at %s\n",
                   hash_id, audio_data.size(), sequence_number, format_timestamp().c_str());
    } else {
            // No data available. Maybe shutdown was requested.
      if (audio_stream->shutdown_flag_.load()) {
        done = true;
      } else {
        printf("MessageSink: no audio data available in queue, but continuing\n");
      }
    }
    next_time += duration;
    std::this_thread::sleep_until(next_time);
  }

    // Send a last message indicating end of stream.
  auto message = std::make_unique<audio2_stream_msgs::msg::AudioChunk>();
  message->header.handle = 0;   // Placeholder handle
  message->header.description = description_;
  message->header.volume = 1.0f;   // Placeholder volume
  message->header.handle = 0;   // Placeholder handle
  message->header.sequence = message->header.SEQUENCE_EOS;
  message->header.chunk_start_time = rclcpp::Clock().now();
  message->header.uuid = uuid;

  publisher_->publish(std::move(message));

  printf("MessageSink::run exiting\n");
}

void MessageSource::run([[maybe_unused]] AudioStream * audio_stream)
{
    // This is just a placeholder, no need for a separate thread as ros2 handles callbacks.
  printf("MessageSource::run exiting (not needed)\n");
}

void MessageSource::callback(
  const audio2_stream_msgs::msg::AudioChunk::SharedPtr message,
  AudioStream * audio_stream
)
{
  assert(audio_stream);
  std::size_t hash_id = std::hash<std::thread::id>{}(std::this_thread::get_id()) % 10000;
  printf("MessageSource: thread %zu received audio chunk with %zu bytes sequence %d at %s\n",
    hash_id, message->data.size(), message->header.sequence, format_timestamp().c_str());
  if ((message->header.sequence == message->header.SEQUENCE_EOS)) {
    printf("MessageSource received end-of-stream message\n");
    audio_stream->shutdown();
    return;
  }

  VIO_SOUNDFILE_HANDLE vio_handle;

  if (auto err = ropen_vio_from_vector(message->data, vio_handle)) {
    RCLCPP_ERROR(rcl_logger, "Failed to open sound file for reading from buffer: %s", err->c_str());
    return;
  }

  auto r_format = sfg_format_from_sndfile_format(vio_handle.fileh.format());
  auto w_format = audio_stream->sink_->rw_format_;
  std::vector<uint8_t> r_buffer;
  std::vector<uint8_t> w_buffer;
  auto result = create_convert_vectors(r_format, w_format,
    audio_stream->queue_frames_ * vio_handle.fileh.channels(), r_buffer, w_buffer);
  if (result.has_value()) {
    RCLCPP_ERROR(rcl_logger, "Failed to create convert vectors: %s", result.value().c_str());
    return;
  }
  bool done = false;
  printf("MessageSource read: length %zu bytes from audio chunk\n", vio_handle.vio_data.length);
  while (!(audio_stream->shutdown_flag_.load()) && !done) {
    int samples_read = sfg_read(vio_handle.fileh, r_format, r_buffer.data(),
      audio_stream->queue_frames_ * vio_handle.fileh.channels());
    if (samples_read <= 0) {
      done = true;
      break;       // End of file or error
    }
    printf("MessageSource: read %d samples from audio chunk at %s\n", samples_read,
      format_timestamp().c_str());
    int samples_converted = convert_types(r_format, w_format, r_buffer.data(), w_buffer.data(),
      samples_read);
    if (samples_converted < 0) {
      RCLCPP_ERROR(rcl_logger, "Error converting audio data for streaming: %d", samples_converted);
      done = true;
      break;
    } else if (samples_converted != samples_read) {
      RCLCPP_ERROR(rcl_logger, "Mismatch in converted samples count: expected %d, got %d",
        samples_read, samples_converted);
      done = true;
      break;
    }

        // Resize buffer to actual converted sample count to avoid pushing garbage data
    w_buffer.resize(samples_converted * sample_size_from_sfg_format(w_format));

        // Try to push to the queue without blocking.
        // If the queue is full, drop this chunk to avoid blocking the ROS2 executor.
        // Blocking here would prevent other callbacks from running and cause message delivery delays.
    if (!audio_stream->queue_.push(w_buffer)) {
      RCLCPP_WARN(rcl_logger,
        "Audio queue is full, dropping audio chunk to avoid blocking executor!");
            // TODO: move this to another thread to avoid blocking the callback?
            // Still notify in case the consumer is waiting
      audio_stream->data_available_.store(true);
      audio_stream->data_available_.notify_one();
      continue;
    }
    printf("MessageSource: Pushed %d samples to audio queue ra: %lu wa: %lu at %s\n",
      samples_converted, audio_stream->queue_.read_available(),
      audio_stream->queue_.write_available(), format_timestamp().c_str());
    audio_stream->data_available_.store(true);
    audio_stream->data_available_.notify_one();
  }
  printf("MessageSource: callback exiting at %s\n", format_timestamp().c_str());
}

std::optional<std::string> AudioStream::start()
{
  // Do an initial check to make sure source and sink are compatible before starting threads
  auto merge_parms_result = merge_parms();
  if (merge_parms_result.has_value()) {
    return merge_parms_result.value();
  }
  printf("AudioStream::start thread %zu called for <%s> at %s\n",
    std::hash<std::thread::id>{}(std::this_thread::get_id()) % 10000, description_.c_str(),
    format_timestamp().c_str());
  if (sink_) {
    sink_thread_ = std::make_unique<std::jthread>(&AudioTerminal::run, sink_.get(), this);
  }
  if (source_) {
    source_thread_ = std::make_unique<std::jthread>(&AudioTerminal::run, source_.get(), this);
  }
  return std::nullopt;
}

void AudioStream::shutdown()
{
  printf("AudioStream::shutdown called for <%s> at %s\n", description_.c_str(),
    format_timestamp().c_str());
  shutdown_flag_.store(true);
  data_available_.store(true);
  data_available_.notify_all();

    // Avoid deadlock: don't join a thread from within itself
  auto current_thread_id = std::this_thread::get_id();

  if (source_thread_ && source_thread_->joinable()) {
    if (source_thread_->get_id() != current_thread_id) {
      source_thread_->join();
    } else {
      printf("AudioStream::shutdown: skipping source_thread join (called from source thread)\n");
    }
  }
  if (sink_thread_ && sink_thread_->joinable()) {
    if (sink_thread_->get_id() != current_thread_id) {
      sink_thread_->join();
    } else {
      printf("AudioStream::shutdown: skipping sink_thread join (called from sink thread)\n");
    }
  }
  printf("AudioStream: sending shutdown_complete for %s at %s\n", description_.c_str(),
    format_timestamp().c_str());
  shutdown_complete_.store(true);
}

void AudioStream::process_fileh(SndfileHandle & fileh)
{
  printf("AudioStream::process_fileh called for <%s> at %s\n", description_.c_str(),
    format_timestamp().c_str());
  int samplerate = fileh.samplerate();
    // Messages will be sent at a constant rate based on samplerate and buffer size.
  auto duration = std::chrono::microseconds(static_cast<int64_t>(1'000'000.0 *
      queue_frames_ / (samplerate)));

        // These were probably set during the open, but reset here to be sure.
  printf(
    "AudioStream::process_fileh setting samplerate %d and channels %d for source config ranges\n",
    samplerate, fileh.channels());
  if (source_) {
    source_->config_ranges_->samplerate_values = {fileh.samplerate()};
    source_->config_ranges_->channel_values = {fileh.channels()};
    source_->are_parms_bound_ = true;
  }
  if (!parms_fixed()) {
    auto fix_result = fix_parms();
    if (fix_result.has_value()) {
      RCLCPP_ERROR(rcl_logger, "Error fixing audio parameters for file source: %s",
        fix_result->c_str());
      return;
    }
  }
  auto r_format = sfg_format_from_sndfile_format(fileh.format());
  auto w_format = (sink_) ? sink_->rw_format_ : SFG_FLOAT;
  printf("AudioStream::process_fileh: file format %d, r_format %d, w_format %d\n",
    fileh.format(), r_format, w_format);
  std::vector<uint8_t> r_buffer;
  std::vector<uint8_t> w_buffer;
  auto result = create_convert_vectors(r_format, w_format,
    queue_frames_ * fileh.channels(), r_buffer, w_buffer);
  if (result.has_value()) {
    RCLCPP_ERROR(rcl_logger, "Failed to create convert vectors: %s", result.value().c_str());
    return;
  }

  bool done = false;
  printf("AudioStream::process_fileh: length %zu bytes from audio chunk\n", fileh.frames() *
    fileh.channels() * sample_size_from_sfg_format(r_format));
  auto next_time = std::chrono::steady_clock::now();
  while (!(shutdown_flag_.load()) && !done) {
    int samples_read = sfg_read(fileh, r_format, r_buffer.data(),
      queue_frames_ * fileh.channels());
    if (samples_read <= 0) {
      done = true;
      break;       // End of file or error
    }
    printf("AudioStream::process_fileh: read %d samples from audio chunk at %s\n", samples_read,
      format_timestamp().c_str());
    int samples_converted = convert_types(r_format, w_format, r_buffer.data(), w_buffer.data(),
      samples_read);
    if (samples_converted < 0) {
      RCLCPP_ERROR(rcl_logger, "Error converting audio data for streaming: %d", samples_converted);
      done = true;
      break;
    } else if (samples_converted != samples_read) {
      RCLCPP_ERROR(rcl_logger, "Mismatch in converted samples count: expected %d, got %d",
        samples_read, samples_converted);
      done = true;
      break;
    }

        // Resize buffer to actual converted sample count to avoid pushing garbage data
    w_buffer.resize(samples_converted * sample_size_from_sfg_format(w_format));

        // Try to push to the queue without blocking.
    if (!queue_.push(w_buffer)) {
      RCLCPP_WARN(rcl_logger,
        "Audio queue is full, dropping audio chunk to avoid blocking executor!");
            // TODO: move this to another thread to avoid blocking the callback?
            // Still notify in case the consumer is waiting
      data_available_.store(true);
      data_available_.notify_one();
      continue;
    }
    printf("AudioStream::process_fileh: Pushed %d samples to audio queue ra: %lu wa: %lu at %s\n",
      samples_converted, queue_.read_available(),
      queue_.write_available(), format_timestamp().c_str());
    data_available_.store(true);
    data_available_.notify_one();
    next_time += duration;
    std::this_thread::sleep_until(next_time);

  }
}

void AudioStream::process_raw(std::vector<uint8_t> & audio_data, SfgRwFormat r_format)
{
    // Messages will be sent at a constant rate based on samplerate and buffer size.
  auto duration = std::chrono::microseconds(static_cast<int64_t>(1'000'000.0 *
      queue_frames_ / (samplerate_)));

  if (!parms_fixed()) {
    auto fix_result = fix_parms();
    if (fix_result.has_value()) {
      RCLCPP_ERROR(rcl_logger, "Error fixing audio parameters for file source: %s",
        fix_result->c_str());
      return;
    }
  }

  auto w_format = sink_->rw_format_;
  std::vector<uint8_t> w_buffer;

  auto r_sample_size = sample_size_from_sfg_format(r_format);
  auto w_sample_size = sample_size_from_sfg_format(w_format);

  // Calculate chunk size in samples for each iteration
  int chunk_samples = queue_frames_ * channels_;
  int chunk_bytes = chunk_samples * r_sample_size;

  // Resize write buffer to hold converted chunk
  w_buffer.reserve(chunk_samples * w_sample_size);
  bool done = false;
  printf("AudioStream::process_raw: length %zu bytes from audio chunk\n", audio_data.size());
  auto next_time = std::chrono::steady_clock::now();

  size_t offset = 0;
  while (!(shutdown_flag_.load()) && !done) {
    size_t remaining_bytes = audio_data.size() - offset;
    if (remaining_bytes == 0) {
      done = true;
      break;
    }

    int bytes_to_read = std::min(static_cast<size_t>(chunk_bytes), remaining_bytes);
    int samples_read = bytes_to_read / r_sample_size;

    if (samples_read <= 0) {
      done = true;
      break;       // End of data
    }
    printf("AudioStream::process_raw: read %d samples from audio chunk at offset %zu at %s\n",
      samples_read, offset, format_timestamp().c_str());

    // Convert from the current position in audio_data
    int samples_converted = convert_types(r_format, w_format,
      audio_data.data() + offset, w_buffer.data(), samples_read);

    if (samples_converted < 0) {
      RCLCPP_ERROR(rcl_logger, "Error converting audio data for streaming: %d", samples_converted);
      done = true;
      break;
    } else if (samples_converted != samples_read) {
      RCLCPP_ERROR(rcl_logger, "Mismatch in converted samples count: expected %d, got %d",
        samples_read, samples_converted);
      done = true;
      break;
    }

        // Resize buffer to actual converted sample count to avoid pushing garbage data
    w_buffer.resize(samples_converted * sample_size_from_sfg_format(w_format));

        // Try to push to the queue without blocking.
    if (!queue_.push(w_buffer)) {
      RCLCPP_WARN(rcl_logger,
        "Audio queue is full, dropping audio chunk to avoid blocking executor!");
            // TODO: move this to another thread to avoid blocking the callback?
            // Still notify in case the consumer is waiting
      data_available_.store(true);
      data_available_.notify_one();
      offset += bytes_to_read;
      continue;
    }
    printf("AudioStream::process_raw: Pushed %d samples to audio queue ra: %lu wa: %lu at %s\n",
      samples_converted, queue_.read_available(),
      queue_.write_available(), format_timestamp().c_str());
    data_available_.store(true);
    data_available_.notify_one();

    // Advance offset for next chunk
    offset += bytes_to_read;

    next_time += duration;
    std::this_thread::sleep_until(next_time);
  }
  // push to queue one last time to signal end of stream
  push_empty_to_queue();
}

void AudioStream::push_empty_to_queue()
{
  std::vector<uint8_t> empty_buffer;
      // todo: what if queue is full?
  queue_.push(empty_buffer);
  data_available_.store(true);
  data_available_.notify_one();
}

// Callback function for CURL to write response data
static size_t write_callback(void * contents, size_t size, size_t nmemb, void * userp)
{
  printf("CURL write_callback called with nmemb %zu at %s\n", nmemb,
    format_timestamp().c_str());
  size_t realsize = size * nmemb;
  auto * buffer = static_cast<std::vector<uint8_t> *>(userp);

  uint8_t * ptr = static_cast<uint8_t *>(contents);
  buffer->insert(buffer->end(), ptr, ptr + realsize);

  return realsize;
}

// Helper function to split a string by a delimiter
/*
static std::vector<std::string> split_string(const std::string & str, char delimiter)
{
  std::vector<std::string> result;
  std::string current;
  for (char c : str) {
    if (c == delimiter) {
      if (!current.empty()) {
        result.push_back(current);
        current.clear();
      }
    } else {
      current += c;
    }
  }
  if (!current.empty()) {
    result.push_back(current);
  }
  return result;
}
*/

// ToDo: Do not limit speed if writing to a file.
std::optional<std::string> TtsSource::open()
{
  RCLCPP_INFO(rcl_logger, "TTS source opening");
  inja::Environment env;
  env.add_callback("env", 1, [](inja::Arguments & args) {
      std::string var_name_str = args.at(0)->get<std::string>();
      const char * var_name = var_name_str.c_str();
      const char * var_value = std::getenv(var_name);
      return std::string(var_value ? var_value : "");
  });

  nlohmann::json json_payload;
  nlohmann::json vars_json;
  if (!model_.empty()) {
    vars_json["model"] = model_;
  }
  if (!tts_format_.empty()) {
    vars_json["format"] = tts_format_;
  }
  if (!voice_.empty()) {
    vars_json["voice"] = voice_;
  }
  vars_json["text"] = text_;

  if (name_.empty()) {
    name_ = "espeak";
  }
  if (text_.empty()) {
    return std::string("TTS text cannot be empty");
  }

  if (name_ == "espeak") {
    printf("TtsSource::open using espeak TTS at %s\n", format_timestamp().c_str());
    tts_method_ = TtsMethod::TTS_PROGRAM_WAV;
    // samplerate = 48000;
  } else if (name_ == "piper") {
    printf("TtsSource::open using Piper TTS at %s\n", format_timestamp().c_str());
    tts_method_ = TtsMethod::TTS_PROGRAM_RAW;
    if (voice_.empty()) {
      voice_ = "en_US-amy-medium";
    }
    int samplerate = (voice_.size() >= 3 &&
      voice_.substr(voice_.size() - 3) == "low") ? 16000 : 22050;
    config_ranges_->samplerate_values = {samplerate};
    config_ranges_->channel_values = {1};
    are_parms_bound_ = true;

  } else {
    // Search for template with name in package share directory
    auto template_path = ament_index_cpp::get_package_share_directory("audio2_stream") +
      "/templates/tts/" + name_ + ".json";
    if (!std::filesystem::exists(template_path)) {
      return std::string("Unsupported TTS provider ") + name_ + ", template file not found: " +
             template_path;
    }
    tts_method_ = TtsMethod::TTS_TEMPLATE;
    printf("Loading template from path: %s\n", template_path.c_str());
    inja::Template temp = env.parse_template(template_path);
    std::string rendered = env.render(temp, vars_json);
    printf("Rendered template: %s\n", rendered.c_str());

    template_rendered_json_ = nlohmann::json::parse(rendered);

    // Tests
    if (!template_rendered_json_["url"].is_string()) {
      return std::string("Template rendering failed: url is not a string");
    } else if (!template_rendered_json_["headers"].is_array()) {
      return std::string("Template rendering failed: headers is not an array");
    } else if (!template_rendered_json_["body"].is_object()) {
      return std::string("Template rendering failed: body is not an object");
    }
  }
  json_str_ = json_payload.dump();
  headers_ = curl_slist_append(headers_, "Content-Type: application/json");
  return std::nullopt;
}

std::optional<std::string> TtsSource::fetch_tts_program(std::vector<uint8_t> & audio_data)
{
  // The text is passed on stdin and the program is run without a shell, so the text and
  // voice (both from the request message) can never be interpreted as commands.
  std::vector<std::string> argv;
  if (name_ == "espeak") {
    if (hasEspeakNG()) {
      argv = {"espeak-ng", "--stdout", "--stdin"};
    } else if (hasEspeak()) {
      argv = {"espeak", "--stdout", "--stdin"};
    } else {
      return std::string("Neither espeak nor espeak-ng is installed");
    }
  } else if (name_ == "piper") {
    if (!isProgramInstalled("piper")) {
      return std::string("Piper TTS program is not installed");
    }
    // With no text arguments, piper reads the text from stdin
    argv = {"piper", "--output-raw", "-m", voice_, "--data-dir", PIPER_DATA_DIR};
  } else {
    return std::string("Unsupported TTS provider: ") + name_;
  }
  std::string command;
  for (const auto & arg : argv) {
    command += (command.empty() ? "" : " ") + arg;
  }
  printf("TtsSource::fetch_tts_program executing command: %s (text on stdin) at %s\n",
    command.c_str(), format_timestamp().c_str());
  // TODO: we should stream this data to the audio stream as it comes in, rather than buffering
  // the whole thing in memory.
  int exit_code = 0;
  if (auto error = run_process(argv, text_, audio_data, exit_code)) {
    return std::string("Failed to execute TTS command: ") + *error;
  }
  if (exit_code != 0) {
    return std::format("TTS command failed with code {}", exit_code);
  }
  printf("TtsSource::fetch_tts_program completed with %zu bytes of audio data at %s\n",
    audio_data.size(), format_timestamp().c_str());
  return std::nullopt;
}

std::optional<std::string> TtsSource::fetch_tts_template(std::vector<uint8_t> & audio_data)
{
  printf("TtsSource::fetch_tts_template called at %s\n", format_timestamp().c_str());

  CURL * curl = curl_easy_init();
  if (!curl) {
    return std::string("Failed to initialize CURL");
  }
  //curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);

  // Set CURL options
  std::string template_url = template_rendered_json_["url"].get<std::string>();
  curl_easy_setopt(curl, CURLOPT_URL, template_url.c_str());

  struct curl_slist * headers = nullptr;
  for (const auto & header : template_rendered_json_["headers"]) {
    std::string header_str = header.get<std::string>();
    headers = curl_slist_append(headers, header_str.c_str());
    printf("TtsSource::fetch_tts_template added header: %s\n", header_str.c_str());
  }
  headers = curl_slist_append(headers, "Content-Type: application/json");
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

  std::string template_body = template_rendered_json_["body"].dump();
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, template_body.c_str());
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);

  // Set up response buffer
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, static_cast<void *>(&audio_data));

  // Perform request
  printf("TtsSource: Sending TTS request to %s at %s\n", name_.c_str(), format_timestamp().c_str());
  CURLcode res = curl_easy_perform(curl);
  printf("TtsSource: Received TTS response from %s at %s\n", name_.c_str(),
    format_timestamp().c_str());
  curl_slist_free_all(headers);

  if (res != CURLE_OK) {
    std::string error_msg = std::string("CURL error: ") + curl_easy_strerror(res);
    curl_easy_cleanup(curl);
    return error_msg;
  }

  // Check HTTP response code
  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  if (http_code != 200) {
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "HTTP error code: %ld", http_code);
    curl_easy_cleanup(curl);
    return std::string(buffer);
  }

  curl_easy_cleanup(curl);
  return std::nullopt;
}

void TtsSource::run(AudioStream * audio_stream)
{
  assert(audio_stream);
  printf("TtsSource::run started at %s\n", format_timestamp().c_str());

  // TODO: add an option to normalize the volume of results.
  do {
    auto open_result = open();
    if (open_result.has_value()) {
      RCLCPP_ERROR(rcl_logger, "TtsSource: Failed to open TTS source: %s\n",
        open_result.value().c_str());
      break;
    }
    std::vector<uint8_t> audio_data;
        // Reserve some data. CURL will expand as needed.
    audio_data.reserve(CURL_MAX_WRITE_SIZE);
    if (tts_method_ == TtsMethod::TTS_PROGRAM_WAV || tts_method_ == TtsMethod::TTS_PROGRAM_RAW) {
      auto fetch_result = fetch_tts_program(audio_data);
      if (fetch_result.has_value()) {
        RCLCPP_ERROR(rcl_logger, "TtsSource: Error fetching TTS audio: %s\n",
          fetch_result.value().c_str());
        break;
      }
    } else if (tts_method_ == TtsMethod::TTS_TEMPLATE) {
      auto fetch_result = fetch_tts_template(audio_data);
      if (fetch_result.has_value()) {
        RCLCPP_ERROR(rcl_logger, "TtsSource: Error fetching TTS audio: %s\n",
          fetch_result.value().c_str());
        break;
      }
    } else {
      RCLCPP_ERROR(rcl_logger, "TtsSource: Unsupported TTS method\n");
      break;
    }
    if (audio_data.empty()) {
      RCLCPP_ERROR(rcl_logger, "TtsSource: No audio data received\n");
      break;
    }

    RCLCPP_INFO(rcl_logger, "TtsSource: Received %zu bytes of audio data\n", audio_data.size());

    if (tts_method_ == TtsMethod::TTS_PROGRAM_RAW) {
        // If the data is raw, we can process it directly without the virtual file.
        // TODO: we need to know the format of the raw data. For now we assume it's PCM_16, 1 channel.
      audio_stream->process_raw(audio_data, SFG_SHORT);
    } else {
        // Convert the audio data (which is a file content) to the stream format
      VIO_SOUNDFILE_HANDLE vio_handle;
      if (auto err = ropen_vio_from_vector(audio_data, vio_handle)) {
        RCLCPP_ERROR(rcl_logger, "TtsSource: Failed to open sound file from TTS audio data: %s\n",
          err->c_str());
        break;
      }
      printf("TtsSource: Opened virtual sound file from TTS audio data, length %zu bytes at %s\n",
        vio_handle.vio_data.length, format_timestamp().c_str());
      audio_stream->process_fileh(vio_handle.fileh);
    }
  } while (false);
      // push to queue one last time to signal end of stream
  audio_stream->push_empty_to_queue();
  RCLCPP_INFO(rcl_logger, "TtsSource::run completed at %s\n", format_timestamp().c_str());
}
