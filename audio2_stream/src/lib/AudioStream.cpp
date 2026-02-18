#include <algorithm>
#include <sndfile.hh>
#include <cstring>
#include <ios>
#include <iterator>
#include <fstream>
#include <cstdio>
#include <audio2_stream/AudioStream.hpp>
#include <audio2_stream/AlsaDeviceImpl.hpp>
#include "nlohmann/json.hpp"

static auto rcl_logger = rclcpp::get_logger("audio2_stream/AudioStream");

static bool isProgramInstalled(const std::string& program) {
    std::string command = "which " + program + " > /dev/null 2>&1";
    return system(command.c_str()) == 0;
}

static bool hasEspeak() {
    // Initialized only the first time this function is called
    static bool installed = isProgramInstalled("espeak");
    return installed;
}

static bool hasEspeakNG() {
    // Initialized only the first time this function is called
    static bool installed = isProgramInstalled("espeak-ng");
    return installed;
}

std::set<int> AudioStream::combine_samplerates() {
  if (!sink_) {
    return source_->config_ranges_->samplerate_values;
  }
  if (!source_) {
    return sink_->config_ranges_->samplerate_values;
  }
  std::set<int> intersection;
  std::set_intersection(
    sink_->config_ranges_->samplerate_values.begin(), sink_->config_ranges_->samplerate_values.end(),
    source_->config_ranges_->samplerate_values.begin(), source_->config_ranges_->samplerate_values.end(),
    std::inserter(intersection, intersection.begin())
  );
  return intersection;
}

std::set<int> AudioStream::combine_channels() {
  if (!sink_) {
    return source_->config_ranges_->channel_values;
  }
  if (!source_) {
    return sink_->config_ranges_->channel_values;
  }
  std::set<int> intersection;
  std::set_intersection(
    sink_->config_ranges_->channel_values.begin(), sink_->config_ranges_->channel_values.end(),
    source_->config_ranges_->channel_values.begin(), source_->config_ranges_->channel_values.end(),
    std::inserter(intersection, intersection.begin())
  );
  return intersection;
}

std::set<SfgRwFormat> AudioStream::combine_formats() {
  if (!sink_) {
    return source_->config_ranges_->format_values;
  }
  if (!source_) {
    return sink_->config_ranges_->format_values;
  }
  std::set<SfgRwFormat> intersection;
  std::set_intersection(
    sink_->config_ranges_->format_values.begin(), sink_->config_ranges_->format_values.end(),
    source_->config_ranges_->format_values.begin(), source_->config_ranges_->format_values.end(),
    std::inserter(intersection, intersection.begin())
  );
  return intersection;
}

std::optional<std::string> AudioStream::merge_parms() {
  auto samplerates = combine_samplerates();
  auto channels = combine_channels();
  auto formats = combine_formats();
  if (samplerates.empty()) {
    return std::string("No compatible samplerate found between source and sink");
  }
  if (channels.empty()) {
    return std::string("No compatible channel count found between source and sink");
  }
  if (formats.empty()) {
    return std::string("No compatible format found between source and sink");
  }
  return std::nullopt;
}

std::optional<std::string> AudioStream::fix_parms() {
  auto samplerates = combine_samplerates();
  if (samplerates.size() == 1) {
    printf("Available samplerate: %d\n", *samplerates.begin());
  } else {
    printf("Available samplerates: ");
    for (auto rate : samplerates) {
      printf("%d ", rate);
    }
    printf("\n");
  }
  auto channels = combine_channels();
  auto formats = combine_formats();
  if (formats.size() == 1) {
    printf("Available format: %d\n", static_cast<int>(*formats.begin()));
  } else {
    printf("Available formats: ");
    for (auto format : formats) {
      printf("%d ", static_cast<int>(format));
    }
    printf("\n");
  }
  if (samplerates.empty()) {
    return std::string("No compatible samplerate found between source and sink");
  }
  if (channels.empty()) {
    return std::string("No compatible channel count found between source and sink");
  }
  if (formats.empty()) {
    return std::string("No compatible format found between source and sink");
  }

  if (formats.contains(SFG_RW_FORMAT)) {
    rw_format_ = SFG_RW_FORMAT;
  } else {
        // The formats are ordered by preference in the enum, so just pick the first one.
    rw_format_ = *formats.begin();
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
  printf("AudioStream::fix_parms selected samplerate %u, channels %u, format %d\n",
    samplerate_, channels_, rw_format_);
  return std::nullopt;
}

std::optional<std::string> AlsaTerminal::open(snd_pcm_stream_t direction)
{
    // Create default device implementation if none provided
  if (!alsa_device_) {
    alsa_device_ = std::make_unique<AlsaDeviceImpl>();
  }

  AlsaHwParams hw_params;
  hw_params.device = alsa_device_name_.c_str();
  hw_params.channels = channels_;
  hw_params.samplerate = samplerate_;
  hw_params.format = alsa_format_;
  hw_params.direction = direction;

  printf("AlsaTerminal::open called at %s\n", format_timestamp().c_str());
  printf(
    "AlsaTerminal::open called with hw_params: device=%s, channels=%u, samplerate=%u, format=%d, direction=%d\n",
           hw_params.device,
           hw_params.channels,
           hw_params.samplerate,
           hw_params.format,
           hw_params.direction);

  AlsaSwParams sw_params;

  auto result = alsa_device_->open(hw_params, sw_params, direction);
  printf("AlsaTerminal::open completed at %s\n", format_timestamp().c_str());
  if (result.has_value()) {
    return result.value();
  }
    // alsa_device_->open may change the format if original is not supported.
  alsa_format_ = alsa_device_->get_format();
  if (samplerate_ != hw_params.samplerate) {
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "Error: Requested samplerate %u, got %u from ALSA device.\n",
      samplerate_, hw_params.samplerate);
    return std::string(buffer);
  }
  is_open_ = true;
  return std::nullopt;
}

void AlsaTerminal::close()
{
  printf("AlsaTerminal::close called\n");
  if (alsa_device_) {
    alsa_device_->close();
  }
  return;
}

std::optional<std::string> AlsaSink::fix_rate(unsigned int rate)
{
  if (rate == samplerate_) {
    printf("AlsaSink::fix_rate: no sample rate change needed: %u Hz\n", rate);
    return std::nullopt; // No change needed
  }
  snd_pcm_t* handle = alsa_device_->get_handle();
  auto result = alsa_fix_rate(rate, handle);
  if (result.has_value()) {
    printf("AlsaSink::fix_rate: error fixing sample rate: %s\n", result->c_str());
  } else {
    printf("AlsaSink::fix_rate: sample rate fixed to %u Hz\n", rate);
    samplerate_ = rate;
  }
  return result;
}

void AlsaSink::run(AudioStream * audio_stream)
{
  assert(audio_stream);
  assert(alsa_device_);
  printf("AlsaSink::run started\n");
  if (audio_stream->shutdown_flag_.load()) {
    printf("AlsaSink::run exiting immediately due to shutdown flag\n");
    return;
  }
  while (!audio_stream->shutdown_flag_.load()) {
    std::string error_str = alsa_device_->get_error();
    if (error_str.length() > 0) {
      printf("AlsaSink::run exiting due to ALSA error: %s\n", error_str.c_str());
      break;
    }
    //if (!alsa_device_->get_handle()) {
    //  printf("AlsaSink::run exiting because ALSA device is not open\n");
    //  break;
    //}
    std::vector<uint8_t> audio_data;

        // Wait to pop from queue
    audio_stream->data_available_.wait(false);     // Wait until there's something to process
    std::size_t hash_id = std::hash<std::thread::id>{}(std::this_thread::get_id()) % 10000;
    printf("\nAlsaSink::run thread %zu woke up to process audio data at %s\n", hash_id,
      format_timestamp().c_str());
    audio_stream->data_available_.store(false);
    if (!is_open_) {
      //auto result = audio_stream->fix_parms();
      //if (result.has_value()) {
      //  RCLCPP_ERROR(rcl_logger, "Failed to fix parameters in AlsaSink: %s", result->c_str());
      //  return;
      //}
      if (audio_stream->rw_format_ == SFG_NONE) {
        RCLCPP_ERROR(rcl_logger, "Audio format not set in AlsaSink");
        return;
      } else if (audio_stream->rw_format_ == SFG_INVALID) {
        RCLCPP_ERROR(rcl_logger, "Audio format in AlsaSink is invalid");
        return;
      }
          // ToDo: vary format depending on sfg format
      alsa_format_ = ALSA_FORMAT;

      if (audio_stream->samplerate_ == 0 || audio_stream->channels_ == 0) {
        RCLCPP_ERROR(rcl_logger, "Audio parameters not set in AlsaSink: samplerate %u, channels %u",
          audio_stream->samplerate_, audio_stream->channels_);
        return;
      }
      samplerate_ = audio_stream->samplerate_;
      channels_ = audio_stream->channels_;
      printf("AlsaSink::run opening ALSA device\n");
      auto open_result = open(SND_PCM_STREAM_PLAYBACK);
      if (open_result.has_value()) {
        RCLCPP_ERROR(rcl_logger, "Failed to open ALSA device in AlsaSink: %s", open_result->c_str());
        return;
      }
      printf("AlsaSink::run ALSA device opened successfully\n");
      is_open_ = true;
    }

        // empty the queue
    do{
      printf("AlsaSink: popping audio data from queue ra: %zu wa: %zu \n",
        audio_stream->queue_.read_available(), audio_stream->queue_.write_available());
      auto pop_result = audio_stream->queue_.pop(audio_data);
      if (!pop_result) {
        break;
      }
      int bytes_per_sample = snd_pcm_format_width(alsa_format_) / 8;
      if (true) {
                // Output ALSA status for debugging
        snd_pcm_t * alsa_dev = alsa_device_->get_handle();
        if (alsa_dev) {
          snd_pcm_status_t * stat;
          snd_pcm_status_alloca(&stat);
          snd_pcm_status(alsa_dev, stat);
          snd_output_t *output;
          snd_output_stdio_attach(&output, stdout, 0);
          snd_pcm_status_dump(stat, output);
        }
      }
      int write_result = alsa_device_->write(
                static_cast<int>(audio_data.size() / bytes_per_sample),
                audio_data.data(),
                channels_, // channels
                alsa_format_,
                nullptr  // shutdown flag
      );
      if (write_result < 0) {
        printf("Error writing to ALSA device: %s\n", snd_strerror(write_result));
      }
    }  while (true);
  }
  printf("AlsaSink::run exiting, pushing silence.\n");
    // Push silence to fill the buffer before closing
  const int bytes_per_sample = snd_pcm_format_width(alsa_format_) / 8;
  int silence_frames = ALSA_PERIOD_SIZE * ALSA_BUFFER_PERIODS;
  const int silence_size = silence_frames * channels_ * bytes_per_sample;
  std::vector<uint8_t> silence_buffer(silence_size, 0);
  while (silence_frames > 0) {
    int written = alsa_device_->write(
            silence_frames * channels_,
            silence_buffer.data(),
            channels_,
            alsa_format_,
            nullptr  // no shutdown flag so we can finish writing silence
    );
    if (written < 0) {
      printf("Error writing silence to ALSA device: %s\n", snd_strerror(written));
      break;
    }
    printf("Wrote %d silence frames to ALSA device of %d\n", written / channels_, silence_frames);
    silence_frames -= written / channels_;
  }

  close();
  printf("AlsaSink::run final exiting\n");
  return;
}

void AlsaSource::run(AudioStream * audio_stream)
{
  assert(audio_stream);
  printf("AlsaSource::run started\n");
  std::vector<uint8_t> audio_data;
    // Add capacity to audio_data to hold one queue chunk of audio.
  auto bytes_per_chunk = audio_stream->queue_frames_ * channels_ *
    sample_size_from_sfg_format(SFG_RW_FORMAT);

  while (!audio_stream->shutdown_flag_.load()) {
    audio_data.resize(bytes_per_chunk);
    if (!alsa_device_->get_handle()) {
      break;
    }
    if (audio_stream->queue_.write_available() == 0) {
            // Queue is full, wait briefly
            // TODO: How to handle this?
      printf("Audio queue is full, waiting...");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    auto read_result = alsa_device_->read(
            audio_stream->queue_frames_ * channels_,
            audio_data.data(),
            channels_,
            alsa_format_,
            &audio_stream->shutdown_flag_);
    if (read_result < 0) {
      printf("Error reading from ALSA device: %s\n", snd_strerror(read_result));
            // TODO: handle error
    }
    audio_data.resize(read_result * sample_size_from_sfg_format(SFG_RW_FORMAT));
    if (!audio_stream->queue_.push(audio_data)) {
            // We should not reach here since we checked write_available above
      printf("Audio queue is full, waiting...");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    audio_stream->data_available_.store(true);
    audio_stream->data_available_.notify_one();
    printf("AlsaSource: Pushed %d samples to audio queue at %s\n", read_result,
      format_timestamp().c_str());
  }
  printf("AlsaSource::run exiting.\n");

  close();
  printf("AlsaSource::run final exiting\n");
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
      // TODO: no need to limit the format here.
  //config_ranges_->format_values = {sfg_format_from_sndfile_format(sndfile_format)};
  printf("Opened file %s: channels=%d, samplerate=%d, format=0x%X\n",
    file_path_.c_str(), sf_channels, sf_samplerate, sndfile_format);

  return std::nullopt;
}

void SndFileSource::run(AudioStream * audio_stream)
{
  assert(audio_stream);
  printf("SndFileSource::run started\n");
  audio_stream->process_fileh(sndfileh_);
  printf("SndFileSource::run exiting\n");
  audio_stream->shutdown();
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
  auto bytes_per_chunk = audio_stream->queue_frames_ * channels_ *
    sample_size_from_sfg_format(SFG_RW_FORMAT);
  size_t file_size = bytes_per_chunk + MAX_HEADER;
  snd_file_write.reserve(file_size);

    // Messages will be sent at a constant rate based on samplerate and buffer size.
  auto duration = std::chrono::microseconds(static_cast<int64_t>(1'000'000.0 *
      audio_stream->queue_frames_ / (samplerate_)));
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
        channels_, samplerate_, audio_stream->queue_frames_))
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
  auto w_format = audio_stream->rw_format_;
  std::vector<uint8_t> r_buffer;
  std::vector<uint8_t> w_buffer;
  create_convert_vectors(r_format, w_format,
    audio_stream->queue_frames_ * vio_handle.fileh.channels(), r_buffer, w_buffer);
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
  int samplerate = fileh.samplerate();
    // Messages will be sent at a constant rate based on samplerate and buffer size.
  auto duration = std::chrono::microseconds(static_cast<int64_t>(1'000'000.0 *
        queue_frames_ / (samplerate)));

        // These were probably set during the open, bu reset here to be sure.
  if (source_) {
    source_->config_ranges_->samplerate_values = {fileh.samplerate()};
    source_->config_ranges_->channel_values = {fileh.channels()};
  }

  auto result = fix_parms();
  if (result.has_value()) {
    RCLCPP_ERROR(rcl_logger, "Error fixing parameters for audio stream: %s", result->c_str());
    return;
  }

  auto r_format = sfg_format_from_sndfile_format(fileh.format());
  auto w_format = rw_format_;
  std::vector<uint8_t> r_buffer;
  std::vector<uint8_t> w_buffer;
  create_convert_vectors(r_format, w_format,
    queue_frames_ * fileh.channels(), r_buffer, w_buffer);

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

void AudioStream::process_raw(std::vector<uint8_t> & audio_data, int samplerate, int channels, SfgRwFormat r_format)
{
    // Messages will be sent at a constant rate based on samplerate and buffer size.
  auto duration = std::chrono::microseconds(static_cast<int64_t>(1'000'000.0 *
        queue_frames_ / (samplerate)));

  auto w_format = rw_format_;
  std::vector<uint8_t> w_buffer;

  auto r_sample_size = sample_size_from_sfg_format(r_format);
  auto w_sample_size = sample_size_from_sfg_format(w_format);

  // Calculate chunk size in samples for each iteration
  int chunk_samples = queue_frames_ * channels;
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

// ToDo: Do not limit speed if writing to a file.
std::optional<std::string> TtsSource::initialize()
{
  // We initialize here since we may need to know the samplerate for timing.
  nlohmann::json json_payload;
  if (name_.empty()) {
    name_ = "espeak";
  }
  if (text_.empty()) {
    return std::string("TTS text cannot be empty");
  }

  if (name_ == "espeak") {
    printf("TtsSource::initialize using espeak TTS at %s\n", format_timestamp().c_str());
    tts_method_ = TtsMethod::TTS_PROGRAM_WAV;
    samplerate_ = 48000;
  } else if (name_ == "piper") {
    printf("TtsSource::initialize using Piper TTS at %s\n", format_timestamp().c_str());
    tts_method_ = TtsMethod::TTS_PROGRAM_RAW;
    if (voice_.empty()) {
      voice_ = "en_US-amy-medium";
    }
    if (voice_.size() >= 3 && voice_.substr(voice_.size() - 3) == "low") {
      samplerate_ = 16000;
    } else {
      samplerate_ = 22050;
    }
  } else if (name_ == "openai") {
    printf("TtsSource::initialize using OpenAI TTS at %s\n", format_timestamp().c_str());
    tts_method_ = TtsMethod::TTS_CURL;
    const char* api_key_cstr = std::getenv("OPENAI_API_KEY");
    if (!api_key_cstr || strlen(api_key_cstr) == 0) {
      return std::string("OPENAI_API_KEY environment variable is not set");
    }
    samplerate_ = 24000;
    url_ = "https://api.openai.com/v1/audio/speech";
      // Defaults
    if (model_.empty()) {
      model_ = "gpt-4o-mini-tts";
    }
    if (voice_.empty()) {
      voice_ = "coral";
    }
    if (format_.empty()) {
      format_ = "wav";
    }
    // Headers
    std::string auth_header = "Authorization: Bearer " + std::string(api_key_cstr);
    headers_ = curl_slist_append(headers_, auth_header.c_str());
    // Payload
    json_payload["model"] = model_;
    json_payload["input"] = text_;
    json_payload["voice"] = voice_;
    json_payload["format"] = format_;
  }
  else if (name_ == "elevenlabs") {
    printf("TtsSource::initialize using ElevenLabs TTS at %s\n", format_timestamp().c_str());
    tts_method_ = TtsMethod::TTS_CURL;
    const char* api_key_cstr = std::getenv("ELEVENLABS_API_KEY");
    if (!api_key_cstr || strlen(api_key_cstr) == 0) {
      return std::string("ELEVENLABS_API_KEY environment variable is not set");
    }
      // Defaults
    if (model_.empty()) {
      model_ = "eleven_flash_v2_5";
    }
    if (voice_.empty()) {
      voice_ = "JBFqnCBsd6RMkjVDRZzb";
    }
    if (format_.empty()) {
      format_ = "mp3_44100_128";
    }
    url_ = "https://api.elevenlabs.io/v1/text-to-speech/" + voice_ + "?output_format=" + format_;
    auto rate_split = split_string(format_, '_');
    if (rate_split.size() < 2) {
      return std::string("Invalid format string for ElevenLabs TTS: ") + format_;
    }
    samplerate_ = std::stoi(rate_split[1]);
      // Headers
    std::string auth_header = "xi-api-key: " + std::string(api_key_cstr);
    headers_ = curl_slist_append(headers_, auth_header.c_str());
      // Payload
    json_payload["model_id"] = model_;
    json_payload["text"] = text_;
    json_payload["voice_id"] = voice_;
    json_payload["output_format"] = format_;
  } else if (name_ == "piper-http") {
      printf("TtsSource::initialize using Piper HTTP TTS at %s\n", format_timestamp().c_str());
      tts_method_ = TtsMethod::TTS_CURL;
      url_ = "http://localhost:5000";
      if (voice_.size() >= 3 && voice_.substr(voice_.size() - 3) == "low") {
        samplerate_ = 16000;
      } else {
        samplerate_ = 22050;
      }
      json_payload["text"] = text_;
      json_payload["voice"] = voice_;
  } else {
    return std::string("Unsupported TTS provider: ") + name_;
  }
  json_str_ = json_payload.dump();
  printf("TtsSource::initialize JSON payload: %s\n", json_str_.c_str());
  headers_ = curl_slist_append(headers_, "Content-Type: application/json");
  return std::nullopt;
}

std::optional<std::string> TtsSource::fetch_tts_program(std::vector<uint8_t> & audio_data)
{
  std::string command;
  if (name_ == "espeak") {
    if (hasEspeakNG()) {
      command = "espeak-ng --stdout \"" + text_ + "\"";
    } else if (hasEspeak()) {
      command = "espeak --stdout \"" + text_ + "\"";
    } else {
      return std::string("Neither espeak nor espeak-ng is installed");
    }
  } else if (name_ == "piper") {
    if (!isProgramInstalled("piper")) {
      return std::string("Piper TTS program is not installed");
    }
    command = "piper --output-raw -m " + voice_ + " --data-dir " + PIPER_DATA_DIR + " -- " + "\"" + text_ + "\"";
  } else {
    return std::string("Unsupported TTS provider: ") + name_;
  }
  printf("TtsSource::fetch_tts_program executing command: %s at %s\n", command.c_str(),
    format_timestamp().c_str());
  FILE * pipe = popen(command.c_str(), "r");
  if (!pipe) {
    return std::string("Failed to execute TTS command");
  }
  printf("TtsSource::fetch_tts_program started reading audio data at %s\n", format_timestamp().c_str());
  // ToDo: the buffer size should match the audio chunk size.
  char buffer[4096];
  size_t bytes_read;
  while ((bytes_read = fread(buffer, 1, sizeof(buffer), pipe)) > 0) {
    audio_data.insert(audio_data.end(), buffer, buffer + bytes_read);
  }
  int result = pclose(pipe);
  if (result != 0) {
    return std::format("TTS command failed with code {}", result);
  }
  printf("TtsSource::fetch_tts_program completed with %zu bytes of audio data at %s\n",
    audio_data.size(), format_timestamp().c_str());
  return std::nullopt;
}

std::optional<std::string> TtsSource::fetch_tts_curl(std::vector<uint8_t> & audio_data)
{
  printf("TtsSource::fetch_tts_curl called to url %s at %s\n", url_.c_str(), format_timestamp().c_str());
  nlohmann::json json_payload;

  CURL * curl = curl_easy_init();
  if (!curl) {
    return std::string("Failed to initialize CURL");
  }

  // Set CURL options
  curl_easy_setopt(curl, CURLOPT_URL, url_.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers_);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_str_.c_str());
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);

  // Set up response buffer
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, static_cast<void *>(&audio_data));

  // Perform request
  printf("TtsSource: Sending TTS request to %s at %s\n", name_.c_str(), format_timestamp().c_str());
  CURLcode res = curl_easy_perform(curl);
  printf("TtsSource: Received TTS response from %s at %s\n", name_.c_str(), format_timestamp().c_str());

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

  std::vector<uint8_t> audio_data;
      // Reserve some data. CURL will expand as needed.
  audio_data.reserve(CURL_MAX_WRITE_SIZE);
  if (tts_method_ == TtsMethod::TTS_PROGRAM_WAV || tts_method_ == TtsMethod::TTS_PROGRAM_RAW) {
    auto fetch_result = fetch_tts_program(audio_data);

    if (fetch_result.has_value()) {
      printf("TtsSource: Error fetching TTS audio: %s\n", fetch_result.value().c_str());
      return;
    }
  } else if (tts_method_ == TtsMethod::TTS_CURL) {
      auto fetch_result = fetch_tts_curl(audio_data);

      if (fetch_result.has_value()) {
        printf("TtsSource: Error fetching TTS audio: %s\n", fetch_result.value().c_str());
        return;
      }
  } else {
    printf("TtsSource: Unsupported TTS method\n");
    return;
  }
  if (audio_data.empty()) {
    printf("TtsSource: No audio data received\n");
    return;
  }

  printf("TtsSource: Received %zu bytes of audio data\n", audio_data.size());

  if (tts_method_ == TtsMethod::TTS_PROGRAM_RAW) {
      // If the data is raw, we can process it directly without the virtual file.
      // TODO: we need to know the format of the raw data. For now we assume it's PCM_16.
    audio_stream->process_raw(audio_data, samplerate_, 1, SFG_SHORT);
  } else {
      // Convert the audio data (which is a file content) to the stream format
    VIO_SOUNDFILE_HANDLE vio_handle;
    if (auto err = ropen_vio_from_vector(audio_data, vio_handle)) {
      printf("TtsSource: Failed to open sound file from TTS audio data: %s\n", err->c_str());
      return;
    }
    audio_stream->process_fileh(vio_handle.fileh);
  }
  printf("TtsSource::run completed at %s\n", format_timestamp().c_str());
  audio_stream->shutdown();
}
