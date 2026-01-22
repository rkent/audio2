#include <algorithm>
#include <sndfile.hh>
#include <cstring>
#include <ios>
#include <fstream>
#include <cstdio>
#include <audio2_stream/AudioStream.hpp>

static auto rcl_logger = rclcpp::get_logger("audio2_stream/AudioStream");

std::optional<std::string> AlsaTerminal::open(snd_pcm_stream_t direction)
{
    AlsaHwParams hw_params;
    hw_params.device = alsa_device_name_.c_str();
    hw_params.channels = channels_;
    hw_params.samplerate = samplerate_;
    hw_params.format = format_;
    hw_params.direction = direction;

    printf("AlsaTerminal::open called at %s\n", format_timestamp().c_str());
    printf("AlsaTerminal::open called with hw_params: device=%s, channels=%u, samplerate=%u, format=%d, direction=%d\n",
           hw_params.device,
           hw_params.channels,
           hw_params.samplerate,
           hw_params.format,
           hw_params.direction);
    
    AlsaSwParams sw_params;

    auto result = alsa_open(hw_params, sw_params, alsa_dev_);
    printf("AlsaTerminal::open completed at %s\n", format_timestamp().c_str());
    if (result.has_value()) {
        error_str_ = result.value();
        return error_str_;
    }
    // alsa_open may change the format if original is not supported.
    format_ = hw_params.format;
    if (samplerate_ != static_cast<int>(hw_params.samplerate)) {
        char buffer[256];
        snprintf(buffer, sizeof(buffer), "Error: Requested samplerate %u, got %u from ALSA device.\n", samplerate_, hw_params.samplerate);
        return std::string(buffer);
    }
    return std::nullopt;
};

void AlsaTerminal::close() {
    printf("AlsaTerminal::close called\n");
    if (alsa_dev_) {
        snd_pcm_drain(alsa_dev_);
        snd_pcm_close(alsa_dev_);
        alsa_dev_ = nullptr;
    }
    return;
}

void AlsaSink::run(AudioStream * audio_stream)
{
    assert(audio_stream);
    assert(alsa_dev_);
    printf("AlsaSink::run started\n");
    while (!audio_stream->shutdown_flag_.load()) {
        if (error_str_.length() > 0) {
            break;
        }
        if (!alsa_dev_) {
            error_str_ = "ALSA device is not opened";
            break;
        }
        std::vector<uint8_t> audio_data;

        // Wait to pop from queue
        audio_stream->data_available_.wait(false); // Wait until there's something to process
        std::size_t hash_id = std::hash<std::thread::id>{}(std::this_thread::get_id()) % 10000;
        printf("\nAlsaSink::run thread %zu woke up to process audio data at %s\n", hash_id, format_timestamp().c_str());
        audio_stream->data_available_.store(false);
        // empty the queue
       do {
            printf("AlsaSink: popping audio data from queue ra: %zu wa: %zu \n", audio_stream->queue_.read_available(), audio_stream->queue_.write_available());
            auto pop_result = audio_stream->queue_.pop(audio_data);
            if (!pop_result) {
                break;
            }
            int bytes_per_sample = snd_pcm_format_width(format_) / 8;
            if (true) {
                // Output ALSA status for debugging
                snd_pcm_status_t * stat;
                snd_pcm_status_alloca(&stat);
                snd_pcm_status(alsa_dev_, stat);
                snd_output_t *output;
                snd_output_stdio_attach(&output, stdout, 0);
                snd_pcm_status_dump(stat, output);
            }
            int write_result = alsa_write(
                static_cast<int>(audio_data.size() / bytes_per_sample),
                alsa_dev_,
                audio_data.data(),
                channels_, // channels
                format_,
                nullptr  // shutdown flag
            );
            if (write_result < 0) {
                printf("Error writing to ALSA device: %s\n", snd_strerror(write_result));
            }
        }  while (true);
    }
    printf("AlsaSink::run exiting, pushing silence.\n");
    // Push silence to fill the buffer before closing
    const int bytes_per_sample = snd_pcm_format_width(format_) / 8;
    int silence_frames = ALSA_PERIOD_SIZE * ALSA_BUFFER_PERIODS;
    const int silence_size = silence_frames * channels_ * bytes_per_sample;
    std::vector<uint8_t> silence_buffer(silence_size, 0);
    while (silence_frames > 0) {
        int written = alsa_write(
            silence_frames,
            alsa_dev_,
            silence_buffer.data(),
            channels_,
            format_,
            nullptr  // no shutdown flag so we can finish writing silence
        );
        if (written < 0) {
            printf("Error writing silence to ALSA device: %s\n", snd_strerror(written));
            break;
        }
        printf("Wrote %d silence frames to ALSA device of %d\n", written, silence_frames);
        silence_frames -= written;
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
    auto bytes_per_chunk = audio_stream->queue_frames_ * channels_ * sample_size_from_sfg_format(SFG_RW_FORMAT);

    while (!audio_stream->shutdown_flag_.load()) {
        audio_data.resize(bytes_per_chunk);
        if (!alsa_dev_) {
            error_str_ = "ALSA device is not opened";
            break;
        }
        if (audio_stream->queue_.write_available() == 0) {
            // Queue is full, wait briefly
            // TODO: How to handle this?
            printf("Audio queue is full, waiting...");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        auto read_result = alsa_read(
            audio_stream->queue_frames_ * channels_,
            alsa_dev_,
            audio_data.data(),
            channels_,
            format_,
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
        printf("AlsaSource: Pushed %d samples to audio queue at %s\n", read_result, format_timestamp().c_str());
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
    return std::nullopt;
}

void SndFileSource::run(AudioStream * audio_stream)
{
    auto r_format = sfg_format_from_sndfile_format(sndfileh_.format());
    auto w_format = audio_stream->rw_format_;
    std::vector<uint8_t> r_buffer;
    std::vector<uint8_t> w_buffer;
    create_convert_vectors(r_format, w_format, audio_stream->queue_frames_ * sndfileh_.channels(), r_buffer, w_buffer);
    auto duration = std::chrono::microseconds(static_cast<int64_t>(1'000'000.0 * audio_stream->queue_frames_ / (sndfileh_.samplerate())));
    auto next_time = std::chrono::steady_clock::now();
    bool done = false;

    while (!(audio_stream->shutdown_flag_.load()) && !done) {
        // Add one to the queue each duration
        while (audio_stream->queue_.write_available() > 0 &&
               !audio_stream->shutdown_flag_.load()) {
            int samples_read = sfg_read(sndfileh_, r_format, r_buffer.data(), audio_stream->queue_frames_ * sndfileh_.channels());
            if (samples_read <= 0) {
                done = true;
                break; // End of file or error
            }
            int samples_converted = convert_types(r_format, w_format, r_buffer.data(), w_buffer.data(), samples_read);
            if (samples_converted < 0) {
                RCLCPP_ERROR(rcl_logger, "Error converting audio data for streaming: %d", samples_converted);
                done = true;
                break;
            } else if (samples_converted != samples_read) {
                RCLCPP_ERROR(rcl_logger, "Mismatch in converted samples count: expected %d, got %d", samples_read, samples_converted);
                done = true;
                break;
            }

            while (!audio_stream->queue_.push(w_buffer) && !audio_stream->shutdown_flag_.load()) {
                // We should not reach here since we checked write_available above
                RCLCPP_WARN(rcl_logger, "Audio queue is full, waiting...");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            printf("SndFileSource: Pushed %d samples to audio queue at %s\n", samples_converted, format_timestamp().c_str());
            audio_stream->data_available_.store(true);
            audio_stream->data_available_.notify_one();
            next_time += duration;
            std::this_thread::sleep_until(next_time);
        }
    }
    printf("SndFileSource::run exiting\n");
    audio_stream->shutdown();
}

MessageSink::MessageSink(
    std::string topic,
    int channels,
    int samplerate,
    int sfFormat,
    rclcpp::Publisher<audio2_stream_msgs::msg::AudioChunk>::SharedPtr publisher,
    std::string description
) :
    AudioTerminal(),
    topic_(topic),
    channels_(channels),
    samplerate_(samplerate),
    sfFormat_(sfFormat),
    publisher_(publisher),
    description_(description)
{}

void MessageSink::run(AudioStream * audio_stream)
{
    assert(audio_stream);
    printf("MessageSink::run started\n");
    uint32_t sequence_number = 0;
    auto uuid = generate_uuid();

    std::vector<uint8_t> audio_data;  // to hold popped audio data

    // snd_file_write is used to hold the serialized audio chunk including header
    // TODO: determine appropriate buffer size
    std::vector<unsigned char> snd_file_write;
    auto bytes_per_chunk = audio_stream->queue_frames_ * channels_ * sample_size_from_sfg_format(SFG_RW_FORMAT);
    size_t file_size = bytes_per_chunk + MAX_HEADER;
    snd_file_write.reserve(file_size);

    // Messages will be sent at a constant rate based on samplerate and buffer size.
    auto duration = std::chrono::microseconds(static_cast<int64_t>(1'000'000.0 * audio_stream->queue_frames_ / (samplerate_)));
    auto next_time = std::chrono::steady_clock::now();

    // Wait to pop from queue. After the first, we rely on time only.
    audio_stream->data_available_.wait(false); // Wait until there's something to process
    audio_stream->data_available_.store(false);

    bool done = false;
    while (!done) {
        // Send a single chunk as a message
        if (audio_stream->queue_.pop(audio_data)) {

            // Create and populate the AudioChunk message
            auto message = std::make_unique<audio2_stream_msgs::msg::AudioChunk>();
            message->header.handle = 0; // Placeholder handle
            message->header.description = description_;
            message->header.volume = 1.0f; // Placeholder volume
            message->header.handle = 0; // Placeholder handle
            message->header.sequence = ++sequence_number;
            message->header.chunk_start_time = rclcpp::Clock().now();
            message->header.uuid = uuid;

            // Create a virtual sound file in memory for topic publish
            auto data_size = audio_data.size();

            VIO_SOUNDFILE_HANDLE m_vio_sndfileh;
            if (auto err = wopen_vio_to_vector(snd_file_write, m_vio_sndfileh, SFG_RW_FORMAT, sfFormat_,  channels_, samplerate_, audio_stream->queue_frames_)) {
                printf("Failed to open sound file for writing to buffer: %s\n", err->c_str());
                return;
            }

            //if (data_size > snd_file_write.capacity() - MAX_HEADER) {
            //    RCLCPP_WARN(rcl_logger, "Audio data size %zu exceeds buffer capacity %zu, resizing", data_size, snd_file_write.capacity() - MAX_HEADER);
            //    snd_file_write.reserve(data_size + MAX_HEADER);
            //}

            auto data_samples = static_cast<int>(data_size) / sample_size_from_sfg_format(SFG_RW_FORMAT);
            int samples_written = sfg_write_convert(m_vio_sndfileh.fileh, SFG_RW_FORMAT, SFG_RW_FORMAT,
                reinterpret_cast<char*>(audio_data.data()), data_samples);
            if (samples_written != data_samples) {
                RCLCPP_ERROR(rcl_logger, "Error writing audio data to virtual sound file: expected %d samples, wrote %d samples", data_samples, samples_written);
                return;
            }

            // Copy the serialized data into message.data
            message->data.reserve(samples_written * sample_size_from_sfg_format(SFG_RW_FORMAT) + MAX_HEADER);
            std::copy(snd_file_write.data(), snd_file_write.data() + m_vio_sndfileh.vio_data.length,
                      std::back_inserter(message->data));

            // Publish the message
            publisher_->publish(std::move(message));

            // print info about the data
            auto time_since_epoch = next_time.time_since_epoch();
            auto hours = std::chrono::duration_cast<std::chrono::hours>(time_since_epoch) % 24;
            auto minutes = std::chrono::duration_cast<std::chrono::minutes>(time_since_epoch) % 60;
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch) % 60;
            auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch) % 1000;
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
    message->header.handle = 0; // Placeholder handle
    message->header.description = description_;
    message->header.volume = 1.0f; // Placeholder volume
    message->header.handle = 0; // Placeholder handle
    message->header.sequence = message->header.SEQUENCE_EOS;
    message->header.chunk_start_time = rclcpp::Clock().now();
    message->header.uuid = uuid;

    publisher_->publish(std::move(message));

    printf("MessageSink::run exiting\n");
}

MessageSource::MessageSource(
    std::string topic
) :
    AudioTerminal(),
    topic_(topic)
{}

void MessageSource::run([[maybe_unused]]AudioStream * audio_stream)
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
    printf("MessageSource: thread %zu received audio chunk with %zu bytes sequence %d at %s\n", hash_id, message->data.size(), message->header.sequence, format_timestamp().c_str());
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
    create_convert_vectors(r_format, w_format, audio_stream->queue_frames_ * vio_handle.fileh.channels(), r_buffer, w_buffer);
    bool done = false;
    printf("MessageSource read: length %zu bytes from audio chunk\n", vio_handle.vio_data.length);
    while (!(audio_stream->shutdown_flag_.load()) && !done) {
        int samples_read = sfg_read(vio_handle.fileh, r_format, r_buffer.data(), audio_stream->queue_frames_ * vio_handle.fileh.channels());
        if (samples_read <= 0) {
            done = true;
            break; // End of file or error
        }
        printf("MessageSource: read %d samples from audio chunk at %s\n", samples_read, format_timestamp().c_str());
        int samples_converted = convert_types(r_format, w_format, r_buffer.data(), w_buffer.data(), samples_read);
        if (samples_converted < 0) {
            RCLCPP_ERROR(rcl_logger, "Error converting audio data for streaming: %d", samples_converted);
            done = true;
            break;
        } else if (samples_converted != samples_read) {
            RCLCPP_ERROR(rcl_logger, "Mismatch in converted samples count: expected %d, got %d", samples_read, samples_converted);
            done = true;
            break;
        }

        // Resize buffer to actual converted sample count to avoid pushing garbage data
        w_buffer.resize(samples_converted * sample_size_from_sfg_format(w_format));

        // Try to push to the queue without blocking.
        // If the queue is full, drop this chunk to avoid blocking the ROS2 executor.
        // Blocking here would prevent other callbacks from running and cause message delivery delays.
        if (!audio_stream->queue_.push(w_buffer)) {
            RCLCPP_WARN(rcl_logger, "Audio queue is full, dropping audio chunk to avoid blocking executor!");
            // TODO: move this to another thread to avoid blocking the callback?
            // Still notify in case the consumer is waiting
            audio_stream->data_available_.store(true);
            audio_stream->data_available_.notify_one();
            continue;
        }
        printf("MessageSource: Pushed %d samples to audio queue ra: %lu wa: %lu at %s\n", samples_converted, audio_stream->queue_.read_available(), audio_stream->queue_.write_available(), format_timestamp().c_str());
        audio_stream->data_available_.store(true);
        audio_stream->data_available_.notify_one();
    }
    printf("MessageSource: callback exiting at %s\n", format_timestamp().c_str());
}

void AudioStream::start()
{
    printf("AudioStream::start thread %zu called for <%s> at %s\n", std::hash<std::thread::id>{}(std::this_thread::get_id()) % 10000, description_.c_str(), format_timestamp().c_str());
    if (sink_) sink_thread_ = std::make_unique<std::jthread>(&AudioTerminal::run, sink_.get(), this);
    if (source_) source_thread_ = std::make_unique<std::jthread>(&AudioTerminal::run, source_.get(), this);
}

void AudioStream::shutdown()
{
    printf("AudioStream::shutdown called for %s\n", description_.c_str());
    shutdown_flag_.store(true);
    data_available_.store(true);
    data_available_.notify_all();
    if (source_thread_ && source_thread_->joinable()) {
        source_thread_->join();
    }
    if (sink_thread_ && sink_thread_->joinable()) {
        sink_thread_->join();
    }
    shutdown_complete_.store(true);
    printf("AudioStream: shutdown sent for %s\n", description_.c_str());
}
