#include <algorithm>
#include <sndfile.hh>
#include <cstring>
#include <ios>
#include <fstream>
#include <cstdio>
#include <audio2_stream/AudioStream.hpp>
#include <cstdint>
#include <random>

#include "unique_identifier_msgs/msg/uuid.hpp"

static auto rcl_logger = rclcpp::get_logger("audio2_stream/AudioStream");

// Adapted from https://github.com/autowarefoundation/autoware_utils/tree/main/autoware_utils_uuid
inline unique_identifier_msgs::msg::UUID generate_uuid()
{
  // Generate random number
  unique_identifier_msgs::msg::UUID uuid;
  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(uuid.uuid.begin(), uuid.uuid.end(), bit_eng);

  return uuid;
}

std::optional<std::string> AlsaTerminal::open(snd_pcm_stream_t direction)
{
    AlsaHwParams hw_params;
    hw_params.device = alsa_device_name_.c_str();
    hw_params.channels = channels_;
    hw_params.samplerate = samplerate_;
    hw_params.format = format_;
    hw_params.direction = direction;

    AlsaSwParams sw_params;

    return alsa_open(hw_params, sw_params, alsa_dev_);
};

void AlsaTerminal::close() {
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
        audio_stream->data_available_.store(false);
        // empty the queue
        while (audio_stream->queue_.pop(audio_data)) {
            int bytes_per_sample = snd_pcm_format_width(format_) / 8;
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
        }
    }
    puts("AlsaSink::run exiting\n");
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
    create_convert_vectors(r_format, w_format, BUFFER_FRAMES * sndfileh_.channels(), r_buffer, w_buffer);
    auto duration = std::chrono::microseconds(static_cast<int64_t>(1'000'000.0 * BUFFER_FRAMES / (sndfileh_.samplerate())));
    auto next_time = std::chrono::steady_clock::now();
    bool done = false;

    while (!(audio_stream->shutdown_flag_.load()) && !done) {
        // Fill the queue before sleeping
        while (audio_stream->queue_.write_available() > 0 &&
               !audio_stream->shutdown_flag_.load()) {
            int samples_read = sfg_read2(sndfileh_, r_format, r_buffer.data(), BUFFER_FRAMES * sndfileh_.channels());
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
            printf("Pushed %d samples to audio queue\n", samples_converted);
            audio_stream->data_available_.store(true);
            audio_stream->data_available_.notify_one();
        }
        next_time += duration;
        std::this_thread::sleep_until(next_time);
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
    std::vector<char> snd_file_write;
    auto bytes_per_chunk = BUFFER_FRAMES * channels_ * sample_size_from_sfg_format(SFG_RW_FORMAT);
    size_t file_size = bytes_per_chunk + MAX_HEADER;
    snd_file_write.reserve(file_size);

    // Messages will be sent at a constant rate based on samplerate and buffer size.
    auto duration = std::chrono::microseconds(static_cast<int64_t>(1'000'000.0 * BUFFER_FRAMES / (samplerate_)));
    auto next_time = std::chrono::steady_clock::now();

    // Wait to pop from queue. After the first, we rely on time only.
    audio_stream->data_available_.wait(false); // Wait until there's something to process
    audio_stream->data_available_.store(false);

    // Create and populate the AudioChunk message
    auto message = audio2_stream_msgs::msg::AudioChunk();

    // Fill in the message header
    // ToDo: consider making message lifetime match the AudioStream lifetime.
    message.header.handle = 0; // Placeholder handle
    message.header.description = description_;
    message.header.volume = 1.0f; // Placeholder volume
    message.header.handle = 0; // Placeholder handle
    message.header.uuid = uuid;

    bool done = false;
    while (!done) {
        // Send a single chunk as a message
        if (audio_stream->queue_.pop(audio_data)) {

            // Fill in the message header
            message.header.sequence = ++sequence_number;
            message.header.chunk_start_time = rclcpp::Clock().now();
            message.data.clear();

            // Create a virtual sound file in memory for topic publish
            auto data_size = audio_data.size();

            if (data_size > snd_file_write.capacity() - MAX_HEADER) {
                RCLCPP_WARN(rcl_logger, "Audio data size %zu exceeds buffer capacity %zu, resizing", data_size, snd_file_write.capacity() - MAX_HEADER);
                snd_file_write.reserve(data_size + MAX_HEADER);
            }
            snd_file_write.clear();

            VIO_SOUNDFILE_HANDLE m_vio_sndfileh;
            // TODO: revise VIO to have consistent types
            m_vio_sndfileh.vio_data.data = snd_file_write.data();
            m_vio_sndfileh.vio_data.length = 0;
            m_vio_sndfileh.vio_data.offset = 0;
            m_vio_sndfileh.vio_data.capacity = snd_file_write.capacity();
            if (auto err = open_sndfile_from_buffer2(m_vio_sndfileh, SFM_WRITE, sfFormat_, channels_, samplerate_)) {
                printf("Failed to open sound file for writing to buffer: %s\n", err->c_str());
                return;
            }

            auto data_samples = static_cast<int>(data_size) / sample_size_from_sfg_format(SFG_RW_FORMAT);
            int samples_written = sfg_write_convert(m_vio_sndfileh.fileh, SFG_RW_FORMAT, SFG_RW_FORMAT,
                reinterpret_cast<char*>(audio_data.data()), data_samples);
            if (samples_written != data_samples) {
                RCLCPP_ERROR(rcl_logger, "Error writing audio data to virtual sound file: expected %d samples, wrote %d samples", data_samples, samples_written);
                return;
            }

            // Copy the serialized data into message.data
            message.data.reserve(samples_written * sample_size_from_sfg_format(SFG_RW_FORMAT) + MAX_HEADER);
            std::copy(snd_file_write.data(), snd_file_write.data() + m_vio_sndfileh.vio_data.length,
                      std::back_inserter(message.data));

            // Publish the message
            publisher_->publish(message);

            // print info about the data
            auto time_since_epoch = next_time.time_since_epoch();
            auto hours = std::chrono::duration_cast<std::chrono::hours>(time_since_epoch) % 24;
            auto minutes = std::chrono::duration_cast<std::chrono::minutes>(time_since_epoch) % 60;
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch) % 60;
            auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch) % 1000;
            printf("MessageSink sending audio chunk of %zu bytes on topic %s at %02ld:%02ld:%02ld:%03ld\n", 
                   audio_data.size(), topic_.c_str(), hours.count(), minutes.count(), seconds.count(), milliseconds.count());
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
    message.header.sequence = message.header.SEQUENCE_EOS;
    message.header.chunk_start_time = rclcpp::Clock().now();
    message.data.clear();
    publisher_->publish(message);

    printf("MessageSink::run exiting\n");
}

void AudioStream::start()
{
    sink_thread_ = std::make_unique<std::jthread>(&AudioTerminal::run, sink_.get(), this);
    source_thread_ = std::make_unique<std::jthread>(&AudioTerminal::run, source_.get(), this);
}

void AudioStream::shutdown()
{
    printf("AudioStream::shutdown called\n");
    shutdown_flag_.store(true);
    data_available_.store(true);
    data_available_.notify_all();
    printf("AudioStream::shutdown notified\n");
}
