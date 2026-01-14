#include <algorithm>
#include <sndfile.hh>
#include <cstring>
#include <ios>
#include <fstream>
#include <cstdio>
#include <audio2_stream/AudioStream.hpp>

#include "rclcpp/rclcpp.hpp"

static auto rcl_logger = rclcpp::get_logger("audio2_stream/AudioStream");

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
    const int BUFFER_FRAMES = 480;
    auto r_format = sfg_format_from_sndfile_format(sndfileh_.format());
    auto w_format = audio_stream->rw_format_;
    auto r_sample_size = sample_size_from_sfg_format(r_format);
    auto w_sample_size = sample_size_from_sfg_format(w_format);
    auto r_buffer_size = BUFFER_FRAMES * sndfileh_.channels() * r_sample_size;
    auto w_buffer_size = BUFFER_FRAMES * sndfileh_.channels() * w_sample_size;
    std::vector<uint8_t> r_buffer(r_buffer_size);
    std::vector<uint8_t> w_buffer(w_buffer_size);
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

void AudioStream::shutdown()
{
    printf("AudioStream::shutdown called\n");
    shutdown_flag_.store(true);
    data_available_.store(true);
    data_available_.notify_all();
    printf("AudioStream::shutdown notified\n");
}

void AudioStream::start()
{
    source_thread_ = std::make_unique<std::jthread>(&AudioTerminal::run, source_.get(), this);
    sink_thread_ = std::make_unique<std::jthread>(&AudioTerminal::run, sink_.get(), this);
}
