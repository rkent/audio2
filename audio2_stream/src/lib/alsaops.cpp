#include <cstring>

#include "audio2_stream/alsaops.hpp"
#include "rclcpp/rclcpp.hpp"

#include <sndfile.h>
#include <atomic>
#include <format>
#include <string>
#include <optional>
#include <cstring>
#include <cstdio>
#include <cstdarg>

template<typename T>
class AlsaWrite {
public:
    int operator()(snd_pcm_t* alsa_dev, T* data, int frames, int channels, std::atomic<bool>* shutdown_flag=nullptr)
    {	static	int epipe_count = 0;

        int total = 0;
        int retval;

        if (epipe_count > 0)
            epipe_count --;

        while (total < frames)
        {
            // Check if shutdown has been requested
            if (shutdown_flag && shutdown_flag->load()) {
                break;
            }

            retval = snd_pcm_writei (alsa_dev, data + total * channels, frames - total);

            if (retval > 0)
            {	total += retval;
                if (total == frames)
                    return total;

                continue;
                };

            switch (retval)
            {
                case 0:
                if (snd_pcm_state(alsa_dev) == SND_PCM_STATE_PREPARED)
                {
                    // I don't understand why we get here, but for some reason the card refuses to start
                    // unless we explicitly start it.
                    snd_pcm_start (alsa_dev);
                } else {
                    printf("alsa_write: wrote zero frames.n");
                }
                continue;
                case -EAGAIN :
                        puts ("alsa_write: EAGAIN");
                        continue;
                        break;

                case -EPIPE :
                        if (epipe_count > 0)
                        {	printf ("alsa_write: EPIPE %d\n", epipe_count);
                            if (epipe_count > 140)
                                return retval;
                            };
                        epipe_count += 100;

                        snd_pcm_prepare (alsa_dev);
                        break;

                case -EBADFD :
                        fprintf (stderr, "alsa_write: Bad PCM state.n");
                        return 0;
                        break;

    #if defined ESTRPIPE && ESTRPIPE != EPIPE
                case -ESTRPIPE :
                        fprintf (stderr, "alsa_write: Suspend event.n");
                        return 0;
                        break;
    #endif

                case -EIO :
                        puts ("alsa_write: EIO");
                        return 0;

                default :
                        fprintf (stderr, "alsa_write: retval = %d\n", retval);
                        return 0;
                        break;
                }; /* switch */
            }; /* while */

        return total;
    }
}; /* AlsaWrite */

AlsaWrite<float> alsa_write_float;
AlsaWrite<short> alsa_write_short;
AlsaWrite<int> alsa_write_int;
AlsaWrite<double> alsa_write_double;
int alsa_write(int samples, snd_pcm_t* alsa_dev, void* data, int channels, snd_pcm_format_t alsa_format, std::atomic<bool>* shutdown_flag)
{
    std::size_t hash_id = std::hash<std::thread::id>{}(std::this_thread::get_id()) % 10000;
    printf("alsa_write: thread: %zu, samples=%d, channels=%d, format=%x at %s\n", hash_id, samples, channels, alsa_format, format_timestamp().c_str());
    int frames = samples / channels;
    if (alsa_format == SND_PCM_FORMAT_S16) {
        return channels * alsa_write_short(alsa_dev, reinterpret_cast<short*>(data), frames, channels, shutdown_flag);
    } else if (alsa_format == SND_PCM_FORMAT_S32) {
        return channels * alsa_write_int(alsa_dev, reinterpret_cast<int*>(data), frames, channels, shutdown_flag);
    } else if (alsa_format == SND_PCM_FORMAT_FLOAT) {
        return channels *alsa_write_float(alsa_dev, reinterpret_cast<float*>(data), frames, channels, shutdown_flag);
    } else if (alsa_format == SND_PCM_FORMAT_FLOAT64) {
        return channels * alsa_write_double(alsa_dev, reinterpret_cast<double*>(data), frames, channels, shutdown_flag);
    } else {
        return -1;
    }   
}

// Macro definitions for alsa_open
#define _S(cstr) std::string(cstr)
// We allow either a std::string or a const char* to be passed in as estr
#define ECALL(func, estr, ...) \
if ((err = func(__VA_ARGS__)) < 0) {\
    error_str = std::format("alsa_open error: {} ({})", estr, snd_strerror (err)); \
    continue; \
};

std::optional<std::string>
alsa_open (AlsaHwParams & hw_vals, AlsaSwParams & sw_vals, snd_pcm_t *& alsa_dev)
{
    auto requested_hw_vals = hw_vals; // Copy for reporting
    const char * device = hw_vals.device;
    alsa_dev = nullptr;
    snd_pcm_hw_params_t *hw_params;
    snd_pcm_sw_params_t *sw_params;

    int err;

    // Sometimes open fails with a requested format, but succeeds with a different one.
    std::string error_str;
    for (snd_pcm_format_t format : {hw_vals.format, SND_PCM_FORMAT_S32, SND_PCM_FORMAT_S16, SND_PCM_FORMAT_FLOAT}) {
        if (error_str.length() > 0) {
            printf("alsa_open failed: (%s)\nretrying with format %d\n", error_str.c_str(), format);
            error_str.clear();
            hw_vals = requested_hw_vals; // Reset to requested values
        }
        printf("alsa_open: Trying to open device %s with format %d at %s\n", device, format, format_timestamp().c_str());
        hw_vals.format = format;
        // TODO: pre-open alsa device to reduce latency.
        ECALL(snd_pcm_open, _S("cannot open audio device ") + _S(device), &alsa_dev, device, hw_vals.direction, 0);
        printf("alsa_open: snd_pcm_open succeeded at %s\n", format_timestamp().c_str());
        snd_pcm_hw_params_alloca (&hw_params);

        ECALL(snd_pcm_hw_params_any, "cannot initialize hardware parameter structure", alsa_dev, hw_params);
        ECALL(snd_pcm_hw_params_set_access, "cannot set access type", alsa_dev, hw_params, hw_vals.access);
        ECALL(snd_pcm_hw_params_set_format, "cannot set sample format", alsa_dev, hw_params, format);
        ECALL(snd_pcm_hw_params_set_rate_near, "cannot set sample rate", alsa_dev, hw_params, &hw_vals.samplerate, 0);
        ECALL(snd_pcm_hw_params_set_channels, "cannot set channel count", alsa_dev, hw_params, hw_vals.channels);
        ECALL(snd_pcm_hw_params_set_period_size_near, "cannot set period size", alsa_dev, hw_params, &hw_vals.period_size, 0);
        if (hw_vals.buffer_size < hw_vals.period_size * 2) {
            hw_vals.buffer_size = hw_vals.period_size * 2;
        }
        ECALL(snd_pcm_hw_params_set_buffer_size_near, "cannot set buffer size", alsa_dev, hw_params, &hw_vals.buffer_size);
        ECALL(snd_pcm_hw_params, "cannot install hw params", alsa_dev, hw_params);

        // TODO: set sw_vals based on actual buffer size and period size.
        snd_pcm_sw_params_alloca(&sw_params);
        ECALL(snd_pcm_sw_params_current, "snd_pcm_sw_params_current", alsa_dev, sw_params);
        ECALL(snd_pcm_sw_params_set_start_threshold, "cannot set start threshold", alsa_dev, sw_params, sw_vals.start_threshold);
        ECALL(snd_pcm_sw_params_set_stop_threshold, "cannot set stop threshold", alsa_dev, sw_params, sw_vals.stop_threshold);
        ECALL(snd_pcm_sw_params_set_silence_size, "cannot set silence size", alsa_dev, sw_params, sw_vals.silence_size);
        ECALL(snd_pcm_sw_params_set_silence_threshold, "cannot set silence threshold", alsa_dev, sw_params, sw_vals.silence_threshold);
        ECALL(snd_pcm_sw_params, "cannot install sw params", alsa_dev, sw_params);
        snd_pcm_reset (alsa_dev);
        if (error_str.empty()) {
            break;
        }
        printf("alsa_open: alsa_open succeeded at %s\n", format_timestamp().c_str());
    }
    if (error_str.empty()) {
        if (hw_vals.format != requested_hw_vals.format) {
            printf("\nWarning: Requested ALSA format %d not available, using %d instead.\n", requested_hw_vals.format, hw_vals.format);
        }
        if (hw_vals.samplerate != requested_hw_vals.samplerate) {
            printf("\nWarning: Requested ALSA samplerate %u not available, using %u instead.\n", requested_hw_vals.samplerate, hw_vals.samplerate);
        }
        if (hw_vals.period_size != requested_hw_vals.period_size) {
            printf("\nWarning: Requested ALSA period size %lu not available, using %lu instead.\n", requested_hw_vals.period_size, hw_vals.period_size);
        }
        if (hw_vals.buffer_size != requested_hw_vals.buffer_size) {
            printf("\nWarning: Requested ALSA buffer size %lu not available, using %lu instead.\n", requested_hw_vals.buffer_size, hw_vals.buffer_size);
        }
        return std::nullopt;
    }
    if (alsa_dev)
    {
        snd_pcm_close (alsa_dev);
    }
    return error_str;
} /* alsa_open */

AlsaWriteThread::AlsaWriteThread(
    AlsaHwParams hw_vals,
    AlsaSwParams sw_vals,
    boost::lockfree::spsc_queue<std::vector<uint8_t>>* queue,
    std::atomic<bool>* shutdown_flag,
    std::atomic<bool>* data_available
    )
    : queue_(queue), shutdown_flag_(shutdown_flag), data_available_(data_available)
{
    snd_pcm_t * alsa_dev = nullptr;
    auto open_result = alsa_open(hw_vals, sw_vals, alsa_dev);
    if (open_result.has_value()) {
        error_str_ = open_result.value();
        return;
    }
    alsa_dev_ = alsa_dev;
    hw_vals_ = hw_vals;
}

snd_pcm_t * AlsaWriteThread::get_alsa_dev() {
    return alsa_dev_;
}

std::string AlsaWriteThread::get_error() {
    return error_str_;
}

void AlsaWriteThread::close() {
    if (alsa_dev_) {
        snd_pcm_drain(alsa_dev_);
        snd_pcm_close(alsa_dev_);
        alsa_dev_ = nullptr;
    }
    return;
}

void AlsaWriteThread::run()
{
    while (!shutdown_flag_->load()) {
        if (error_str_.length() > 0) {
            break;
        }
        if (!alsa_dev_) {
            error_str_ = "ALSA device is not opened";
            break;
        }
        std::vector<uint8_t> audio_data;
        // Try to pop from queue
        if (data_available_) {
            data_available_->wait(false); // Wait until there's something to process
            data_available_->store(false);
        }
        if (!queue_->pop(audio_data)) {
            // Check if shutdown has been requested
            if (shutdown_flag_ && shutdown_flag_->load()) {
                error_str_ = "Playback interrupted by shutdown signal";
                break;
            }
            // Queue is empty, sleep briefly and continue. We should not reach this
            printf("Audio queue is empty, waiting...");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        int bytes_per_sample = snd_pcm_format_width(hw_vals_.format) / 8;
        int write_result = alsa_write(
            static_cast<int>(audio_data.size() / bytes_per_sample),
            alsa_dev_,
            audio_data.data(),
            hw_vals_.channels, // channels
            hw_vals_.format,
            shutdown_flag_);
        if (write_result < 0) {
            printf("Error writing to ALSA device: %s\n", snd_strerror(write_result));
        }
    }
    close();
    return;
}

template<typename T>
class AlsaRead {
public:
    int operator()(snd_pcm_t* alsa_dev, T* data, int frames, int channels, std::atomic<bool>* shutdown_flag=nullptr)
    {	static	int epipe_count = 0;

        int total = 0;
        int retval;

        if (epipe_count > 0)
            epipe_count --;

        while (total < frames)
        {
            // Check if shutdown has been requested
            if (shutdown_flag && shutdown_flag->load()) {
                break;
            }

            retval = snd_pcm_readi(alsa_dev, data + total * channels, frames - total);

            if (retval > 0) {
                total += retval;
                if (total == frames) {
                    return total;
                }
                continue;
            }

            switch (retval)
            {
                case 0:
                    continue;
                case -EAGAIN :
                    puts ("alsa_read: EAGAIN");
                    continue;
                    break;

                case -EPIPE :
                    if (epipe_count > 0) {
                        printf ("alsa_read: EPIPE %d\n", epipe_count);
                        if (epipe_count > 140) {
                            return retval;
                        }
                    }
                    epipe_count += 100;
                    snd_pcm_prepare (alsa_dev);
                    break;

                case -EBADFD :
                    fprintf (stderr, "alsa_read: Bad PCM state.n");
                    return 0;
                    break;

                default :
                    fprintf (stderr, "alsa_read: retval = %d\n", retval);
                    return 0;
                    break;
                }; /* switch */
            }; /* while */

        return total;
    }
}; /* AlsaRead */

AlsaRead<float> alsa_read_float;
AlsaRead<short> alsa_read_short;
AlsaRead<int> alsa_read_int;
AlsaRead<double> alsa_read_double;
int alsa_read(int samples, snd_pcm_t* alsa_dev, void* data, int channels, snd_pcm_format_t alsa_format, std::atomic<bool>* shutdown_flag)
{
    std::size_t hash_id = std::hash<std::thread::id>{}(std::this_thread::get_id()) % 10000;
    printf("alsa_read: thread: %zu, samples=%d, channels=%d, format=%x at %s\n", hash_id, samples, channels, alsa_format, format_timestamp().c_str());
    int frames = samples / channels;
    if (alsa_format == SND_PCM_FORMAT_S16) {
        return channels * alsa_read_short(alsa_dev, reinterpret_cast<short*>(data), frames, channels, shutdown_flag);
    } else if (alsa_format == SND_PCM_FORMAT_S32) {
        return channels * alsa_read_int(alsa_dev, reinterpret_cast<int*>(data), frames, channels, shutdown_flag);
    } else if (alsa_format == SND_PCM_FORMAT_FLOAT) {
        return channels *alsa_read_float(alsa_dev, reinterpret_cast<float*>(data), frames, channels, shutdown_flag);
    } else if (alsa_format == SND_PCM_FORMAT_FLOAT64) {
        return channels * alsa_read_double(alsa_dev, reinterpret_cast<double*>(data), frames, channels, shutdown_flag);
    } else {
        return -1;
    }
}
