#include <csignal>
#include <cstdio>
#include <cstring>
#include <sndfile.h>
#include <alsa/asoundlib.h>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <fstream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "audio2_stream/alsaops.hpp"
#include "audio2_stream/buffer_file.hpp"
#include "boost/lockfree/spsc_queue.hpp"

// Global flag to signal thread shutdown
std::atomic<bool> shutdown_flag(false);
// Global flag to signify that new data is available in the queue
std::atomic<bool> data_available(false);

static auto rcl_logger = rclcpp::get_logger("audio2_stream");

boost::lockfree::spsc_queue<PlayBufferParams> audio_queue(10); // Queue size of 10

#define ALSA_FORMAT SND_PCM_FORMAT_S16

void signal_handler(int signal) {
    if (signal == SIGINT) {
        RCLCPP_INFO(rcl_logger, "Keyboard interrupt received. Shutting down.");
        shutdown_flag.store(true);
        data_available.store(true); // Wake up any waiting threads
        data_available.notify_all();
        rclcpp::shutdown();
    }
}

int main(int argc, [[maybe_unused]] char ** argv)
{
    std::signal(SIGINT, signal_handler); // Register the signal handler
    rclcpp::init(argc, argv);

    // boost::lockfree::spsc_queue<PlayBufferParams> audio_queue(10); // Queue size of 10
    boost::lockfree::spsc_queue<std::vector<uint8_t>> audio_queue(10);
    // Start play_buffer thread
    //std::thread playback_thread(play_buffer_thread, &audio_queue, &shutdown_flag, &data_available);

    // Enqueue all files from command line arguments
    for (int k = 1; k < argc; k++) {
        auto file_path = std::string(argv[k]);
        std::unique_ptr<SndFileSource> snd_file_source = std::make_unique<SndFileSource>(file_path);
        auto open_result = snd_file_source->open();
        int channels = snd_file_source->sndfileh_.channels();
        int samplerate = snd_file_source->sndfileh_.samplerate();
        int sndfile_format = snd_file_source->sndfileh_.format();
        if (open_result.has_value()) {
            RCLCPP_ERROR(rcl_logger, "Cannot open file <%s>: %s", file_path.c_str(), open_result.value().c_str());
            return 1;
        }
        RCLCPP_INFO(rcl_logger, "Opened file %s: channels=%d, samplerate=%d, format=0x%X",
            file_path.c_str(), channels, samplerate, sndfile_format);

        std::unique_ptr<AlsaSink> alsa_sink = std::make_unique<AlsaSink>(
            ALSA_DEVICE_NAME,
            channels,
            samplerate,
            ALSA_FORMAT
        );
        auto alsa_open_result = alsa_sink->open(SND_PCM_STREAM_PLAYBACK);
        if (alsa_open_result.has_value()) {
            RCLCPP_ERROR(rcl_logger, "Cannot open ALSA device: %s", alsa_open_result->c_str());
            return 1;
        }

        //std::atomic<bool> audio_close(false);
        //std::atomic<bool> audio_available(false);
        boost::lockfree::spsc_queue<std::vector<uint8_t>> audio_queue(10);
        SfgRwFormat rw_format = sfg_format_from_alsa_format(ALSA_FORMAT);

        std::unique_ptr<AudioStream> audio_stream = std::make_unique<AudioStream>(
            &shutdown_flag,
            &data_available,
            &audio_queue,
            rw_format,
            snd_file_source.get(),
            alsa_sink.get()
        );

        std::thread sndfile_stream_thread(&AudioTerminal::run, audio_stream->source_, audio_stream.get());
        std::thread alsa_write_thread(&AudioTerminal::run, audio_stream->sink_, audio_stream.get());

        RCLCPP_INFO(rcl_logger, "Enqueued file %s", argv[k]);
        sndfile_stream_thread.join();
        printf("Sndfile stream thread joined for file %s\n", argv[k]);
        shutdown_flag.store(true);
        data_available.store(true); // Wake up any waiting threads
        data_available.notify_all();
        alsa_write_thread.join();
        printf("End of threads for file %s\n", argv[k]);
        break;
        if (shutdown_flag.load()) {
            break;
        }
    }

    RCLCPP_INFO(rcl_logger, "Shutting down...");
    rclcpp::shutdown();
    return 0;
}
