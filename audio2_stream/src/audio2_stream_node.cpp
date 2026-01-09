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
        SndfileHandle fileh = SndfileHandle(argv[k]);
        if (fileh.error()) {
            RCLCPP_ERROR(rcl_logger, "Cannot open file <%s>: %s", file_path.c_str(), fileh.strError());
            return 1;
        }
        RCLCPP_INFO(rcl_logger, "File opened: %s, Sample Rate: %d, Format %X", file_path.c_str(), fileh.samplerate(), fileh.format());
        printf("channels: %d\n", fileh.channels());

        AlsaHwParams hw_vals;
        hw_vals.channels = fileh.channels();
        hw_vals.samplerate = fileh.samplerate();
        hw_vals.format = ALSA_FORMAT;
        AlsaSwParams sw_vals;
        auto alsa_write = std::make_unique<AlsaWriteThread>(
            hw_vals,
            sw_vals,
            &audio_queue,
            &shutdown_flag,
            &data_available
        );
        auto snd_file_stream = std::make_unique<SndFileStream>(
            fileh,
            ALSA_FORMAT,
            &shutdown_flag,
            &data_available,
            &audio_queue
        );
        std::thread sndfile_stream_thread(&SndFileStream::run, snd_file_stream.get());

        std::thread alsa_write_thread(&AlsaWriteThread::run, alsa_write.get());
        RCLCPP_INFO(rcl_logger, "Enqueued file %s", argv[k]);
        sndfile_stream_thread.join();
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
