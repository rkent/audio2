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
#include "sndplay/alsaops.hpp"
#include "sndplay/buffer_file.hpp"
#include "boost/lockfree/spsc_queue.hpp"

// Global flag to signal thread shutdown
std::atomic<bool> shutdown_flag(false);
// Global flag to signify that new data is available in the queue
std::atomic<bool> data_available(false);

static auto rcl_logger = rclcpp::get_logger("sndplay");

boost::lockfree::spsc_queue<PlayBufferParams> audio_queue(10); // Queue size of 10

void signal_handler(int signal) {
    if (signal == SIGINT) {
        RCLCPP_INFO(rcl_logger, "Keyboard interrupt received. Shutting down.");
        shutdown_flag.store(true);
        data_available.store(true); // Wake up any waiting threads
        data_available.notify_all();
        rclcpp::shutdown();
    }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
{
    std::signal(SIGINT, signal_handler); // Register the signal handler
    rclcpp::init(argc, argv);

    // thread wakeup mechanism. Might be better to use atomic ready and wait in C++20

    boost::lockfree::spsc_queue<PlayBufferParams> audio_queue(10); // Queue size of 10
    // Start play_buffer thread
    std::thread playback_thread(play_buffer_thread, &audio_queue, &shutdown_flag, &data_available);

    // Enqueue all files from command line arguments
    for (int k = 1; k < argc; k++) {
        PlayBufferParams params = get_file(argv[k]);
        if (params.file_data == nullptr) {
            RCLCPP_ERROR(rcl_logger, "Failed to load file %s", argv[k]);
            continue; // Error reading file
        }

        // Wait until queue has space
        while (!audio_queue.push(params) && !shutdown_flag.load()) {
            RCLCPP_WARN(rcl_logger, "Audio queue is full, waiting...");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        data_available.store(true);
        data_available.notify_one();

        if (shutdown_flag.load()) {
            break;
        }
        RCLCPP_INFO(rcl_logger, "Enqueued file %s", argv[k]);
    }

    // Wait for playback thread to finish processing all items
    RCLCPP_INFO(rcl_logger, "Waiting for playback to complete...");
    while (!audio_queue.empty() && !shutdown_flag.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Signal shutdown and wait for thread to exit
    // shutdown_flag.store(true);
    playback_thread.join();

    RCLCPP_INFO(rcl_logger, "Shutting down...");
    rclcpp::shutdown();
    return 0;
}
