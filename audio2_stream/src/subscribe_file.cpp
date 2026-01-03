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
#include "audio2_stream_msgs/msg/audio_data.hpp"

// Global flag to signal thread shutdown
std::atomic<bool> shutdown_flag(false);
// Global flag to signify that new data is available in the queue
std::atomic<bool> data_available(false);

AlsaHwParams hw_vals;
AlsaSwParams sw_vals;

static auto rcl_logger = rclcpp::get_logger("audio2_stream/subscribe_file");

void signal_handler(int signal) {
    if (signal == SIGINT) {
        RCLCPP_INFO(rcl_logger, "Keyboard interrupt received. Shutting down.");
        data_available.store(true); // Wake up any waiting threads
        data_available.notify_all();
        shutdown_flag.store(true);
        rclcpp::shutdown();
    }
}

class FileSubscriberNode : public rclcpp::Node {
public:
    FileSubscriberNode(boost::lockfree::spsc_queue<PlayBufferParams>* queue) 
        : Node("file_subscribe"), audio_queue_(queue) {
        subscriber_ = this->create_subscription<audio2_stream_msgs::msg::AudioData>(
            "file_publish", 10,
            std::bind(&FileSubscriberNode::audio_data_callback, this, std::placeholders::_1));
    }
    void audio_data_callback(const audio2_stream_msgs::msg::AudioData::SharedPtr msg) {
        RCLCPP_INFO(rcl_logger, "Received audio data");
        auto file_data = std::make_shared<std::vector<char>>(msg->data.begin(), msg->data.end());
        PlayBufferParams params{file_data, hw_vals, sw_vals};
        // Wait until queue has space
        while (!audio_queue_->push(params) && !shutdown_flag.load()) {
            RCLCPP_WARN(rcl_logger, "Audio queue is full, waiting...");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        data_available.store(true);
        data_available.notify_one();
    }
private:
    rclcpp::Subscription<audio2_stream_msgs::msg::AudioData>::SharedPtr subscriber_;
    boost::lockfree::spsc_queue<PlayBufferParams>* audio_queue_;
};

int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
{
    std::signal(SIGINT, signal_handler); // Register the signal handler
    rclcpp::init(argc, argv);

    boost::lockfree::spsc_queue<PlayBufferParams> audio_queue(10); // Queue size of 10
    // Start play_buffer thread
    std::thread playback_thread(play_buffer_thread, &audio_queue, &shutdown_flag, &data_available);

    auto file_subscriber = std::make_shared<FileSubscriberNode>(&audio_queue);
    rclcpp::spin(file_subscriber);
    rclcpp::shutdown();
    return 0;
}
