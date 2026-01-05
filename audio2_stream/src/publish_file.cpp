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
#include "audio2_stream_msgs/msg/audio_chunk.hpp"

// Global flag to signal thread shutdown
std::atomic<bool> shutdown_flag(false);
// Global flag to signify that new data is available in the queue
std::atomic<bool> data_available(false);

static auto rcl_logger = rclcpp::get_logger("audio2_stream/publish_file");

void signal_handler(int signal) {
    if (signal == SIGINT) {
        RCLCPP_INFO(rcl_logger, "Keyboard interrupt received. Shutting down.");
        shutdown_flag.store(true);
        rclcpp::shutdown();
    }
}

class FilePublisherNode : public rclcpp::Node {
public:
    FilePublisherNode() : Node("file_publisher") {
        publisher_ = this->create_publisher<audio2_stream_msgs::msg::AudioChunk>("file_publish", 10);
    }
    void publish_file_data(const std::string & file_path) {
        RCLCPP_INFO(rcl_logger, "Publishing audio data from file: %s", file_path.c_str());
        std::ifstream file(file_path, std::ios::binary | std::ios::ate);
        if (!file.is_open()) {
            RCLCPP_ERROR(rcl_logger, "Cannot open file %s", file_path.c_str());
            return;
        }
        std::streamsize file_size = file.tellg();
        file.seekg(0, std::ios::beg);
        RCLCPP_INFO(rcl_logger, "File size: %ld bytes", file_size);
        std::vector<uint8_t> buffer(file_size);
        if (!file.read(reinterpret_cast<char*>(buffer.data()), file_size)) {
            RCLCPP_ERROR(rcl_logger, "Cannot read file %s", file_path.c_str());
            return;
        }
        RCLCPP_INFO(rcl_logger, "Read %ld bytes from file %s", file_size, file_path.c_str());
        auto message = audio2_stream_msgs::msg::AudioChunk();
        message.data = buffer;
        while (rclcpp::ok()) {
            RCLCPP_INFO(rcl_logger, "Publishing audio data...");
            publisher_->publish(message);
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        }
        RCLCPP_INFO(rcl_logger, "Published audio data from file %s", file_path.c_str());
    }
private:
    rclcpp::Publisher<audio2_stream_msgs::msg::AudioChunk>::SharedPtr publisher_;
};

int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
{
    std::signal(SIGINT, signal_handler); // Register the signal handler
    rclcpp::init(argc, argv);

    auto file_publisher = std::make_shared<FilePublisherNode>();
    // Enqueue all files from command line arguments
    file_publisher-> publish_file_data(argv[1]);
    rclcpp::spin(file_publisher);
    rclcpp::shutdown();
    return 0;
}
