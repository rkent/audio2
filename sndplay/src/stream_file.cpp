/**
 * @file stream_file.cpp
 * @brief Implementation of audio streaming from file buffer to a ROS2 topic.
 */

 #include <sndfile.h>
 #include <atomic>

 #include "rclcpp/rclcpp.hpp"
 #include "sndplay_msgs/msg/audio_data.hpp"
 #include "sndplay/buffer_file.hpp"
 
 // Global flag to signal thread shutdown
std::atomic<bool> shutdown_flag(false);
// Global flag to signify that new data is available in the queue
std::atomic<bool> data_available(false);

static auto rcl_logger = rclcpp::get_logger("sndplay/stream_file");

void signal_handler(int signal) {
    if (signal == SIGINT) {
        RCLCPP_INFO(rcl_logger, "Keyboard interrupt received. Shutting down.");
        shutdown_flag.store(true);
        rclcpp::shutdown();
    }
}

class FileStreamerNode : public rclcpp::Node {
public:
    FileStreamerNode() : Node("file_streamer") {
        publisher_ = this->create_publisher<sndplay_msgs::msg::AudioData>("file_publish", 10);
    }
    void publish_file_data(const std::string & file_path) {
        RCLCPP_INFO(rcl_logger, "Streaming audio data from file: %s", file_path.c_str());
        SF_INFO file_sfinfo;
        file_ = sf_open(file_path.c_str(), SFM_READ, &file_sfinfo);
        if (file_ == nullptr) {
            RCLCPP_ERROR(rcl_logger, "Cannot open file %s", file_path.c_str());
            return;
        }
        printf("File opened: %s, Sample Rate: %d, Format %X\n", file_path.c_str(), file_sfinfo.samplerate, file_sfinfo.format);
        printf("channels: %d, sections: %d, seekable: %d\n", file_sfinfo.channels, file_sfinfo.sections, file_sfinfo.seekable);
        RCLCPP_INFO(rcl_logger, "File opened: %s, Sample Rate: %d, Format %X", file_path.c_str(), file_sfinfo.samplerate, file_sfinfo.format);

        int file_sample_size_ = sample_size_from_format(file_sfinfo.format);
        const int MAX_HEADER = 128;
        const int read_frames = 32000;
        int file_buffer_size = read_frames * file_sfinfo.channels * file_sample_size_ + MAX_HEADER;
        std::vector<unsigned char> file_buffer(file_buffer_size);

        // Virtual file for topic publish
        VIO_SOUNDFILE f_vio_sndfile;
        SF_INFO topic_sfinfo;
        topic_sfinfo.samplerate = file_sfinfo.samplerate;
        topic_sfinfo.channels = file_sfinfo.channels;
        topic_sfinfo.format = SF_FORMAT_MPEG | SF_FORMAT_MPEG_LAYER_III;
        f_vio_sndfile.vio_data.data = file_buffer.data();
        f_vio_sndfile.vio_data.length = 0;
        f_vio_sndfile.vio_data.offset = 0;
        f_vio_sndfile.vio_data.capacity = static_cast<sf_count_t>(file_buffer.size());
        f_vio_sndfile.sfinfo = topic_sfinfo;
        if (open_sndfile_from_buffer(f_vio_sndfile, SFM_WRITE) != 0) {
            RCLCPP_ERROR(rcl_logger, "Failed to open virtual sound file for topic publishing");
            sf_close(file_);
            return;
        }

        int topic_sample_size = sample_size_from_format(topic_sfinfo.format);
        int topic_file_size = read_frames * topic_sfinfo.channels * topic_sample_size + MAX_HEADER;
        // printf("Allocating buffer of size %d bytes for writing\n", topic_file_size);

        int samples_read = 0;
        do {
            samples_read = sfg_read(file_, &file_sfinfo, file_buffer.data(), read_frames * file_sfinfo.channels);
            printf("Read %d samples from file\n", samples_read);
            std::vector<unsigned char> topic_buffer(topic_file_size);
            VIO_SOUNDFILE t_vio_sndfile;
            t_vio_sndfile.vio_data.data = topic_buffer.data();
            t_vio_sndfile.vio_data.length = 0;
            t_vio_sndfile.vio_data.offset = 0;
            t_vio_sndfile.vio_data.capacity = static_cast<sf_count_t>(topic_buffer.size());
            t_vio_sndfile.sfinfo = topic_sfinfo;
            if (open_sndfile_from_buffer(t_vio_sndfile, SFM_WRITE) != 0) {
                printf("Failed to open sound file for writing to buffer\n");
                return;
            }
            int samples_written = sfg_write(t_vio_sndfile.sndfile, file_buffer.data(), topic_sfinfo.format, samples_read);
            if (samples_written != samples_read) {
                RCLCPP_ERROR(rcl_logger, "samples_written %d does not match expected %d", samples_written, samples_read);
                sf_close(t_vio_sndfile.sndfile);
                break;
            }
            printf("length after write: %ld offset: %ld capacity: %ld\n", t_vio_sndfile.vio_data.length, t_vio_sndfile.vio_data.offset, t_vio_sndfile.vio_data.capacity);
            sf_close(t_vio_sndfile.sndfile);
        } while (samples_read > 0);
        sf_close(file_);
        return;
    }
private:
    rclcpp::Publisher<sndplay_msgs::msg::AudioData>::SharedPtr publisher_;
    SNDFILE* file_;
};

int main(int argc, char ** argv)
{
    std::signal(SIGINT, signal_handler); // Register the signal handler
    rclcpp::init(argc, argv);

    auto file_publisher = std::make_shared<FileStreamerNode>();
    // Enqueue all files from command line arguments
    file_publisher-> publish_file_data(argv[1]);
    rclcpp::spin(file_publisher);
    rclcpp::shutdown();
    return 0;
}