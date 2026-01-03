/**
 * @file loop_file.cpp
 * @brief Test file read, loop through snd_file buffers to the ALSA device.
 */

#include <sndfile.h>
#include <atomic>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "audio2_stream_msgs/msg/audio_data.hpp"
#include "audio2_stream/buffer_file.hpp" 

#define TOPIC_FORMAT (SF_FORMAT_WAV | SF_FORMAT_PCM_32)
//#define TOPIC_FORMAT (SF_FORMAT_OGG | SF_FORMAT_VORBIS)
//#define TOPIC_FORMAT (SF_FORMAT_OGG | SF_FORMAT_OPUS) 
//#define TOPIC_FORMAT (SF_FORMAT_MPEG | SF_FORMAT_MPEG_LAYER_III)
#define ALSA_FORMAT SND_PCM_FORMAT_S16

 // Global flag to signal thread shutdown
std::atomic<bool> shutdown_flag(false);
// Global flag to signify that new data is available in the queue
std::atomic<bool> data_available(false);

static auto rcl_logger = rclcpp::get_logger("audio2_stream/loop_file");

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
        publisher_ = this->create_publisher<audio2_stream_msgs::msg::AudioData>("file_publish", 10);
    }
    void publish_file_data(const std::string & file_path) {
        // Open the sound file
        RCLCPP_INFO(rcl_logger, "Streaming audio data from file: %s", file_path.c_str());
        SF_INFO file_sfinfo;
        file_ = sf_open(file_path.c_str(), SFM_READ, &file_sfinfo);
        if (file_ == nullptr) {
            RCLCPP_ERROR(rcl_logger, "Cannot open file %s", file_path.c_str());
            return;
        }
        RCLCPP_INFO(rcl_logger, "File opened: %s, Sample Rate: %d, Format %X", file_path.c_str(), file_sfinfo.samplerate, file_sfinfo.format);
        printf("channels: %d, sections: %d, seekable: %d\n", file_sfinfo.channels, file_sfinfo.sections, file_sfinfo.seekable);

        // Disable normalization since we are handling scaling ourselves.
        sf_command (file_, SFC_SET_NORM_FLOAT, NULL, SF_FALSE) ;
        sf_command (file_, SFC_SET_NORM_DOUBLE, NULL, SF_FALSE) ;

        SfgRwFormat file_rw_format_ = sfg_format_from_sndfile_format(file_sfinfo.format);
        int file_sample_size_ = sample_size_from_sfg_format(file_rw_format_);
        const int MAX_HEADER = 128;
        const int read_frames = 1024 * 16;
        int file_buffer_size = read_frames * file_sfinfo.channels * file_sample_size_;
        std::vector<char> file_buffer(file_buffer_size);

        // Virtual file for topic publish
        SF_INFO topic_sfinfo;
        topic_sfinfo.samplerate = file_sfinfo.samplerate;
        topic_sfinfo.channels = file_sfinfo.channels;
        topic_sfinfo.format = TOPIC_FORMAT;
        SfgRwFormat topic_rw_format_ = sfg_format_from_sndfile_format(topic_sfinfo.format);
        int topic_sample_size = sample_size_from_sfg_format(topic_rw_format_);
        int topic_file_size = read_frames * topic_sfinfo.channels * topic_sample_size + MAX_HEADER;
        printf("Topic publish file size: %d bytes\n", topic_file_size);

        RCLCPP_INFO(rcl_logger, "Topic publish format: %s", format_to_string(topic_sfinfo.format).c_str());

        int samples_read = 0;
        AlsaHwParams hw_vals;
        hw_vals.channels = file_sfinfo.channels;
        hw_vals.samplerate = file_sfinfo.samplerate;
        hw_vals.format = ALSA_FORMAT;
        AlsaSwParams sw_vals;
        snd_pcm_t * alsa_dev = alsa_open(hw_vals, sw_vals);
        if (alsa_dev == nullptr) {
            RCLCPP_ERROR(rcl_logger, "Failed to open ALSA device for topic publish");
            return;
        }

        do {
            samples_read = sfg_read2(file_, file_rw_format_, file_buffer.data(), read_frames * file_sfinfo.channels);
            printf("Read %d samples from file using format %s\n", samples_read, sfg_format_to_string(file_rw_format_));
            if (samples_read <= 0) {
                if (samples_read < 0) {
                    RCLCPP_ERROR(rcl_logger, "Error reading from file: %s %d", sf_strerror(file_), samples_read);
                }
                sf_close(file_);
                break;
            }

            // Create a virtual sound file in memory for topic publish
            std::vector<char> topic_buffer(topic_file_size);
            VIO_SOUNDFILE t_vio_sndfile;
            t_vio_sndfile.vio_data.data = topic_buffer.data();
            t_vio_sndfile.vio_data.length = 0;
            t_vio_sndfile.vio_data.offset = 0;
            t_vio_sndfile.vio_data.capacity = static_cast<sf_count_t>(topic_buffer.size());
            t_vio_sndfile.sfinfo = topic_sfinfo;
            if (auto err = open_sndfile_from_buffer(t_vio_sndfile, SFM_WRITE)) {
                printf("Failed to open sound file for writing to buffer: %s\n", err->c_str());
                return;
            }

            // Set compression level for OGG/OPUS
            //double compression_level = 0.5; // 0.0 (fastest) to 1.0 (best)
            //sf_command(t_vio_sndfile.sndfile, SFC_SET_COMPRESSION_LEVEL, &compression_level, sizeof(compression_level));
            //int mode = SF_BITRATE_MODE_VARIABLE; // Variable bitrate mode
            //sf_command(t_vio_sndfile.sndfile, SFC_SET_BITRATE_MODE, &mode, sizeof(mode));
            //sf_command(t_vio_sndfile.sndfile, SFC_SET_VBR_ENCODING_QUALITY, &compression_level, sizeof(compression_level));
            //mode = sf_command(t_vio_sndfile.sndfile, SFC_GET_BITRATE_MODE, NULL, 0);
            //printf("Bitrate mode: %d\n", mode);

            int samples_written = sfg_write_convert(t_vio_sndfile.sndfile, file_rw_format_, topic_rw_format_, file_buffer.data(), samples_read);
            if (samples_written != samples_read) {
                RCLCPP_ERROR(rcl_logger, "samples_written %d does not match expected %d", samples_written, samples_read);
                sf_close(t_vio_sndfile.sndfile);
                break;
            }
            printf("Wrote %d samples to virtual topic buffer using format %s\n", samples_written, sfg_format_to_string(topic_rw_format_));
            
            // Reopen the file for reading
            sf_close(t_vio_sndfile.sndfile);
            double compression_ratio = static_cast<double>(t_vio_sndfile.vio_data.length) /
                static_cast<double>(samples_written * topic_sample_size);
            printf("length after write: %ld offset: %ld capacity: %ld compression_ratio: %.3f\n", t_vio_sndfile.vio_data.length,
              t_vio_sndfile.vio_data.offset, t_vio_sndfile.vio_data.capacity, compression_ratio);
            // VIO_SOUNDFILE t_vio_sndfile;
            //t_vio_sndfile.vio_data.data = topic_buffer.data();
            t_vio_sndfile.vio_data.length = static_cast<sf_count_t>(topic_buffer.size());
            t_vio_sndfile.vio_data.offset = 0;
            //t_vio_sndfile.vio_data.capacity = static_cast<sf_count_t>(topic_buffer.size());
            //t_vio_sndfile.sfinfo = topic_sfinfo;
            if (auto err = open_sndfile_from_buffer(t_vio_sndfile, SFM_READ)) {
                RCLCPP_ERROR(rcl_logger, "Failed to open sound file for reading from buffer: %s", err->c_str());
                return;
            }

            // sf_seek(t_vio_sndfile.sndfile, 0, SF_SEEK_SET);
            auto err = alsa_play(t_vio_sndfile.sndfile, t_vio_sndfile.sfinfo, alsa_dev, hw_vals.format, &shutdown_flag);
            if (err) {
                RCLCPP_ERROR(rcl_logger, "ALSA play error: %s", err->c_str());
            }
            
            sf_close(t_vio_sndfile.sndfile);
        } while (!shutdown_flag.load());
        snd_pcm_drain(alsa_dev);
        snd_pcm_close(alsa_dev);
        return;
    }
private:
    rclcpp::Publisher<audio2_stream_msgs::msg::AudioData>::SharedPtr publisher_;
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