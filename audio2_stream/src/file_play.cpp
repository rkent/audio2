/**
 * @file file_play.cpp
 * @brief Test file read, loop through snd_file buffers to the ALSA device.
 */

#include <sndfile.h>
#include <atomic>
#include <fstream>
#include <sndfile.hh>

#include "rclcpp/rclcpp.hpp"
#include "audio2_stream/buffer_file.hpp"

#define ALSA_FORMAT SND_PCM_FORMAT_S16

 // Global flag to signal thread shutdown
std::atomic<bool> shutdown_flag(false);

static auto rcl_logger = rclcpp::get_logger("audio2_stream/file_play");

void signal_handler(int signal) {
    if (signal == SIGINT) {
        RCLCPP_INFO(rcl_logger, "Keyboard interrupt received. Shutting down.");
        shutdown_flag.store(true);
        rclcpp::shutdown();
    }
}

class FilePlayerNode : public rclcpp::Node {
public:
    FilePlayerNode() : Node("file_play") {
    }
    void play_file_data(const std::string & file_path) {
        RCLCPP_INFO(rcl_logger, "Playing audio data from file: %s", file_path.c_str());
        // Open the sound file
        SndfileHandle fileh = SndfileHandle(file_path);
        if (fileh.error()) {
            RCLCPP_ERROR(rcl_logger, "Cannot open file <%s>: %s", file_path.c_str(), fileh.strError());
            return;
        }
        RCLCPP_INFO(rcl_logger, "File opened: %s, Sample Rate: %d, Format %X", file_path.c_str(), fileh.samplerate(), fileh.format());
        printf("channels: %d\n", fileh.channels());

        AlsaHwParams hw_vals;
        hw_vals.channels = fileh.channels();
        hw_vals.samplerate = fileh.samplerate();
        hw_vals.format = ALSA_FORMAT;
        AlsaSwParams sw_vals;
        snd_pcm_t * alsa_dev = alsa_open(hw_vals, sw_vals);
        if (alsa_dev == nullptr) {
            RCLCPP_ERROR(rcl_logger, "Failed to open ALSA device for file play");
            return;
        }

        // Disable normalization since we are handling scaling ourselves.
        fileh.command(SFC_SET_NORM_FLOAT, NULL, SF_FALSE) ;
        fileh.command(SFC_SET_NORM_DOUBLE, NULL, SF_FALSE) ;

        SfgRwFormat file_rw_format = sfg_format_from_sndfile_format(fileh.format());
        int file_sample_size = sample_size_from_sfg_format(file_rw_format);
        const int read_frames = 1024 * 16;
        int file_buffer_size = read_frames * fileh.channels() * file_sample_size;
        std::vector<char> file_buffer(file_buffer_size);

        auto err = alsa_play(fileh, alsa_dev, hw_vals.format, &shutdown_flag);
        if (err) {
            RCLCPP_ERROR(rcl_logger, "ALSA play error: %s", err->c_str());
        }
        snd_pcm_drain(alsa_dev);
        snd_pcm_close(alsa_dev);
        return;
    }
};

int main(int argc, char ** argv)
{
    std::signal(SIGINT, signal_handler); // Register the signal handler
    rclcpp::init(argc, argv);

    auto file_player = std::make_shared<FilePlayerNode>();
    // Enqueue all files from command line arguments
    file_player->play_file_data(argv[1]);
    rclcpp::spin(file_player);
    rclcpp::shutdown();
    return 0;
}