#include "sndplay/buffer_file.hpp"

#include <sndfile.h>
#include <cstring>
#include <ios>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
// #include "rclcpp/rclcpp.hpp"

static auto rcl_logger = rclcpp::get_logger("sndplay/buffer_file");

// Virtual I/O callbacks
static sf_count_t vio_get_filelen(void *user_data) {
    VIO_DATA *vio = (VIO_DATA *)user_data;
    return vio->length;
}

static sf_count_t vio_seek(sf_count_t offset, int whence, void *user_data) {
    VIO_DATA *vio = (VIO_DATA *)user_data;
    switch (whence) {
        case SEEK_SET:
            vio->offset = offset;
            break;
        case SEEK_CUR:
            vio->offset += offset;
            break;
        case SEEK_END:
            vio->offset = vio->length + offset;
            break;
        default:
            return -1;
    }
    if (vio->offset < 0) vio->offset = 0;
    if (vio->offset > vio->length) vio->offset = vio->length;
    return vio->offset;
}

static sf_count_t vio_read(void *ptr, sf_count_t count, void *user_data) {
    VIO_DATA *vio = (VIO_DATA *)user_data;
    sf_count_t available = vio->length - vio->offset;
    sf_count_t to_read = (count < available) ? count : available;
    memcpy(ptr, vio->data + vio->offset, to_read);
    vio->offset += to_read;
    return to_read;
}

static sf_count_t vio_write(const void *ptr, sf_count_t count, void *user_data) {
    VIO_DATA *vio = (VIO_DATA *)user_data;
    sf_count_t available = vio->capacity - vio->offset;
    sf_count_t to_write = (count < available) ? count : available;
    
    if (to_write > 0) {
        memcpy((void *)(vio->data + vio->offset), ptr, to_write);
        vio->offset += to_write;
        // Update length if we've written past the current end
        if (vio->offset > vio->length) {
            vio->length = vio->offset;
        }
    }
    return to_write;
}

static sf_count_t vio_tell(void *user_data) {
    VIO_DATA *vio = (VIO_DATA *)user_data;
    return vio->offset;
}

static SF_VIRTUAL_IO vio = {
    vio_get_filelen,
    vio_seek,
    vio_read,
    vio_write,
    vio_tell
};

int open_sndfile_from_buffer(VIO_SOUNDFILE & vio_sndfile, int mode)
{
    vio_sndfile.sndfile = sf_open_virtual(&vio, mode, &vio_sndfile.sfinfo, &vio_sndfile.vio_data);
    if (!vio_sndfile.sndfile) {
        RCLCPP_ERROR(rcl_logger, "Failed to open sound file from buffer: %s", sf_strerror(nullptr));
        return -1;
    }
    
    auto sfinfo = vio_sndfile.sfinfo;
    auto sndfile = vio_sndfile.sndfile;
    SF_FORMAT_INFO major_info, subtype_info;
    subtype_info.format = sfinfo.format & SF_FORMAT_SUBMASK;
    major_info.format = sfinfo.format;
    
    // Get major format info
    if (sf_command (sndfile, SFC_GET_FORMAT_INFO, &major_info, sizeof (major_info))) {
        RCLCPP_ERROR(rcl_logger, "sf_command SFC_GET_FORMAT_MAJOR failed");
        printf("major_info format: %08x\n", major_info.format);
        major_info.name = "Unknown";
    }
    // Get subtype format info
    if (sf_command (sndfile, SFC_GET_FORMAT_INFO, &subtype_info, sizeof (subtype_info))) {
        RCLCPP_ERROR(rcl_logger, "sf_command SFC_GET_FORMAT_SUBTYPE failed");
        subtype_info.name = "Unknown";
    }
    RCLCPP_INFO(rcl_logger, "Opening type [%s %s], %d channels, %d Hz", major_info.name, subtype_info.name, sfinfo.channels, sfinfo.samplerate);
    return 0;
}

static int sfg_write(SNDFILE * sndfile, void * buffer, int format, int samples)
{
    int subtype = format & SF_FORMAT_SUBMASK;
    switch (subtype) {
        case SF_FORMAT_PCM_16:
        case SF_FORMAT_VORBIS:
        case SF_FORMAT_OPUS:
        case SF_FORMAT_MPEG_LAYER_III:
            return sf_write_short(sndfile, reinterpret_cast<short*>(buffer), samples);
        case SF_FORMAT_PCM_32:
            return sf_write_int(sndfile, reinterpret_cast<int*>(buffer), samples);
        case SF_FORMAT_FLOAT:
            return sf_write_float(sndfile, reinterpret_cast<float*>(buffer), samples);
        case SF_FORMAT_DOUBLE:
            return sf_write_double(sndfile, reinterpret_cast<double*>(buffer), samples);
        default:
            return -1;
    }
}

int write_buffer(void* buffer, int format, int frames, int channels, int sample_rate)
{
    SF_INFO sfinfo;
    memset (&sfinfo, 0, sizeof (sfinfo)) ;
    sfinfo.format = format;
    sfinfo.channels = channels;
    sfinfo.samplerate = sample_rate;
    sfinfo.frames = frames;

    printf("Preparing to write buffer: format=%x, frames=%d, channels=%d, sample_rate=%d\n", format, frames, channels, sample_rate);
    const int MAX_HEADER = 128;
    int subtype = sfinfo.format & SF_FORMAT_SUBMASK;
    int sample_size;
    switch (subtype) {
        case SF_FORMAT_PCM_16:
        case SF_FORMAT_OPUS:
        case SF_FORMAT_VORBIS:
        case SF_FORMAT_MPEG_LAYER_III:
            sample_size = sizeof(short);
            break;
        case SF_FORMAT_PCM_32:
            sample_size = sizeof(int);
            break;
        case SF_FORMAT_FLOAT:
            sample_size = sizeof(float);
            break;
        case SF_FORMAT_DOUBLE:
            sample_size = sizeof(double);
            break;
        default:
            RCLCPP_ERROR(rcl_logger, "Unsupported subtype format for writing to buffer");
            return -1;
    }
    int file_size = sfinfo.frames * sfinfo.channels * sample_size + MAX_HEADER;
    printf("Allocating buffer of size %d bytes for writing\n", file_size);
    std::vector<unsigned char> file_data(file_size);
    VIO_SOUNDFILE vio_sndfile;
    vio_sndfile.vio_data.data = file_data.data();
    vio_sndfile.vio_data.length = 0;
    vio_sndfile.vio_data.offset = 0;
    vio_sndfile.vio_data.capacity = static_cast<sf_count_t>(file_data.size());
    vio_sndfile.sfinfo = sfinfo;
    if (open_sndfile_from_buffer(vio_sndfile, SFM_WRITE) != 0) {
        printf("Failed to open sound file for writing to buffer\n");
        return -1;
    }
    int write_count = sfg_write(vio_sndfile.sndfile, buffer, format, frames * channels);
    if (write_count < 0) {
        RCLCPP_ERROR(rcl_logger, "Error writing to buffer: %s", sf_strerror(vio_sndfile.sndfile));
        return -1;
    }
    printf("Wrote %d samples to buffer\n", write_count);
    return 0;
}

PlayBufferParams get_file(char * file_path)
{
    AlsaHwParams hw_vals;
    AlsaSwParams sw_vals;

    printf ("Playing file %s\n", file_path) ;

    // Read entire file into memory
    std::ifstream file(file_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        printf("Error: Cannot open file %s\n", file_path);
        return PlayBufferParams{nullptr, hw_vals, sw_vals};
    }

    std::streamsize file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    auto file_data = std::make_shared<std::vector<unsigned char>>(file_size);
    if (!file.read(reinterpret_cast<char*>(file_data->data()), file_size)) {
        printf("Error: Cannot read file %s\n", file_path);
        return PlayBufferParams{nullptr, hw_vals, sw_vals};
    }
    file.close();
    printf("File size %ld bytes\n", file_size);
    return PlayBufferParams{file_data, hw_vals, sw_vals};
}

//play_buffer thread function that listens to the audio_queue
void
play_buffer_thread(boost::lockfree::spsc_queue<PlayBufferParams>* audio_queue, std::atomic<bool>* shutdown_flag, std::atomic<bool>* data_available)
{
    snd_pcm_t* alsa_dev = nullptr;
    bool first_call = true;
    AlsaHwParams old_hw_vals;
    AlsaSwParams old_sw_vals;

    RCLCPP_INFO(rcl_logger, "Play buffer thread started");

    while (!shutdown_flag->load()) {
        PlayBufferParams params;

        // Try to pop from queue
        data_available->wait(false); // Wait until there's something to process
        data_available->store(false);
        if (!audio_queue->pop(params)) {
            // Check if shutdown has been requested
            if (shutdown_flag && shutdown_flag->load()) {
                RCLCPP_INFO(rcl_logger, "Playback interrupted by shutdown signal");
                break;
            }
            // Queue is empty, sleep briefly and continue. We should not reach this
            RCLCPP_INFO(rcl_logger, "Audio queue is empty, waiting...");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Process the audio buffer
        std::shared_ptr<std::vector<unsigned char>> file_data = params.file_data;
        AlsaHwParams hw_vals = params.hw_vals;
        AlsaSwParams sw_vals = params.sw_vals;
        VIO_SOUNDFILE vio_sndfile;
        
        vio_sndfile.vio_data.data = file_data->data();
        vio_sndfile.vio_data.length = static_cast<sf_count_t>(file_data->size());
        vio_sndfile.vio_data.offset = 0;
        vio_sndfile.vio_data.capacity = static_cast<sf_count_t>(file_data->size());

        if (open_sndfile_from_buffer(vio_sndfile, SFM_READ) != 0) {
            RCLCPP_ERROR(rcl_logger, "Failed to open sound file from buffer");
            continue;
        }
        hw_vals.channels = vio_sndfile.sfinfo.channels;
        hw_vals.samplerate = vio_sndfile.sfinfo.samplerate;
        
        // Check if we need to reopen the device
        bool need_reopen = first_call || !alsa_dev || hw_vals != old_hw_vals || sw_vals != old_sw_vals;
        
        if (need_reopen) {
            if (!first_call) {
                RCLCPP_INFO(rcl_logger, "ALSA device reopened with new parameters");
                if (alsa_dev) {
                    snd_pcm_drain(alsa_dev);
                    snd_pcm_close(alsa_dev);
                    alsa_dev = nullptr;
                }
            }
            if ((alsa_dev = alsa_open(hw_vals, sw_vals)) == nullptr) {
                RCLCPP_ERROR(rcl_logger, "Failed to open ALSA device");
                sf_close(vio_sndfile.sndfile);
                continue;
            }
            old_hw_vals = hw_vals;
            old_sw_vals = sw_vals;
            first_call = false;
        }

        // Capture alsa_dev in a local variable to avoid issues with static storage
        snd_pcm_t* current_alsa_dev = alsa_dev;
        
        // Play the audio
        PlaybackResult result = alsa_play(vio_sndfile.sndfile, vio_sndfile.sfinfo, current_alsa_dev, hw_vals.format, shutdown_flag);

        if (!result.success) {
            RCLCPP_ERROR(rcl_logger, "Playback failed: %s", result.error_message);
        } else {
            RCLCPP_INFO(rcl_logger, "Playback completed successfully");
        }

        // Clean up
        if (vio_sndfile.sndfile) {
            sf_close(vio_sndfile.sndfile);
        }
    }

    if (alsa_dev)
    {
        snd_pcm_drain(alsa_dev);
        snd_pcm_close(alsa_dev);
    }

    RCLCPP_INFO(rcl_logger, "Play buffer thread exiting");
}

