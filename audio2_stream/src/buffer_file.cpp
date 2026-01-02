#include "audio2_stream/buffer_file.hpp"

#include <algorithm>
#include <sndfile.h>
#include <cstring>
#include <ios>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
// #include "rclcpp/rclcpp.hpp"

static auto rcl_logger = rclcpp::get_logger("audio2_stream/buffer_file");

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

// Convert sndfile format to human-readable string.
std::string format_to_string(int format)
{
    SF_FORMAT_INFO major_info, subtype_info;
    subtype_info.format = format & SF_FORMAT_SUBMASK;
    major_info.format = format;

    // Get major format info
    if (sf_command (NULL, SFC_GET_FORMAT_INFO, &major_info, sizeof (major_info))) {
        major_info.name = "Unknown";
    }
    // Get subtype format info
    if (sf_command (NULL, SFC_GET_FORMAT_INFO, &subtype_info, sizeof (subtype_info))) {
        subtype_info.name = "Unknown";
    }
    if (std::strcmp(major_info.name, subtype_info.name)) {
        return std::string(major_info.name) + '(' + std::string(subtype_info.name) + ')';
    }
    return std::string(major_info.name);
}

std::optional<std::string> open_sndfile_from_buffer(VIO_SOUNDFILE & vio_sndfile, int mode)
{
    vio_sndfile.sndfile = sf_open_virtual(&vio, mode, &vio_sndfile.sfinfo, &vio_sndfile.vio_data);
    if (!vio_sndfile.sndfile) {
        return std::string("Failed to open sound file from buffer: ") + sf_strerror(nullptr);
    }
    return std::nullopt;
}

int sfg_write(SNDFILE * sndfile, void * buffer, snd_pcm_format_t format, int samples)
{
    int bytes_per_sample = snd_pcm_format_width(format) / 8;
    bool is_unsigned = snd_pcm_format_unsigned(format);
    bool is_float = snd_pcm_format_float(format);

    if (samples <= 0) {
        return 0;
    }

    if (is_float) {
        if (bytes_per_sample == 4) {
            return sf_write_float(sndfile, reinterpret_cast<float*>(buffer), samples);
        } else if (bytes_per_sample == 8) {
            return sf_write_double(sndfile, reinterpret_cast<double*>(buffer), samples);
        } else {
            return -2; // Unsupported float byte size
        }
    } else {
        if (is_unsigned) {
            // Currently not handling unsigned formats
            return -1;
        }
        if (bytes_per_sample == 2) {
            return sf_write_short(sndfile, reinterpret_cast<short*>(buffer), samples);
        } else if (bytes_per_sample == 4) {
            return sf_write_int(sndfile, reinterpret_cast<int*>(buffer), samples);
        } else {
            return -3; // Unsupported integer byte size
        }
    }
    return -4; // Should not reach here
}

// Get sample size in bytes from alsa's snd_pcm_format_t
int sample_size_from_format(snd_pcm_format_t format)
{
    return snd_pcm_format_width(format) / 8;
}

int sfg_read(SNDFILE * sndfile, snd_pcm_format_t format, void * buffer, int samples)
{
    int bytes_per_sample = snd_pcm_format_width(format) / 8;
    bool is_unsigned = snd_pcm_format_unsigned(format);
    bool is_float = snd_pcm_format_float(format);

    if (samples <= 0) {
        return 0;
    }

    if (is_float) {
        if (bytes_per_sample == 4) {
            return sf_read_float(sndfile, reinterpret_cast<float*>(buffer), samples);
        } else if (bytes_per_sample == 8) {
            return sf_read_double(sndfile, reinterpret_cast<double*>(buffer), samples);
        } else {
            return -2; // Unsupported float byte size
        }
    } else {
        if (is_unsigned) {
            // Currently not handling unsigned formats
            return -1;
        }
        if (bytes_per_sample == 2) {
            return sf_read_short(sndfile, reinterpret_cast<short*>(buffer), samples);
        } else if (bytes_per_sample == 4) {
            return sf_read_int(sndfile, reinterpret_cast<int*>(buffer), samples);
        } else {
            return -3; // Unsupported integer byte size
        }
    }
    return -4; // Should not reach here
}

/*
int write_buffer(void* buffer, int sf_format, snd_pcm_format_t snd_format, int frames, int channels, int sample_rate)
{
    SF_INFO sfinfo;
    memset (&sfinfo, 0, sizeof (sfinfo)) ;
    sfinfo.format = sf_format;
    sfinfo.channels = channels;
    sfinfo.samplerate = sample_rate;
    sfinfo.frames = frames;

    printf("Preparing to write buffer: sf_format=%x, frames=%d, channels=%d, sample_rate=%d\n", sf_format, frames, channels, sample_rate);
    const int MAX_HEADER = 128;
    int sample_size = sample_size_from_format(snd_format);
    if (sample_size < 0) {
        RCLCPP_ERROR(rcl_logger, "Unsupported format for writing buffer: %x", sfinfo.format);
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
    if (auto err = open_sndfile_from_buffer(vio_sndfile, SFM_WRITE)) {
        printf("Failed to open sound file for writing to buffer: %s\n", err->c_str());
        return -1;
    }
    int write_count = sfg_write(vio_sndfile.sndfile, buffer, snd_format, frames * channels);
    if (write_count < 0) {
        RCLCPP_ERROR(rcl_logger, "Error writing to buffer: %s", sf_strerror(vio_sndfile.sndfile));
        return -1;
    }
    printf("Wrote %d samples to buffer\n", write_count);
    return 0;
}
*/

// Read an entire binary file into memory.
std::optional<std::string> get_file(const char * file_path, std::shared_ptr<std::vector<unsigned char>> & out_data)
{
    // Read entire file into memory
    std::ifstream file(file_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        return std::string("Cannot open file: ") + file_path;
    }

    std::streamsize file_size = file.tellg();
    file.seekg(0, std::ios::beg);
    out_data = std::make_shared<std::vector<unsigned char>>(file_size);
    if (!file.read(reinterpret_cast<char*>(out_data->data()), file_size)) {
        file.close();
        return std::string("Cannot read file: ") + file_path;
    }
    file.close();
    return std::nullopt;
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

        if (auto err = open_sndfile_from_buffer(vio_sndfile, SFM_READ)) {
            RCLCPP_ERROR(rcl_logger, err->c_str());
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
        auto result = alsa_play(vio_sndfile.sndfile, vio_sndfile.sfinfo, current_alsa_dev, hw_vals.format, shutdown_flag);

        if (result) {
            RCLCPP_ERROR(rcl_logger, "Playback failed: %s", result->c_str() );
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

#define	BUFFER_LEN			(8192*8)

// Scale factors for converting float to integer formats
float SCALE_S8 = 1LL<<7;
float SCALE_S16 = 1LL<<15;
float SCALE_S20 = 1LL<<19;
float SCALE_S24 = 1LL<<23;
float SCALE_S32 = 1LL<<31;

int
scale_data(int readcount, snd_pcm_format_t read_format, snd_pcm_format_t alsa_format, void* r_buffer, void* w_buffer, float scale)
{
    // TODO: return r_buffer directly if same data type
    // printf("Scaling data: readcount=%d, read_format=%x, alsa_format=%x, scale=%f\n", readcount, read_format, alsa_format, scale) ;
    for (int i = 0; i < readcount; ++i) {
        if (read_format == SND_PCM_FORMAT_S32) {
            if (alsa_format == SND_PCM_FORMAT_S32) {
                (reinterpret_cast<int*>(w_buffer))[i] = (reinterpret_cast<int*>(r_buffer))[i];
                continue;
            } else if (alsa_format == SND_PCM_FORMAT_S16) {
                (reinterpret_cast<short*>(w_buffer))[i] = static_cast<short>((reinterpret_cast<int*>(r_buffer))[i]);
                continue;
            }
            float value = (reinterpret_cast<int*>(r_buffer))[i] / scale;
            if (alsa_format == SND_PCM_FORMAT_FLOAT) {
                (reinterpret_cast<float*>(w_buffer))[i] = value;
            } else if (alsa_format == SND_PCM_FORMAT_FLOAT64) {
                (reinterpret_cast<double*>(w_buffer))[i] = static_cast<double>(value);
            } else {
                return -1; // Unsupported ALSA format for int playback
            }
        } else if (read_format == SND_PCM_FORMAT_S16) {
            if (alsa_format == SND_PCM_FORMAT_S16) {
                (reinterpret_cast<short*>(w_buffer))[i] = (reinterpret_cast<short*>(r_buffer))[i];
                continue;
            } else if (alsa_format == SND_PCM_FORMAT_S32) {
                (reinterpret_cast<int*>(w_buffer))[i] = static_cast<int>((reinterpret_cast<short*>(r_buffer))[i]);
                continue;
            }
            float value = static_cast<float>((reinterpret_cast<short*>(r_buffer))[i]) / scale;
            // printf("Scaled S16 value: %f\n", value) ;
            if (alsa_format == SND_PCM_FORMAT_FLOAT) {
                (reinterpret_cast<float*>(w_buffer))[i] = value;
                continue;
            } else if (alsa_format == SND_PCM_FORMAT_FLOAT64) {
                (reinterpret_cast<double*>(w_buffer))[i] = static_cast<double>(value);
                continue;
            } else {
                return -2; // Unsupported ALSA format for short playback
            }
        } else if (read_format == SND_PCM_FORMAT_FLOAT) {
            if (alsa_format == SND_PCM_FORMAT_FLOAT) {
                (reinterpret_cast<float*>(w_buffer))[i] = (reinterpret_cast<float*>(r_buffer))[i];
                continue;
            } else if (alsa_format == SND_PCM_FORMAT_FLOAT64) {
                (reinterpret_cast<double*>(w_buffer))[i] = static_cast<double>((reinterpret_cast<float*>(r_buffer))[i]);
                continue;
            }
            float value = (reinterpret_cast<float*>((r_buffer))[i]) * scale;
            if (alsa_format == SND_PCM_FORMAT_S16) {
                (reinterpret_cast<short*>(w_buffer))[i] = static_cast<short>(value);
            } else if (alsa_format == SND_PCM_FORMAT_S32) {
                (reinterpret_cast<int*>(w_buffer))[i] = static_cast<int>(value);
            } else {
                return -3; // Unsupported ALSA format for float playback
            }
        } else if (read_format == SND_PCM_FORMAT_FLOAT64) {
            if (alsa_format == SND_PCM_FORMAT_FLOAT64) {
                (reinterpret_cast<double*>(w_buffer))[i] = (reinterpret_cast<double*>(r_buffer))[i];
                continue;
            } else if (alsa_format == SND_PCM_FORMAT_FLOAT) {
                (reinterpret_cast<float*>(w_buffer))[i] = static_cast<float>((reinterpret_cast<double*>(r_buffer))[i]);
                continue;
            }
            double value = (reinterpret_cast<double*>(r_buffer))[i] * static_cast<double>(scale);
            if (alsa_format == SND_PCM_FORMAT_S16) {
                (reinterpret_cast<short*>(w_buffer))[i] = static_cast<short>(value);
                continue;
            } else if (alsa_format == SND_PCM_FORMAT_S32) {
                (reinterpret_cast<int*>(w_buffer))[i] = static_cast<int>(value);
                continue;
            } else {
                return -4; // Unsupported ALSA format for double playback
            }
        } else {
            return -5; // Unsupported read type for scaling
        }
    }
    return 0;
}

// debug string of ALSA format
std::string format_to_string(snd_pcm_format_t format)
{
    switch (format) {
        case SND_PCM_FORMAT_S8:
            return "S8";
        case SND_PCM_FORMAT_U8:
            return "U8";
        case SND_PCM_FORMAT_S16:
            return "S16";
        case SND_PCM_FORMAT_U16:
            return "U16";
        case SND_PCM_FORMAT_S24:
            return "S24";
        case SND_PCM_FORMAT_U24:
            return "U24";
        case SND_PCM_FORMAT_S32:
            return "S32";
        case SND_PCM_FORMAT_U32:
            return "U32";
        case SND_PCM_FORMAT_FLOAT:
            return "FLOAT";
        case SND_PCM_FORMAT_FLOAT64:
            return "FLOAT64";
        default:
            return "UNKNOWN";
    }
}

// Play audio from a SNDFILE using ALSA
std::optional<std::string>
alsa_play (SNDFILE *sndfile, SF_INFO sfinfo, snd_pcm_t* alsa_dev, snd_pcm_format_t alsa_format, std::atomic<bool>* shutdown_flag)
{
    static char r_buffer [BUFFER_LEN] ; // read buffer
    static char w_buffer [BUFFER_LEN] ; // write buffer

    int	readcount;
    snd_pcm_format_t read_format;

    // When converting between float to integer formats, sndfile scaling typically relies on
    // normalization, which we cannot do since the data is being streamed. Therefore, we apply
    // a fixed scaling factor here to avoid extremely low volume playback.

    float scale = 1.0;
    int subformat = sfinfo.format & SF_FORMAT_SUBMASK ;
    switch (subformat) {
        case SF_FORMAT_PCM_16:
        case SF_FORMAT_VORBIS:
        case SF_FORMAT_OPUS:
        case SF_FORMAT_MPEG_LAYER_III:
            switch (alsa_format) {
                // For the integer file types, we rely on sndfile's native conversion.
                case SND_PCM_FORMAT_S16:
                case SND_PCM_FORMAT_S32:
                    read_format = alsa_format;
                    break;
                // For float output, we need to scale the data.
                case SND_PCM_FORMAT_FLOAT:
                case SND_PCM_FORMAT_FLOAT64:
                    read_format = SND_PCM_FORMAT_S16;
                    scale = SCALE_S16;
                    break;
                default:
                    return std::string("Unsupported ALSA format for subformat PCM16-like");
            }
            break;
        case SF_FORMAT_PCM_24:
        case SF_FORMAT_PCM_32:
            switch (alsa_format) {
                case SND_PCM_FORMAT_S16:
                case SND_PCM_FORMAT_S32:
                    read_format = alsa_format;
                    break;
                case SND_PCM_FORMAT_FLOAT:
                case SND_PCM_FORMAT_FLOAT64:
                    read_format = SND_PCM_FORMAT_S32;
                    scale = SCALE_S32;
                    break;
                default:
                    return std::string("Unsupported ALSA format for subformat PCM24/32-like");
            }
            break;
        case SF_FORMAT_FLOAT:
            switch (alsa_format) {
                case SND_PCM_FORMAT_S16:
                    read_format = SND_PCM_FORMAT_FLOAT;
                    scale = SCALE_S16;
                    break;
                case SND_PCM_FORMAT_S32:
                    read_format = SND_PCM_FORMAT_FLOAT;
                    scale = SCALE_S32;
                    break;
                case SND_PCM_FORMAT_FLOAT:
                case SND_PCM_FORMAT_FLOAT64:
                    read_format = alsa_format;
                    break;
                default:
                    return std::string("Unsupported ALSA format for float file");
            }
            break;
        case SF_FORMAT_DOUBLE:
            switch (alsa_format) {
                case SND_PCM_FORMAT_S16:
                    read_format = SND_PCM_FORMAT_FLOAT64;
                    scale = SCALE_S16;
                    break;
                case SND_PCM_FORMAT_S32:
                    read_format = SND_PCM_FORMAT_FLOAT64;
                    scale = SCALE_S32;
                    break;
                case SND_PCM_FORMAT_FLOAT:
                case SND_PCM_FORMAT_FLOAT64:
                    read_format = alsa_format;
                    break;
                default:
                    return std::string("Unsupported ALSA format for double file");
            }
            break;
        default:
            return std::string("Unsupported file subformat");
    }

    // Disable normalization since we are handling scaling ourselves.
    sf_command (sndfile, SFC_SET_NORM_FLOAT, NULL, SF_FALSE) ;
    sf_command (sndfile, SFC_SET_NORM_DOUBLE, NULL, SF_FALSE) ;

    int samples = std::min(BUFFER_LEN / sample_size_from_format(read_format), BUFFER_LEN / sample_size_from_format(alsa_format));
    printf("Starting ALSA playback: read_format=%s, alsa_format=%s, scale=%f, samples=%d\n",
        format_to_string(read_format).c_str(), format_to_string(alsa_format).c_str(), scale, samples) ;

    do {
        readcount = sfg_read(sndfile, read_format, r_buffer, samples);
        //printf("Read %d samples from sound file\n", readcount);
        if (readcount < 0) {
            return std::string("Error reading from sound file");
        }
        int scale_result = scale_data(readcount, read_format, alsa_format, r_buffer, w_buffer, scale);
        if (scale_result != 0) {
            return std::string("Error scaling data for playback");
        }
        [[maybe_unused]] auto count = alsa_write(readcount, alsa_dev, w_buffer, sfinfo.channels, alsa_format) ;
        //printf("Wrote %d frames to ALSA\n", count) ;

        // Check if shutdown has been requested
        if (shutdown_flag && shutdown_flag->load()) {
            break;
        }
    } while (readcount != 0);

    if (shutdown_flag && shutdown_flag->load()) {
        return std::string("Playback interrupted by shutdown signal");
    }
    return std::nullopt;
} /* alsa_play */

