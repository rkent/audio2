#include "audio2_stream/buffer_file.hpp"

#include <algorithm>
#include <sndfile.hh>
#include <cstring>
#include <ios>
#include <fstream>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"

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
    //vio_sndfile.sndfile = sf_open_virtual(&vio, mode, &vio_sndfile.sfinfo, &vio_sndfile.vio_data);
    SndfileHandle handle = SndfileHandle(vio, &vio_sndfile.vio_data, mode);
    printf("Opened virtual sound file from buffer with mode %d\n", mode);
    printf("File info - frames: %lld, samplerate: %d, channels: %d, format: %X\n",
        static_cast<long long>(handle.frames()), handle.samplerate(), handle.channels(),
        handle.format());
    if (handle.error()) {
        return handle.strError();
    }
    vio_sndfile.sndfile = handle.rawHandle();
    //if (!vio_sndfile.sndfile) {
    //    return std::string("Failed to open sound file from buffer: ") + sf_strerror(nullptr);
    // }
    return std::nullopt;
}

std::optional<std::string> open_sndfile_from_buffer2(VIO_SOUNDFILE_HANDLE & vio_sndfileh, int mode,
    int format, int channels, int samplerate)
{
    vio_sndfileh.fileh = SndfileHandle(vio, &vio_sndfileh.vio_data, mode, format, channels, samplerate);
    if (vio_sndfileh.fileh.error()) {
        return vio_sndfileh.fileh.strError();
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

// Write samples from a buffer to a SNDFILE with scaling and conversion.
int sfg_write_convert(SndfileHandle & fileh, SfgRwFormat from_format, SfgRwFormat to_format, char * buffer, int samples)
{
    if (samples <= 0) {
        return samples;
    }
    int total = 0;
    const int CONVERT_BUFF_SIZE = 8192;
    char byte_buffer[CONVERT_BUFF_SIZE];
    int convert_samples = std::min(samples, CONVERT_BUFF_SIZE / sample_size_from_sfg_format(to_format));
    int offset = 0;
    while (samples > 0) {
        int to_process = std::min(samples, convert_samples);
        // Convert to target format in byte_buffer
        int samples_converted = convert_types(from_format, to_format, buffer + offset, byte_buffer, to_process);
        if (samples_converted < 0) {
            printf("Error converting types: %d\n", samples_converted);
            return samples_converted; // Conversion error
        }
        // Write converted samples to sndfile
        int written = sfg_write2(fileh.rawHandle(), to_format, byte_buffer, samples_converted);
        if (written < 0) {
            printf("Error writing to sndfile: %d\n", written);
            return written; // Write error
        }
        samples -= samples_converted;
        offset += samples_converted * sample_size_from_sfg_format(from_format);
        total += written;
    }
    return total;
}
int sfg_write2(SNDFILE * sndfile, SfgRwFormat format, void * buffer, int samples)
{
    switch (format) {
        case SFG_SHORT:
            return sf_write_short(sndfile, reinterpret_cast<short*>(buffer), samples);
        case SFG_INT:
            return sf_write_int(sndfile, reinterpret_cast<int*>(buffer), samples);
        case SFG_FLOAT:
            // Calculate and print RMS value of float samples
            {
                float rms = 0.0f;
                float* float_buffer = reinterpret_cast<float*>(buffer);
                for (int i = 0; i < samples; ++i) {
                    rms += float_buffer[i] * float_buffer[i];
                }
                rms = std::sqrt(rms / samples);
                printf("sfg_write2: RMS value of float samples: %f\n", rms);
            }
            printf("sfg_write2: writing %d float samples\n", samples);
            return sf_write_float(sndfile, reinterpret_cast<float*>(buffer), samples);
        case SFG_DOUBLE:
            return sf_write_double(sndfile, reinterpret_cast<double*>(buffer), samples);
        default:
            return -1; // Unsupported format
    }
    return -2; // Should not reach here
}

// Get sample size in bytes from alsa's snd_pcm_format_t
int sample_size_from_alsa_format(snd_pcm_format_t format)
{
    return snd_pcm_format_width(format) / 8;
}

// Get sample size from our SfgRwFormat enum
int sample_size_from_sfg_format(SfgRwFormat format)
{
    switch (format) {
        case SFG_BYTE:
            return 1;
        case SFG_SHORT:
            return 2;
        case SFG_INT:
            return 4;
        case SFG_FLOAT:
            return 4;
        case SFG_DOUBLE:
            return 8;
        default:
            return -1; // Unsupported format
    }
}

// String representation of SfgRwFormat
const char * sfg_format_to_string(SfgRwFormat format)
{
    switch (format) {
        case SFG_BYTE:
            return "BYTE";
        case SFG_SHORT:
            return "SHORT";
        case SFG_INT:
            return "INT";
        case SFG_FLOAT:
            return "FLOAT";
        case SFG_DOUBLE:
            return "DOUBLE";
        default:
            return "UNKNOWN";
    }
}

SfgRwFormat sfg_format_from_alsa_format(snd_pcm_format_t alsa_format)
{
    switch (alsa_format) {
        case SND_PCM_FORMAT_S8:
        case SND_PCM_FORMAT_U8:
            return SFG_BYTE;
        case SND_PCM_FORMAT_S16:
            return SFG_SHORT;
        case SND_PCM_FORMAT_S32:
            return SFG_INT;
        case SND_PCM_FORMAT_FLOAT:
            return SFG_FLOAT;
        case SND_PCM_FORMAT_FLOAT64:
            return SFG_DOUBLE;
        default:
            return SFG_INVALID;
    }
}
// Read/write type to use for different sndfile formats
SfgRwFormat sfg_format_from_sndfile_format(int sf_format)
{
    int sub_format = sf_format & SF_FORMAT_SUBMASK;

    switch (sub_format) {
        case SF_FORMAT_PCM_U8:
        case SF_FORMAT_PCM_S8:
            return SFG_BYTE;
        case SF_FORMAT_PCM_16:
            return SFG_SHORT;
        case SF_FORMAT_PCM_24:
        case SF_FORMAT_PCM_32:
        case SF_FORMAT_VORBIS:
        case SF_FORMAT_OPUS:
            return SFG_INT;
        case SF_FORMAT_FLOAT:
            return SFG_FLOAT;
        case SF_FORMAT_DOUBLE:
            return SFG_DOUBLE;
        default:
            printf("Unsupported sndfile sub-format: %x\n", sub_format);
            return SFG_INT; // Default to INT for unsupported formats
    }
}

int sfg_read(SNDFILE * sndfile, SfgRwFormat format, void * buffer, int samples)
{
    if (samples <= 0) {
        return 0;
    }

    switch (format) {
        case SFG_SHORT:
            return sf_read_short(sndfile, reinterpret_cast<short*>(buffer), samples);
        case SFG_INT:
            return sf_read_int(sndfile, reinterpret_cast<int*>(buffer), samples);
        case SFG_FLOAT:
            return sf_read_float(sndfile, reinterpret_cast<float*>(buffer), samples);
        case SFG_DOUBLE:
            return sf_read_double(sndfile, reinterpret_cast<double*>(buffer), samples);
        default:
            return -1; // Unsupported format
    }
    return -2; // Should not reach here
}

int sfg_read2(SndfileHandle& sndfileh, SfgRwFormat format, void * buffer, int samples)
{
    if (samples <= 0) {
        return 0;
    }
    int read_samples;
    switch (format) {
        case SFG_SHORT:
            return sndfileh.read(reinterpret_cast<short*>(buffer), samples);
        case SFG_INT:
            return sndfileh.read(reinterpret_cast<int*>(buffer), samples);
        case SFG_FLOAT:
            {
                read_samples = sndfileh.read(reinterpret_cast<float*>(buffer), samples);
                if (read_samples <= 0) {
                    return read_samples;
                }
                float* float_buffer = reinterpret_cast<float*>(buffer);
                float rms = 0.0f;
                for (int i = 0; i < read_samples; ++i) {
                    rms += float_buffer[i] * float_buffer[i];
                }
                rms = std::sqrt(rms / read_samples);
                printf("sfg_read2: RMS value of float samples: %f count %d\n", rms, read_samples);
            }
            return read_samples;
        case SFG_DOUBLE:
            return sndfileh.read(reinterpret_cast<double*>(buffer), samples);
        default:
            return -1; // Unsupported format
    }
    return -2; // Should not reach here
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
    int sample_size = sample_size_from_alsa_format(snd_format);
    if (sample_size < 0) {
        RCLCPP_ERROR(rcl_logger, "Unsupported format for writing buffer: %x", sfinfo.format);
        return -1;
    }
    int file_size = sfinfo.frames * sfinfo.channels * sample_size + MAX_HEADER;
    printf("Allocating buffer of size %d bytes for writing\n", file_size);
    std::vector<char> file_data(file_size);
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
std::optional<std::string> get_file(const char * file_path, std::shared_ptr<std::vector<char>> & out_data)
{
    // Read entire file into memory
    std::ifstream file(file_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        return std::string("Cannot open file: ") + file_path;
    }

    std::streamsize file_size = file.tellg();
    file.seekg(0, std::ios::beg);
    out_data = std::make_shared<std::vector<char>>(file_size);
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
        std::shared_ptr<std::vector<char>> file_data = params.file_data;
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
            auto open_result = alsa_open(hw_vals, sw_vals, alsa_dev);
            if (open_result) {
                RCLCPP_ERROR(rcl_logger, "Failed to open ALSA device: %s", open_result->c_str());
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

#define	BUFFER_LEN			(1000*4)

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

void create_convert_vectors(SfgRwFormat r_format, SfgRwFormat w_format, int samples,
    std::vector<uint8_t>& r_buffer, std::vector<uint8_t>& w_buffer)
{
    auto r_sample_size = sample_size_from_sfg_format(r_format);
    auto w_sample_size = sample_size_from_sfg_format(w_format);
    auto r_buffer_size = samples * r_sample_size;
    auto w_buffer_size = samples * w_sample_size;
    r_buffer.resize(r_buffer_size);
    w_buffer.resize(w_buffer_size);
}


// Convert audio data between different SfgRwFormat types.
int convert_types(SfgRwFormat from_format, SfgRwFormat to_format, const void* in_buffer, void* out_buffer, int samples)
{
    const short* in_short = reinterpret_cast<const short*>(in_buffer);
    const int* in_int = reinterpret_cast<const int*>(in_buffer);
    const float* in_float = reinterpret_cast<const float*>(in_buffer);
    const double* in_double = reinterpret_cast<const double*>(in_buffer);

    short* out_short = reinterpret_cast<short*>(out_buffer);
    int* out_int = reinterpret_cast<int*>(out_buffer);
    float* out_float = reinterpret_cast<float*>(out_buffer);
    double* out_double = reinterpret_cast<double*>(out_buffer);

    for (int i = 0; i < samples; ++i) {
        switch(from_format) {
            case SFG_SHORT:
            switch(to_format) {
                case SFG_SHORT:
                out_short[i] = in_short[i];
                break;
                case SFG_INT:
                    //if (!(i%100))
                    //    printf("i = %d\n", i);
                    out_int[i] = static_cast<int>(in_short[i]) << 16;
                    break;
                case SFG_FLOAT:
                    out_float[i] = static_cast<float>(in_short[i]) / SCALE_S16;
                    break;
                case SFG_DOUBLE:
                    out_double[i] = static_cast<double>(in_short[i]) / static_cast<double>(SCALE_S16);
                    break;
                default:
                    return -1; // Unsupported conversion
                }
                break;
            case SFG_INT:
                switch(to_format) {
                    case SFG_SHORT:
                        out_short[i] = static_cast<short>(in_int[i] >> 16);
                        break;
                    case SFG_INT:
                        out_int[i] = in_int[i];
                        break;
                    case SFG_FLOAT:
                        out_float[i] = static_cast<float>(in_int[i]) / SCALE_S32;
                        break;
                    case SFG_DOUBLE:
                        out_double[i] = static_cast<double>(in_int[i]) / static_cast<double>(SCALE_S32);
                        break;
                    default:
                        return -1; // Unsupported conversion
                }
                break;
            case SFG_FLOAT:
                switch(to_format) {
                    case SFG_SHORT:
                        out_short[i] = static_cast<short>(in_float[i] * SCALE_S16);
                        break;
                    case SFG_INT:
                        out_int[i] = static_cast<int>(in_float[i] * SCALE_S32);
                        break;
                    case SFG_FLOAT:
                        out_float[i] = in_float[i];
                        break;
                    case SFG_DOUBLE:
                        out_double[i] = static_cast<double>(in_float[i]);
                        break;
                    default:
                        return -1; // Unsupported conversion
                }
                break;
            case SFG_DOUBLE:
                switch(to_format) {
                    case SFG_SHORT:
                        out_short[i] = static_cast<short>(in_double[i] * static_cast<double>(SCALE_S16));
                        break;
                    case SFG_INT:
                        out_int[i] = static_cast<int>(in_double[i] * static_cast<double>(SCALE_S32));
                        break;
                    case SFG_FLOAT:
                        out_float[i] = static_cast<float>(in_double[i]);
                        break;
                    case SFG_DOUBLE:
                        out_double[i] = in_double[i];
                        break;
                    default:
                        return -1; // Unsupported conversion
                }
                break;
            default:
                return -1; // Unsupported from_format
        }
    }
    return samples;
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

std::optional<std::string>
alsa_play (SndfileHandle fileh, snd_pcm_t* alsa_dev, snd_pcm_format_t alsa_format, std::atomic<bool>* shutdown_flag)
{
    return alsa_play(fileh.rawHandle(), fileh.format(), fileh.channels(), alsa_dev, alsa_format, shutdown_flag);
}

SndFileStream::SndFileStream(SndfileHandle & sndfileh,
    snd_pcm_format_t alsa_format,
    std::atomic<bool>* shutdown_flag,
    std::atomic<bool>* data_available,
    boost::lockfree::spsc_queue<std::vector<uint8_t>>* queue)
    : sndfileh_(sndfileh), alsa_format_(alsa_format),
      shutdown_flag_(shutdown_flag), data_available_(data_available), queue_(queue)
{}

void SndFileStream::run()
{
    const int STREAM_FRAMES = 480;
    auto r_format = sfg_format_from_sndfile_format(sndfileh_.format());
    auto w_format = sfg_format_from_alsa_format(alsa_format_);
    auto r_sample_size = sample_size_from_sfg_format(r_format);
    auto w_sample_size = sample_size_from_sfg_format(w_format);
    auto r_buffer_size = STREAM_FRAMES * sndfileh_.channels() * r_sample_size;
    auto w_buffer_size = STREAM_FRAMES * sndfileh_.channels() * w_sample_size;
    std::vector<uint8_t> r_buffer(r_buffer_size);
    std::vector<uint8_t> w_buffer(w_buffer_size);
    auto duration = std::chrono::microseconds(static_cast<int64_t>(1'000'000.0 * STREAM_FRAMES / (sndfileh_.samplerate())));
    auto next_time = std::chrono::steady_clock::now();
    int silent_frames = ALSA_PERIOD_SIZE * ALSA_BUFFER_PERIODS;

    while (!(shutdown_flag_ && shutdown_flag_->load())) {
        int samples_read = sfg_read2(sndfileh_, r_format, r_buffer.data(), STREAM_FRAMES * sndfileh_.channels());
        if (samples_read <= 0) {
            if (silent_frames > 0) {
                // Push silence
                std::fill(r_buffer.begin(), r_buffer.end(), 0);
                silent_frames -= STREAM_FRAMES;
                samples_read = STREAM_FRAMES * sndfileh_.channels();
                printf("End of file reached, pushing silence (%d frames left)\n", silent_frames);
            } else {
                break; // End of file or error
            }
        }
        int samples_converted = convert_types(r_format, w_format, r_buffer.data(), w_buffer.data(), samples_read);
        if (samples_converted < 0) {
            RCLCPP_ERROR(rcl_logger, "Error converting audio data for streaming: %d", samples_converted);
            break;
        } else if (samples_converted != samples_read) {
            RCLCPP_ERROR(rcl_logger, "Mismatch in converted samples count: expected %d, got %d", samples_read, samples_converted);
            break;
        }

        while (!queue_->push(w_buffer) && !shutdown_flag_->load()) {
            RCLCPP_WARN(rcl_logger, "Audio queue is full, waiting...");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        printf("Pushed %d samples to audio queue\n", samples_converted);
        data_available_->store(true);
        data_available_->notify_one();
        next_time += duration;
        std::this_thread::sleep_until(next_time);
    }
    printf("SndFileStream thread exiting\n");
}

// Play audio from a SNDFILE using ALSA
std::optional<std::string>
alsa_play (SNDFILE *sndfile, int format, int channels, snd_pcm_t* alsa_dev, snd_pcm_format_t alsa_format, std::atomic<bool>* shutdown_flag)
{
    static char r_buffer [BUFFER_LEN] ; // read buffer
    static char w_buffer [BUFFER_LEN] ; // write buffer

    int	readcount;
    auto read_sfg_format = sfg_format_from_sndfile_format(format);

    // Disable normalization since we are handling scaling ourselves.
    sf_command (sndfile, SFC_SET_NORM_FLOAT, NULL, SF_FALSE) ;
    sf_command (sndfile, SFC_SET_NORM_DOUBLE, NULL, SF_FALSE) ;

    int samples = std::min(BUFFER_LEN / sample_size_from_sfg_format(read_sfg_format), BUFFER_LEN / sample_size_from_alsa_format(alsa_format));
    printf("Starting ALSA playback: alsa_format=%s, samples=%d\n",
        format_to_string(alsa_format).c_str(), samples) ;

    while ((readcount = sfg_read(sndfile, read_sfg_format, r_buffer, samples)) > 0) {
        printf("Read %d samples from sound file expecting %d\n", readcount, samples);
        if (readcount < 0) {
            return std::string("Error reading from sound file");
        }
        int samples_converted = convert_types(read_sfg_format, sfg_format_from_alsa_format(alsa_format), r_buffer, w_buffer, readcount);
        if (samples_converted < 0) {
            return std::string("Error converting audio data for playback");
        } else if (samples_converted != readcount) {
            return std::string("Mismatch in converted samples count");
        }
        auto count = alsa_write(readcount, alsa_dev, w_buffer, channels, alsa_format, shutdown_flag) ;
        if (count < 0) {
            return std::string("Error writing to ALSA device");
        } else if (count != readcount) {
            return std::string("Mismatch in ALSA write count: expected ") + std::to_string(readcount) + ", got " + std::to_string(count);
        }
        printf("Wrote %d samples to ALSA\n", count) ;

        // Check if shutdown has been requested
        if (shutdown_flag && shutdown_flag->load()) {
            break;
        }
    }

    if (shutdown_flag && shutdown_flag->load()) {
        return std::string("Playback interrupted by shutdown signal");
    }
    return std::nullopt;
} /* alsa_play */

std::optional<std::string>
alsa_play (SNDFILE *sndfile, SF_INFO sfinfo, snd_pcm_t* alsa_dev, snd_pcm_format_t alsa_format, std::atomic<bool>* shutdown_flag)
{
    return alsa_play(sndfile, sfinfo.format, sfinfo.channels, alsa_dev, alsa_format, shutdown_flag);
}
