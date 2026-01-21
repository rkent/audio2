#include "audio2_stream/buffer_file.hpp"

#include <algorithm>
#include <sndfile.hh>
#include <cstring>
#include <ios>
#include <fstream>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"

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

std::optional<std::string> open_sndfile_from_buffer(VIO_SOUNDFILE_HANDLE & vio_sndfileh, int mode,
    int format, int channels, int samplerate)
{
    vio_sndfileh.fileh = SndfileHandle(vio, &vio_sndfileh.vio_data, mode, format, channels, samplerate);
    if (vio_sndfileh.fileh.error()) {
        return vio_sndfileh.fileh.strError();
    }
    return std::nullopt;
}

// Open a virtual sound file for reading from a vector buffer.
std::optional<std::string> ropen_vio_from_vector(
    const std::vector<unsigned char> & vector,
    VIO_SOUNDFILE_HANDLE & vio_sndfileh
)
{
    // TODO: Standardize types to avoid monstrosities like this
    vio_sndfileh.vio_data.data = const_cast<char *>(reinterpret_cast<const char *>(vector.data()));
    vio_sndfileh.vio_data.length = static_cast<sf_count_t>(vector.size());
    vio_sndfileh.vio_data.offset = 0;
    vio_sndfileh.vio_data.capacity = static_cast<sf_count_t>(vector.capacity());
    vio_sndfileh.fileh = SndfileHandle(
        vio,
        &vio_sndfileh.vio_data,
        SFM_READ
    );
    if (vio_sndfileh.fileh.error()) {
        return std::string("Failed to open buffer as sound file: ") + vio_sndfileh.fileh.strError();
    }
    return std::nullopt;
}

std::optional<std::string> wopen_vio_to_vector(
    std::vector<unsigned char> & vector,
    VIO_SOUNDFILE_HANDLE & vio_sndfileh,
    SfgRwFormat sfg_format,
    int sf_format,
    int channels,
    int samplerate,
    int frames
)
{
    auto bytes_per_chunk = frames * channels * sample_size_from_sfg_format(sfg_format);
    size_t file_size = bytes_per_chunk + MAX_HEADER;
    vector.reserve(file_size);
    vector.clear();

    vio_sndfileh.vio_data.data = reinterpret_cast<char *>(vector.data());
    vio_sndfileh.vio_data.length = 0;
    vio_sndfileh.vio_data.offset = 0;
    vio_sndfileh.vio_data.capacity = static_cast<sf_count_t>(vector.capacity());
    vio_sndfileh.fileh = SndfileHandle(
        vio,
        &vio_sndfileh.vio_data,
        SFM_WRITE,
        sf_format,
        channels,
        samplerate
    );
    if (vio_sndfileh.fileh.error()) {
        return std::string("Failed to open buffer as sound file for writing: ") + vio_sndfileh.fileh.strError();
    }
    return std::nullopt;
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
        int written = sfg_write(fileh.rawHandle(), to_format, byte_buffer, samples_converted);
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
int sfg_write(SNDFILE * sndfile, SfgRwFormat format, void * buffer, int samples)
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
                printf("sfg_write: writing %d float samples RMS %f\n", samples, rms);
            }
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

int sfg_read(SndfileHandle& sndfileh, SfgRwFormat format, void * buffer, int samples)
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
                printf("sfg_read: RMS value of float samples: %f count %d\n", rms, read_samples);
            }
            return read_samples;
        case SFG_DOUBLE:
            return sndfileh.read(reinterpret_cast<double*>(buffer), samples);
        default:
            return -1; // Unsupported format
    }
    return -2; // Should not reach here
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

// Play audio from a SNDFILE using ALSA
std::optional<std::string>
alsa_play (SndfileHandle fileh, snd_pcm_t* alsa_dev, snd_pcm_format_t alsa_format, std::atomic<bool>* shutdown_flag)
{
    auto sndfile = fileh.rawHandle();
    int format = fileh.format();
    int channels = fileh.channels();

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

    while ((readcount = sfg_read(fileh, read_sfg_format, r_buffer, samples)) > 0) {
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
