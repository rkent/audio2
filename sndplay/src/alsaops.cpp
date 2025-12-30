#ifndef _ALSAOPS_CPP__
#define _ALSAOPS_CPP__

#include <cstring>

#include "sndplay/alsaops.hpp"
#include "rclcpp/rclcpp.hpp"

#define	BUFFER_LEN			(8192)

// Scale factors for converting float to integer formats
float SCALE_S8 = 1LL<<7;
float SCALE_S16 = 1LL<<15;
float SCALE_S20 = 1LL<<19;
float SCALE_S24 = 1LL<<23;
float SCALE_S32 = 1LL<<31;

#define _S(cstr) std::string(cstr)
// We allow either a std::string or a const char* to be passed in as estr
#define ECALL(func, estr, ...) \
if ((err = func(__VA_ARGS__)) < 0) {\
    auto _estr = std::string(estr) ; \
    RCLCPP_ERROR(rcl_logger, "%s: %s", _estr.c_str(), snd_strerror (err)) ; \
    goto catch_error ; \
} ;

static auto rcl_logger = rclcpp::get_logger("sndplay.alsaops") ;

template<typename T>
class AlsaWrite {
public:
    int operator()(snd_pcm_t* alsa_dev, T* data, int frames, int channels)
    {	static	int epipe_count = 0 ;

        int total = 0 ;
        int retval ;

        if (epipe_count > 0)
            epipe_count -- ;

        // debug calc rms
        float rms = 0.0f ;
        for (int i = 0; i < frames * channels; ++i) {
            rms += (float)data[i] * (float)data[i];
        }
        rms = sqrtf(rms / (frames * channels));
        // printf("AlsaWrite: frames=%d, channels=%d, rms=%f\n", frames, channels, rms) ;

        while (total < frames)
        {	retval = snd_pcm_writei (alsa_dev, data + total * channels, frames - total) ;

            if (retval >= 0)
            {	total += retval ;
                if (total == frames)
                    return total ;

                continue ;
                } ;

            switch (retval)
            {	case -EAGAIN :
                        puts ("alsa_write: EAGAIN") ;
                        continue ;
                        break ;

                case -EPIPE :
                        if (epipe_count > 0)
                        {	printf ("alsa_write: EPIPE %d\n", epipe_count) ;
                            if (epipe_count > 140)
                                return retval ;
                            } ;
                        epipe_count += 100 ;

    #if 0
                        if (0)
                        {	snd_pcm_status_t *status ;

                            snd_pcm_status_alloca (&status) ;
                            if ((retval = snd_pcm_status (alsa_dev, status)) < 0)
                                fprintf (stderr, "alsa_out: xrun. can't determine length\n") ;
                            else if (snd_pcm_status_get_state (status) == SND_PCM_STATE_XRUN)
                            {	struct timeval now, diff, tstamp ;

                                gettimeofday (&now, 0) ;
                                snd_pcm_status_get_trigger_tstamp (status, &tstamp) ;
                                timersub (&now, &tstamp, &diff) ;

                                fprintf (stderr, "alsa_write xrun: of at least %.3f msecs. resetting stream\n",
                                        diff.tv_sec * 1000 + diff.tv_usec / 1000.0) ;
                                }
                            else
                                fprintf (stderr, "alsa_write: xrun. can't determine length\n") ;
                            } ;
    #endif

                        snd_pcm_prepare (alsa_dev) ;
                        break ;

                case -EBADFD :
                        fprintf (stderr, "alsa_write: Bad PCM state.n") ;
                        return 0 ;
                        break ;

    #if defined ESTRPIPE && ESTRPIPE != EPIPE
                case -ESTRPIPE :
                        fprintf (stderr, "alsa_write: Suspend event.n") ;
                        return 0 ;
                        break ;
    #endif

                case -EIO :
                        puts ("alsa_write: EIO") ;
                        return 0 ;

                default :
                        fprintf (stderr, "alsa_write: retval = %d\n", retval) ;
                        return 0 ;
                        break ;
                } ; /* switch */
            } ; /* while */

        return total ;
    }
} ; /* AlsaWrite */

AlsaWrite<float> alsa_write_float ;
AlsaWrite<short> alsa_write_short ;
AlsaWrite<int> alsa_write_int ;
AlsaWrite<double> alsa_write_double ;
int alsa_write(int readcount, snd_pcm_t* alsa_dev, void* data, int channels, snd_pcm_format_t alsa_format)
{
    if (alsa_format == SND_PCM_FORMAT_S16) {
        int frames = readcount / channels ;
        return alsa_write_short(alsa_dev, reinterpret_cast<short*>(data), frames, channels) ;
    } else if (alsa_format == SND_PCM_FORMAT_S32) {
        int frames = readcount / channels ;
        return alsa_write_int(alsa_dev, reinterpret_cast<int*>(data), frames, channels) ;
    } else if (alsa_format == SND_PCM_FORMAT_FLOAT) {
        int frames = readcount / channels ;
        return alsa_write_float(alsa_dev, reinterpret_cast<float*>(data), frames, channels) ;
    } else if (alsa_format == SND_PCM_FORMAT_FLOAT64) {
        int frames = readcount / channels ;
        return alsa_write_double(alsa_dev, reinterpret_cast<double*>(data), frames, channels) ;
    } else {
        return -1;
    }   
}

int sfg_read2(SNDFILE *sndfile, void * r_buffer, int read_type, int buffer_len)
{
    switch (read_type) {
        case SND_PCM_FORMAT_S16: {
            size_t slength = buffer_len / sizeof(short);
            return sf_read_short(sndfile, reinterpret_cast<short*>(r_buffer), slength);
        }
        case SND_PCM_FORMAT_S32: {
            size_t ilength = buffer_len / sizeof(int);
            return sf_read_int(sndfile, reinterpret_cast<int*>(r_buffer), ilength);
        }
        case SND_PCM_FORMAT_FLOAT: {
            size_t flength = buffer_len / sizeof(float);
            return sf_read_float(sndfile, reinterpret_cast<float*>(r_buffer), flength);
        }
        case SND_PCM_FORMAT_FLOAT64: {
            size_t dlength = buffer_len / sizeof(double);
            return sf_read_double(sndfile, reinterpret_cast<double*>(r_buffer), dlength);
        }
    }
    return -1;
}

snd_pcm_t *
alsa_open (AlsaHwParams hw_vals, AlsaSwParams sw_vals)
{	
    const char * device = hw_vals.device ;
    unsigned	samplerate = hw_vals.samplerate ;
    int		channels = hw_vals.channels ;
    snd_pcm_t *alsa_dev = NULL ;
    snd_pcm_hw_params_t *hw_params ;
    snd_pcm_uframes_t alsa_period_size, alsa_buffer_frames ;
    snd_pcm_sw_params_t *sw_params ;

    int err ;

    alsa_period_size = hw_vals.period_size ;
    alsa_buffer_frames = hw_vals.buffer_size ;

    ECALL(snd_pcm_open, _S("cannot open audio device ") + _S(device), &alsa_dev, device, SND_PCM_STREAM_PLAYBACK, 0) ;
    snd_pcm_hw_params_alloca (&hw_params);

    ECALL(snd_pcm_hw_params_any, "cannot initialize hardware parameter structure", alsa_dev, hw_params) ;
    ECALL(snd_pcm_hw_params_set_access, "cannot set access type", alsa_dev, hw_params, hw_vals.access) ;
    ECALL(snd_pcm_hw_params_set_format, "cannot set sample format", alsa_dev, hw_params, hw_vals.format) ;
    ECALL(snd_pcm_hw_params_set_rate_near, "cannot set sample rate", alsa_dev, hw_params, &samplerate, 0) ;
    ECALL(snd_pcm_hw_params_set_channels, "cannot set channel count", alsa_dev, hw_params, channels) ;
    ECALL(snd_pcm_hw_params_set_buffer_size_near, "cannot set buffer size", alsa_dev, hw_params, &alsa_buffer_frames) ;
    ECALL(snd_pcm_hw_params_set_period_size_near, "cannot set period size", alsa_dev, hw_params, &alsa_period_size, 0) ;
    ECALL(snd_pcm_hw_params, "cannot install hw params", alsa_dev, hw_params) ;

    snd_pcm_sw_params_alloca(&sw_params) ;
    ECALL(snd_pcm_sw_params_current, "snd_pcm_sw_params_current", alsa_dev, sw_params) ;
    
    /* note: set start threshold to delay start until the ring buffer is full */
    ECALL(snd_pcm_sw_params_set_start_threshold, "cannot set start threshold", alsa_dev, sw_params, sw_vals.start_threshold) ;
    ECALL(snd_pcm_sw_params_set_stop_threshold, "cannot set stop threshold", alsa_dev, sw_params, sw_vals.stop_threshold) ;

    // I don't reallt understand silence threshold and silence size. Recommendations are to set silence_threshold to 0
    // and silence_size to boundary. But the boundary is a very large integer, which makes no sense to me.
    // But I do as I am told. Theoretically this should help with xruns.
    ECALL(snd_pcm_sw_params_set_silence_threshold, "cannot set silence threshold", alsa_dev, sw_params, 0) ;
    snd_pcm_uframes_t boundary;
    ECALL(snd_pcm_sw_params_get_boundary, "cannot get boundary", sw_params, &boundary) ;
    ECALL(snd_pcm_sw_params_set_silence_size, "cannot set silence size", alsa_dev, sw_params, boundary) ;
    ECALL(snd_pcm_sw_params, "cannot install sw params", alsa_dev, sw_params) ;

    snd_pcm_reset (alsa_dev) ;

catch_error :

    if (err < 0 && alsa_dev != NULL)
    {	snd_pcm_close (alsa_dev) ;
        return NULL ;
        } ;

    return alsa_dev ;
} /* alsa_open */


int
scale_data(int readcount, int read_type, snd_pcm_format_t alsa_format, void* r_buffer, void* w_buffer, float scale)
{
    // TODO: return r_buffer directly if same data type
    // printf("Scaling data: readcount=%d, read_type=%x, alsa_format=%x, scale=%f\n", readcount, read_type, alsa_format, scale) ;
    for (int i = 0; i < readcount; ++i) {
        if (read_type == SND_PCM_FORMAT_S32) {
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
        } else if (read_type == SND_PCM_FORMAT_S16) {
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
        } else if (read_type == SND_PCM_FORMAT_FLOAT) {
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
        } else if (read_type == SND_PCM_FORMAT_FLOAT64) {
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

// Play audio from a SNDFILE using ALSA
std::optional<std::string> 
alsa_play (SNDFILE *sndfile, SF_INFO sfinfo, snd_pcm_t* alsa_dev, snd_pcm_format_t alsa_format, std::atomic<bool>* shutdown_flag)
{
    static char r_buffer [BUFFER_LEN] ; // read buffer
    static char w_buffer [BUFFER_LEN] ; // write buffer

    int	readcount, read_type ;

    switch (alsa_format) {
        case SND_PCM_FORMAT_S16:
        case SND_PCM_FORMAT_S24:
        case SND_PCM_FORMAT_S32:
        case SND_PCM_FORMAT_FLOAT:
        case SND_PCM_FORMAT_FLOAT64:
            break;
        default:
            return std::string("Unsupported ALSA format: ") + snd_pcm_format_name(alsa_format);
    }

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
                    read_type = alsa_format;
                    break;
                // For float output, we need to scale the data.
                case SND_PCM_FORMAT_FLOAT:
                case SND_PCM_FORMAT_FLOAT64:
                    read_type = SND_PCM_FORMAT_S16;
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
                    read_type = alsa_format;
                    break;
                case SND_PCM_FORMAT_FLOAT:
                case SND_PCM_FORMAT_FLOAT64:
                    read_type = SF_FORMAT_PCM_32;
                    scale = SCALE_S32;
                    break;
                default:
                    return std::string("Unsupported ALSA format for subformat PCM24/32-like");
            }
            break;
        case SF_FORMAT_FLOAT:
            switch (alsa_format) {
                case SND_PCM_FORMAT_S16:
                    read_type = SND_PCM_FORMAT_FLOAT;
                    scale = SCALE_S16;
                    break;
                case SND_PCM_FORMAT_S32:
                    read_type = SND_PCM_FORMAT_FLOAT;
                    scale = SCALE_S32;
                    break;
                case SND_PCM_FORMAT_FLOAT:
                case SND_PCM_FORMAT_FLOAT64:
                    read_type = alsa_format;
                    break;
                default:
                    return std::string("Unsupported ALSA format for float file");
            }
            break;
        case SF_FORMAT_DOUBLE:
            switch (alsa_format) {
                case SND_PCM_FORMAT_S16:
                    read_type = SND_PCM_FORMAT_FLOAT64;
                    scale = SCALE_S16;
                    break;
                case SND_PCM_FORMAT_S32:
                    read_type = SND_PCM_FORMAT_FLOAT64;
                    scale = SCALE_S32;
                    break;
                case SND_PCM_FORMAT_FLOAT:
                case SND_PCM_FORMAT_FLOAT64:
                    read_type = alsa_format;
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
    printf("Starting ALSA playback: read_type=%x, alsa_format=%x, scale=%f\n", read_type, alsa_format, scale) ;

    do {
        readcount = sfg_read2(sndfile, r_buffer, read_type, BUFFER_LEN);
        printf("Read %d samples from sound file\n", readcount);
        if (readcount < 0) {
            return std::string("Error reading from sound file");
        }
        int scale_result = scale_data(readcount, read_type, alsa_format, r_buffer, w_buffer, scale);
        if (scale_result != 0) {
            return std::string("Error scaling data for playback");
        }
        auto count = alsa_write(readcount, alsa_dev, w_buffer, sfinfo.channels, alsa_format) ;
        printf("Wrote %d frames to ALSA\n", count) ;

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

#endif /* _ALSAOPS_CPP__ */
