#ifndef _ALSAOPS_CPP__
#define _ALSAOPS_CPP__

#include <cstring>

#include "audio2_stream/alsaops.hpp"
#include "rclcpp/rclcpp.hpp"

#define _S(cstr) std::string(cstr)
// We allow either a std::string or a const char* to be passed in as estr
#define ECALL(func, estr, ...) \
if ((err = func(__VA_ARGS__)) < 0) {\
    auto _estr = std::string(estr) ; \
    RCLCPP_ERROR(rcl_logger, "%s: %s", _estr.c_str(), snd_strerror (err)) ; \
    goto catch_error ; \
} ;

static auto rcl_logger = rclcpp::get_logger("audio2_stream.alsaops") ;

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
        // float rms = 0.0f ;
        // for (int i = 0; i < frames * channels; ++i) {
        //     rms += (float)data[i] * (float)data[i];
        // }
        // rms = sqrtf(rms / (frames * channels));
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
int alsa_write(int samples, snd_pcm_t* alsa_dev, void* data, int channels, snd_pcm_format_t alsa_format)
{
    int frames = samples / channels ;
    if (alsa_format == SND_PCM_FORMAT_S16) {
        return alsa_write_short(alsa_dev, reinterpret_cast<short*>(data), frames, channels) ;
    } else if (alsa_format == SND_PCM_FORMAT_S32) {
        return alsa_write_int(alsa_dev, reinterpret_cast<int*>(data), frames, channels) ;
    } else if (alsa_format == SND_PCM_FORMAT_FLOAT) {
        return alsa_write_float(alsa_dev, reinterpret_cast<float*>(data), frames, channels) ;
    } else if (alsa_format == SND_PCM_FORMAT_FLOAT64) {
        return alsa_write_double(alsa_dev, reinterpret_cast<double*>(data), frames, channels) ;
    } else {
        return -1;
    }   
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



#endif /* _ALSAOPS_CPP__ */
