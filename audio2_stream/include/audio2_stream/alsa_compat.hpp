#ifndef AUDIO2_STREAM_ALSA_COMPAT_HPP
#define AUDIO2_STREAM_ALSA_COMPAT_HPP

// ALSA is Linux-only. On Windows, provide the subset of the ALSA API used by this
// package as stubs so that the device-independent code still builds. Every device
// operation fails with -ENODEV, so opening an audio device reports an error at runtime.

#ifndef _WIN32

#include <alsa/asoundlib.h>

#else

#include <cerrno>
#include <cstdio>

#ifndef EBADFD
#define EBADFD 77
#endif

typedef struct _snd_pcm snd_pcm_t;
typedef struct _snd_pcm_hw_params snd_pcm_hw_params_t;
typedef struct _snd_pcm_sw_params snd_pcm_sw_params_t;
typedef struct _snd_pcm_status snd_pcm_status_t;
typedef struct _snd_output snd_output_t;
typedef unsigned long snd_pcm_uframes_t;
typedef long snd_pcm_sframes_t;

// Enum values match ALSA (little-endian) so integer parameters mean the same on all platforms.
typedef enum _snd_pcm_stream
{
  SND_PCM_STREAM_PLAYBACK = 0,
  SND_PCM_STREAM_CAPTURE = 1
} snd_pcm_stream_t;

typedef enum _snd_pcm_access
{
  SND_PCM_ACCESS_MMAP_INTERLEAVED = 0,
  SND_PCM_ACCESS_MMAP_NONINTERLEAVED = 1,
  SND_PCM_ACCESS_MMAP_COMPLEX = 2,
  SND_PCM_ACCESS_RW_INTERLEAVED = 3,
  SND_PCM_ACCESS_RW_NONINTERLEAVED = 4
} snd_pcm_access_t;

typedef enum _snd_pcm_format
{
  SND_PCM_FORMAT_UNKNOWN = -1,
  SND_PCM_FORMAT_S8 = 0,
  SND_PCM_FORMAT_U8 = 1,
  SND_PCM_FORMAT_S16 = 2,
  SND_PCM_FORMAT_U16 = 4,
  SND_PCM_FORMAT_S24 = 6,
  SND_PCM_FORMAT_U24 = 8,
  SND_PCM_FORMAT_S32 = 10,
  SND_PCM_FORMAT_U32 = 12,
  SND_PCM_FORMAT_FLOAT = 14,
  SND_PCM_FORMAT_FLOAT64 = 16
} snd_pcm_format_t;

typedef enum _snd_pcm_state
{
  SND_PCM_STATE_OPEN = 0,
  SND_PCM_STATE_SETUP = 1,
  SND_PCM_STATE_PREPARED = 2,
  SND_PCM_STATE_RUNNING = 3,
  SND_PCM_STATE_XRUN = 4
} snd_pcm_state_t;

#define snd_pcm_hw_params_alloca(ptr) (*(ptr) = nullptr)
#define snd_pcm_sw_params_alloca(ptr) (*(ptr) = nullptr)
#define snd_pcm_status_alloca(ptr) (*(ptr) = nullptr)

inline const char * snd_strerror(int) {return "ALSA is not available on Windows";}

inline int snd_pcm_format_width(snd_pcm_format_t format)
{
  switch (format) {
    case SND_PCM_FORMAT_S8:
    case SND_PCM_FORMAT_U8:
      return 8;
    case SND_PCM_FORMAT_S16:
    case SND_PCM_FORMAT_U16:
      return 16;
    case SND_PCM_FORMAT_S24:
    case SND_PCM_FORMAT_U24:
    case SND_PCM_FORMAT_S32:
    case SND_PCM_FORMAT_U32:
    case SND_PCM_FORMAT_FLOAT:
      return 32;
    case SND_PCM_FORMAT_FLOAT64:
      return 64;
    default:
      return -EINVAL;
  }
}

inline int snd_pcm_open(snd_pcm_t ** pcm, const char *, snd_pcm_stream_t, int)
{
  *pcm = nullptr;
  return -ENODEV;
}
inline int snd_pcm_close(snd_pcm_t *) {return -ENODEV;}
inline int snd_pcm_drain(snd_pcm_t *) {return -ENODEV;}
inline int snd_pcm_prepare(snd_pcm_t *) {return -ENODEV;}
inline int snd_pcm_reset(snd_pcm_t *) {return -ENODEV;}
inline int snd_pcm_start(snd_pcm_t *) {return -ENODEV;}
inline const char * snd_pcm_name(snd_pcm_t *) {return "none";}
inline snd_pcm_state_t snd_pcm_state(snd_pcm_t *) {return SND_PCM_STATE_OPEN;}
inline snd_pcm_sframes_t snd_pcm_avail_update(snd_pcm_t *) {return -ENODEV;}
inline snd_pcm_sframes_t snd_pcm_writei(snd_pcm_t *, const void *, snd_pcm_uframes_t)
{
  return -ENODEV;
}
inline snd_pcm_sframes_t snd_pcm_readi(snd_pcm_t *, void *, snd_pcm_uframes_t) {return -ENODEV;}

inline int snd_pcm_hw_params(snd_pcm_t *, snd_pcm_hw_params_t *) {return -ENODEV;}
inline int snd_pcm_hw_params_any(snd_pcm_t *, snd_pcm_hw_params_t *) {return -ENODEV;}
inline int snd_pcm_hw_params_current(snd_pcm_t *, snd_pcm_hw_params_t *) {return -ENODEV;}
inline int snd_pcm_hw_params_set_access(snd_pcm_t *, snd_pcm_hw_params_t *, snd_pcm_access_t)
{
  return -ENODEV;
}
inline int snd_pcm_hw_params_set_format(snd_pcm_t *, snd_pcm_hw_params_t *, snd_pcm_format_t)
{
  return -ENODEV;
}
inline int snd_pcm_hw_params_set_channels(snd_pcm_t *, snd_pcm_hw_params_t *, unsigned int)
{
  return -ENODEV;
}
inline int snd_pcm_hw_params_set_rate_near(
  snd_pcm_t *, snd_pcm_hw_params_t *, unsigned int *, int *)
{
  return -ENODEV;
}
inline int snd_pcm_hw_params_set_period_size_near(
  snd_pcm_t *, snd_pcm_hw_params_t *, snd_pcm_uframes_t *, int *)
{
  return -ENODEV;
}
inline int snd_pcm_hw_params_set_buffer_size_near(
  snd_pcm_t *, snd_pcm_hw_params_t *, snd_pcm_uframes_t *)
{
  return -ENODEV;
}
inline int snd_pcm_hw_params_get_rate(const snd_pcm_hw_params_t *, unsigned int *, int *)
{
  return -ENODEV;
}

inline int snd_pcm_sw_params(snd_pcm_t *, snd_pcm_sw_params_t *) {return -ENODEV;}
inline int snd_pcm_sw_params_current(snd_pcm_t *, snd_pcm_sw_params_t *) {return -ENODEV;}
inline int snd_pcm_sw_params_set_start_threshold(
  snd_pcm_t *, snd_pcm_sw_params_t *, snd_pcm_uframes_t)
{
  return -ENODEV;
}
inline int snd_pcm_sw_params_set_stop_threshold(
  snd_pcm_t *, snd_pcm_sw_params_t *, snd_pcm_uframes_t)
{
  return -ENODEV;
}
inline int snd_pcm_sw_params_set_silence_size(
  snd_pcm_t *, snd_pcm_sw_params_t *, snd_pcm_uframes_t)
{
  return -ENODEV;
}
inline int snd_pcm_sw_params_set_silence_threshold(
  snd_pcm_t *, snd_pcm_sw_params_t *, snd_pcm_uframes_t)
{
  return -ENODEV;
}

inline int snd_pcm_status(snd_pcm_t *, snd_pcm_status_t *) {return -ENODEV;}
inline int snd_output_stdio_attach(snd_output_t ** output, FILE *, int)
{
  *output = nullptr;
  return -ENODEV;
}
inline int snd_output_close(snd_output_t *) {return 0;}
inline int snd_pcm_hw_params_dump(snd_pcm_hw_params_t *, snd_output_t *) {return -ENODEV;}
inline int snd_pcm_status_dump(snd_pcm_status_t *, snd_output_t *) {return -ENODEV;}

#endif  // _WIN32

#endif  // AUDIO2_STREAM_ALSA_COMPAT_HPP
