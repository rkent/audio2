#ifndef AUDIO2_STREAM_CONFIG_HPP
#define AUDIO2_STREAM_CONFIG_HPP

#include <limits>
#include <alsa/asoundlib.h>
#include <sndfile.hh>

typedef enum {
    SFG_INVALID = -1,
    SFG_BYTE,
    SFG_SHORT,
    SFG_INT,
    SFG_FLOAT,
    SFG_DOUBLE
} SfgRwFormat;

// Default read/write format for audio streams
const SfgRwFormat SFG_RW_FORMAT = SFG_FLOAT;

// Extra buffer size (on bytes) to allocate for sound file wrapper headers
const int MAX_HEADER = 128;

// Number of frames per audio queue chunk
const int STREAM_QUEUE_FRAMES = 480;

// ALSA configuration defaults
const int ALSA_PERIOD_SIZE = 1024;
const int ALSA_BUFFER_PERIODS = 4;
const long unsigned int ALSA_BUFFER_SIZE = ALSA_PERIOD_SIZE * ALSA_BUFFER_PERIODS;
#define ALSA_DEVICE_NAME "default"
const snd_pcm_format_t ALSA_FORMAT = SND_PCM_FORMAT_FLOAT;
const int ALSA_CHANNELS = 2;
const int ALSA_SAMPLERATE = 48000;
const int ALSA_START_PERIODS = ALSA_BUFFER_PERIODS / 2;
const long unsigned int ALSA_START_THRESHOLD =  ALSA_PERIOD_SIZE * ALSA_START_PERIODS;
// Never automatically stop the alsa device
const long unsigned int ALSA_STOP_THRESHOLD = std::numeric_limits<snd_pcm_uframes_t>::max();
// settings for "samples are zeroed after they are played back"
const long unsigned int ALSA_SILENCE_SIZE = std::numeric_limits<snd_pcm_uframes_t>::max();
const long unsigned int ALSA_SILENCE_THRESHOLD = 0;

// Size of audio queue for buffering between source and sink
const int AUDIO_QUEUE_SIZE = 100;

// Default sound file format for audio streams in ROS2 messages
const int SF_FORMAT_DEFAULT = (SF_FORMAT_WAV | SF_FORMAT_FLOAT);

// QOS settings for audio chunk message publishers and subscribers
const int AUDIO_CHUNK_QOS = 100;

#endif // AUDIO2_STREAM_CONFIG_HPP