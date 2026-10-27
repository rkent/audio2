#ifndef AUDIO2_STREAM_CONFIG_HPP
#define AUDIO2_STREAM_CONFIG_HPP

#include <sndfile.hh>

// Sample formats supported by the audio device layer (see audioops.hpp)
typedef enum
{
  AUDIO_FORMAT_UNKNOWN = -1,
  AUDIO_FORMAT_S8,
  AUDIO_FORMAT_S16,
  AUDIO_FORMAT_S32,
  AUDIO_FORMAT_FLOAT,
  AUDIO_FORMAT_FLOAT64
} AudioFormat;

// Direction of an audio device stream
typedef enum
{
  AUDIO_PLAYBACK,
  AUDIO_CAPTURE
} AudioDirection;

// These are set in order of preference so sorting gives the lowest as preferable.
typedef enum
{
  SFG_FLOAT,
  SFG_SHORT,
  SFG_INT,
  SFG_BYTE,
  SFG_DOUBLE,
  SFG_NONE,
  SFG_INVALID
} SfgRwFormat;

// Default read/write format for audio streams
const SfgRwFormat SFG_RW_FORMAT = SFG_FLOAT;

// Preferred rate if multiple compatible rates are found between source and sink
const int PREFERRED_RATE = 24000;
// Extra buffer size (on bytes) to allocate for sound file wrapper headers
const int MAX_HEADER = 128;

// Number of frames per audio queue chunk
const int STREAM_QUEUE_FRAMES = 480;

// Audio device (RtAudio) configuration defaults
const int ALSA_PERIOD_SIZE = 1024;
const int ALSA_BUFFER_PERIODS = 4;
const long unsigned int ALSA_BUFFER_SIZE = ALSA_PERIOD_SIZE * ALSA_BUFFER_PERIODS;
// "default" selects the system default device. Any other name selects the RtAudio device
// whose name matches exactly, or else contains it. "null" discards playback and captures
// silence without touching any hardware.
#define ALSA_DEVICE_NAME "default"
const AudioFormat ALSA_FORMAT = AUDIO_FORMAT_FLOAT;
const int ALSA_CHANNELS = 2;
const int ALSA_SAMPLERATE = 48000;
const int ALSA_START_PERIODS = ALSA_BUFFER_PERIODS / 2;
// Playback starts once this many frames are buffered (or on drain).
const long unsigned int ALSA_START_THRESHOLD = ALSA_PERIOD_SIZE * ALSA_START_PERIODS;

// Size of audio queue for buffering between source and sink
const int AUDIO_QUEUE_SIZE = 100;

// Default sound file format for audio streams in ROS2 messages
const int SF_FORMAT_DEFAULT = (SF_FORMAT_WAV | SF_FORMAT_FLOAT);

// QOS settings for audio chunk message publishers and subscribers
const int AUDIO_CHUNK_QOS = 100;

#define PIPER_DATA_DIR "piper"

#endif // AUDIO2_STREAM_CONFIG_HPP
