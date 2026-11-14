#ifndef AUDIO2_STREAM_SAMPLE_FORMAT_HPP
#define AUDIO2_STREAM_SAMPLE_FORMAT_HPP

/**
 * Sample format of audio exchanged with an audio device, independent of the backend.
 *
 * The integer values match ALSA's little-endian snd_pcm_format_t values (enforced in
 * alsaops.hpp), since they are used as the nodes' integer "sample_format" parameter.
 * Do not renumber them.
 */
enum class SampleFormat : int
{
  Unknown = -1,
  S8 = 0,
  U8 = 1,
  S16 = 2,
  U16 = 4,
  S24 = 6,   // 24 significant bits in a 32-bit container
  U24 = 8,   // 24 significant bits in a 32-bit container
  S32 = 10,
  U32 = 12,
  Float = 14,
  Float64 = 16
};

/**
 * Size in bytes of one sample in memory (the container size, so 4 for S24/U24).
 * \return The size, or -1 for an unknown format.
 */
inline int sample_format_bytes(SampleFormat format)
{
  switch (format) {
    case SampleFormat::S8:
    case SampleFormat::U8:
      return 1;
    case SampleFormat::S16:
    case SampleFormat::U16:
      return 2;
    case SampleFormat::S24:
    case SampleFormat::U24:
    case SampleFormat::S32:
    case SampleFormat::U32:
    case SampleFormat::Float:
      return 4;
    case SampleFormat::Float64:
      return 8;
    default:
      return -1;
  }
}

#endif  // AUDIO2_STREAM_SAMPLE_FORMAT_HPP
