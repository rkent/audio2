// Round trip of audio through the in-memory sound file encoding used for AudioChunk messages.
// encode() and decode() follow MessageSink::run and MessageSource::callback in AudioStream.cpp.

#include <gtest/gtest.h>
#include "audio2_stream/buffer_file.hpp"
#include "audio2_stream/config.hpp"

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

namespace
{

struct CodecCase
{
  std::string name;
  int sf_format;
  int channels;
  int samplerate;
  int frames;
  float tolerance;  // largest allowed per-sample error; 0 means exact
};

// A sine wave per channel, at different frequencies so channel order errors show up
std::vector<float> make_samples(int channels, int samplerate, int frames)
{
  std::vector<float> samples(static_cast<size_t>(frames) * channels);
  for (int f = 0; f < frames; ++f) {
    for (int c = 0; c < channels; ++c) {
      double freq = 440.0 * (c + 1);
      samples[static_cast<size_t>(f) * channels + c] =
        static_cast<float>(0.5 * std::sin(2.0 * M_PI * freq * f / samplerate));
    }
  }
  return samples;
}

// As MessageSink: write float samples to an in-memory sound file, and take the bytes written
// so far as the message data (the writer is still open, as when the message is published).
std::vector<unsigned char> encode(const CodecCase & tc, std::vector<float> samples)
{
  std::vector<unsigned char> file_buffer;
  VIO_SOUNDFILE_HANDLE vio;
  auto err = wopen_vio_to_vector(file_buffer, vio, SFG_FLOAT, tc.sf_format, tc.channels,
    tc.samplerate, tc.frames);
  EXPECT_FALSE(err.has_value()) << *err;
  if (err) {
    return {};
  }
  int count = static_cast<int>(samples.size());
  int written = sfg_write_convert(vio.fileh, SFG_FLOAT, SFG_FLOAT,
    reinterpret_cast<char *>(samples.data()), count);
  EXPECT_EQ(written, count);
  return std::vector<unsigned char>(file_buffer.data(), file_buffer.data() + vio.vio_data.length);
}

struct Decoded
{
  int channels = 0;
  int samplerate = 0;
  std::vector<float> samples;
};

// As MessageSource: open the message data as a sound file, read it in its own sample type,
// and convert to float
Decoded decode(const std::vector<unsigned char> & data, int chunk_frames)
{
  Decoded out;
  VIO_SOUNDFILE_HANDLE vio;
  auto err = ropen_vio_from_vector(data, vio);
  EXPECT_FALSE(err.has_value()) << *err;
  if (err) {
    return out;
  }
  out.channels = vio.fileh.channels();
  out.samplerate = vio.fileh.samplerate();

  auto r_format = sfg_format_from_sndfile_format(vio.fileh.format());
  std::vector<uint8_t> r_buffer;
  std::vector<uint8_t> w_buffer;
  auto result = create_convert_vectors(r_format, SFG_FLOAT, chunk_frames * out.channels,
    r_buffer, w_buffer);
  EXPECT_FALSE(result.has_value()) << *result;
  if (result) {
    return out;
  }
  while (true) {
    int samples_read = sfg_read(vio.fileh, r_format, r_buffer.data(), chunk_frames * out.channels);
    if (samples_read <= 0) {
      EXPECT_EQ(samples_read, 0) << "read error";
      break;
    }
    int converted = convert_types(r_format, SFG_FLOAT, r_buffer.data(), w_buffer.data(),
      samples_read);
    EXPECT_EQ(converted, samples_read);
    if (converted <= 0) {
      break;
    }
    const float * values = reinterpret_cast<const float *>(w_buffer.data());
    out.samples.insert(out.samples.end(), values, values + converted);
  }
  return out;
}

class AudioChunkCodec : public ::testing::TestWithParam<CodecCase> {};

}  // namespace

TEST_P(AudioChunkCodec, RoundTripPreservesSamples)
{
  const CodecCase & tc = GetParam();
  const auto original = make_samples(tc.channels, tc.samplerate, tc.frames);

  const auto data = encode(tc, original);
  ASSERT_FALSE(data.empty());
  const auto decoded = decode(data, tc.frames);

  EXPECT_EQ(decoded.channels, tc.channels);
  EXPECT_EQ(decoded.samplerate, tc.samplerate);
  ASSERT_EQ(decoded.samples.size(), original.size());
  float max_error = 0.0f;
  for (size_t i = 0; i < original.size(); ++i) {
    max_error = std::max(max_error, std::fabs(decoded.samples[i] - original[i]));
  }
  if (tc.tolerance == 0.0f) {
    EXPECT_EQ(std::memcmp(decoded.samples.data(), original.data(),
      original.size() * sizeof(float)), 0) << "max error " << max_error;
  } else {
    EXPECT_LE(max_error, tc.tolerance);
  }
}

INSTANTIATE_TEST_SUITE_P(
  Formats, AudioChunkCodec,
  ::testing::Values(
    // The format the nodes publish (SF_FORMAT_DEFAULT), at the default chunk size
    CodecCase{"DefaultStereo", SF_FORMAT_DEFAULT, 2, 48000, STREAM_QUEUE_FRAMES, 0.0f},
    CodecCase{"DefaultMono", SF_FORMAT_DEFAULT, 1, 24000, STREAM_QUEUE_FRAMES, 0.0f},
    CodecCase{"DefaultLargeChunk", SF_FORMAT_DEFAULT, 2, 44100, 4800, 0.0f},
    CodecCase{"WavFloat64", SF_FORMAT_WAV | SF_FORMAT_DOUBLE, 2, 48000, STREAM_QUEUE_FRAMES, 0.0f},
    // Integer PCM: within one quantization step
    CodecCase{"WavPcm16", SF_FORMAT_WAV | SF_FORMAT_PCM_16, 2, 48000, STREAM_QUEUE_FRAMES,
      1.0f / 32768},
    CodecCase{"WavPcm32", SF_FORMAT_WAV | SF_FORMAT_PCM_32, 2, 48000, STREAM_QUEUE_FRAMES, 1e-6f}),
  [](const ::testing::TestParamInfo<CodecCase> & info) {return info.param.name;});
