#include <gtest/gtest.h>
#include "audio2_stream/WasapiProxyImpl.hpp"
#include <vector>

/**
 * Unit tests for the WASAPI audio backend. Tests that need an audio endpoint
 * are skipped on machines without one.
 */

#define REQUIRE_ENDPOINT(direction) \
  if (WasapiProxyImpl::list_device_names(direction).empty()) { \
    GTEST_SKIP() << "No active WASAPI endpoint for this direction"; \
  }

TEST(WasapiFormatTest, SupportedFormats)
{
  auto f32 = wasapi_sample_format(SND_PCM_FORMAT_FLOAT);
  ASSERT_TRUE(f32.has_value());
  EXPECT_TRUE(f32->is_float);
  EXPECT_EQ(f32->container_bits, 32);
  EXPECT_EQ(f32->valid_bits, 32);

  auto f64 = wasapi_sample_format(SND_PCM_FORMAT_FLOAT64);
  ASSERT_TRUE(f64.has_value());
  EXPECT_TRUE(f64->is_float);
  EXPECT_EQ(f64->container_bits, 64);

  auto s16 = wasapi_sample_format(SND_PCM_FORMAT_S16);
  ASSERT_TRUE(s16.has_value());
  EXPECT_FALSE(s16->is_float);
  EXPECT_EQ(s16->container_bits, 16);
  EXPECT_EQ(s16->valid_bits, 16);

  auto s32 = wasapi_sample_format(SND_PCM_FORMAT_S32);
  ASSERT_TRUE(s32.has_value());
  EXPECT_FALSE(s32->is_float);
  EXPECT_EQ(s32->container_bits, 32);

  // ALSA S24 is 24 significant bits in a 32-bit container
  auto s24 = wasapi_sample_format(SND_PCM_FORMAT_S24);
  ASSERT_TRUE(s24.has_value());
  EXPECT_FALSE(s24->is_float);
  EXPECT_EQ(s24->container_bits, 32);
  EXPECT_EQ(s24->valid_bits, 24);
}

TEST(WasapiFormatTest, UnsupportedFormats)
{
  EXPECT_FALSE(wasapi_sample_format(SND_PCM_FORMAT_U8).has_value());
  EXPECT_FALSE(wasapi_sample_format(SND_PCM_FORMAT_UNKNOWN).has_value());
}

TEST(WasapiProxyTest, UnknownDeviceListsAvailable)
{
  WasapiProxyImpl proxy;
  DeviceParams params;
  params.device = "nonexistent_audio_device_12345";
  params.channels = 2;
  params.samplerate = 48000;
  auto result = proxy.open(params, AudioDirection::Playback);
  ASSERT_TRUE(result.has_value());
  EXPECT_NE(result->find("not found"), std::string::npos) << *result;
  EXPECT_NE(result->find("Available playback devices"), std::string::npos) << *result;
  EXPECT_FALSE(proxy.is_open());
}

TEST(WasapiProxyTest, PlaybackSilence)
{
  REQUIRE_ENDPOINT(AudioDirection::Playback);
  WasapiProxyImpl proxy;
  DeviceParams params;
  params.device = "default";
  params.channels = 1;
  params.samplerate = 24000;  // Not a typical mix rate, so this exercises auto-conversion
  params.format = SND_PCM_FORMAT_FLOAT;
  auto result = proxy.open(params, AudioDirection::Playback);
  ASSERT_FALSE(result.has_value()) << *result;
  EXPECT_TRUE(proxy.is_open());
  EXPECT_EQ(params.samplerate, 24000u);
  EXPECT_EQ(proxy.get_format(), SND_PCM_FORMAT_FLOAT);
  EXPECT_EQ(proxy.device_name(), "default");

  // 100 ms of silence
  std::vector<float> silence(2400, 0.0f);
  int written = proxy.write(static_cast<int>(silence.size()), silence.data(), 1,
    SND_PCM_FORMAT_FLOAT);
  EXPECT_EQ(written, static_cast<int>(silence.size())) << proxy.get_error();

  proxy.close();
  EXPECT_FALSE(proxy.is_open());
}

TEST(WasapiProxyTest, PreopenThenOpen)
{
  REQUIRE_ENDPOINT(AudioDirection::Playback);
  WasapiProxyImpl proxy;
  auto preopen_result = proxy.preopen("default", AudioDirection::Playback);
  ASSERT_FALSE(preopen_result.has_value()) << *preopen_result;
  EXPECT_TRUE(proxy.is_open());
  EXPECT_EQ(proxy.device_name(), "default");

  DeviceParams params;
  params.device = "default";
  params.channels = 2;
  params.samplerate = 48000;
  auto result = proxy.open(params, AudioDirection::Playback);
  EXPECT_FALSE(result.has_value()) << *result;
}

TEST(WasapiProxyTest, CaptureSamples)
{
  REQUIRE_ENDPOINT(AudioDirection::Capture);
  WasapiProxyImpl proxy;
  DeviceParams params;
  params.device = "default";
  params.channels = 2;
  params.samplerate = 48000;
  params.format = SND_PCM_FORMAT_FLOAT;
  auto result = proxy.open(params, AudioDirection::Capture);
  ASSERT_FALSE(result.has_value()) << *result;

  // 100 ms of stereo audio, read in two parts to exercise leftover packet handling
  std::vector<float> buffer(4800 * 2, 1.0f);
  int first = proxy.read(1001 * 2, buffer.data(), 2, SND_PCM_FORMAT_FLOAT);
  EXPECT_EQ(first, 1001 * 2) << proxy.get_error();
  int second = proxy.read(3799 * 2, buffer.data() + first, 2, SND_PCM_FORMAT_FLOAT);
  EXPECT_EQ(second, 3799 * 2) << proxy.get_error();

  proxy.close();
}

TEST(WasapiProxyTest, ReadStopsOnShutdownFlag)
{
  REQUIRE_ENDPOINT(AudioDirection::Capture);
  WasapiProxyImpl proxy;
  DeviceParams params;
  params.device = "default";
  params.channels = 1;
  params.samplerate = 16000;
  auto result = proxy.open(params, AudioDirection::Capture);
  ASSERT_FALSE(result.has_value()) << *result;

  std::atomic<bool> shutdown_flag{true};
  std::vector<float> buffer(16000, 0.0f);
  int got = proxy.read(static_cast<int>(buffer.size()), buffer.data(), 1, SND_PCM_FORMAT_FLOAT,
    &shutdown_flag);
  EXPECT_EQ(got, 0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
