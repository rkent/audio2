#include <gtest/gtest.h>
#include "audio2_stream/AudioStream.hpp"
#include "audio2_stream/IAudioDeviceProxy.hpp"
#ifdef _WIN32
#include "audio2_stream/WasapiProxyImpl.hpp"
#endif
#include <memory>
#include <vector>
#include <thread>
#include <chrono>

/**
 * Test fixture for DeviceSink/DeviceSource tests using the platform's audio backend.
 * On Linux this is the ALSA 'null' device, which accepts all writes and returns silence on reads.
 * On Windows it is the default WASAPI endpoint, so tests are skipped on machines without one.
 */
class DeviceTerminalTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
        // Initialize ROS node for testing
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
        // Cleanup happens automatically
  }

#ifdef _WIN32
  static constexpr const char * TEST_DEVICE = "default";
#else
    // ALSA 'null' device name for testing
  static constexpr const char * TEST_DEVICE = "null";
#endif
};

// Skip when the test device does not exist (Windows machines without that kind of endpoint)
#ifdef _WIN32
#define REQUIRE_TEST_DEVICE(direction) \
  if (WasapiProxyImpl::list_device_names(direction).empty()) { \
    GTEST_SKIP() << "No active WASAPI endpoint for this direction"; \
  }
#else
#define REQUIRE_TEST_DEVICE(direction)
#endif

// Shuts the stream down on scope exit, so a failed ASSERT cannot leave stream threads waiting
struct StreamShutdownGuard
{
  AudioStream * stream;
  ~StreamShutdownGuard()
  {
    stream->shutdown();
  }
};

/**
 * Test that DeviceSink can be created with the test device and opens successfully.
 */
TEST_F(DeviceTerminalTest, DeviceSinkOpenSuccess)
{
    REQUIRE_TEST_DEVICE(AudioDirection::Playback);
    // TODO: need to set sink channel, samplerate, and format.
    auto sink = std::make_unique<DeviceSink>(TEST_DEVICE, make_default_device_proxy());
    auto psink = sink.get();  // Store raw pointer for later use in test

    // Create a simple audio stream with test data
    auto stream = std::make_unique<AudioStream>(
        nullptr,  // No source
        std::move(sink),
        "test_stream",
        100  // Small queue frames for testing
    );
    StreamShutdownGuard guard{stream.get()};

    // Start the stream
    printf("Starting stream for DeviceSinkOpenSuccess test\n");
    stream->start();
    printf("Stream started for DeviceSinkOpenSuccess test\n");
    auto result = psink->open(AudioDirection::Playback, stream.get());

    EXPECT_FALSE(result.has_value()) << "Open should succeed: " <<
    (result.has_value() ? *result : "");
}

/**
 * Test that DeviceSink handles open failure correctly (using non-existent device).
 */
TEST_F(DeviceTerminalTest, DeviceSinkOpenFailure)
{
    // Use a non-existent device name
    auto sink = std::make_unique<DeviceSink>("nonexistent_audio_device_12345",
      make_default_device_proxy());
    auto psink = sink.get();  // Store raw pointer for later use in test
        // Create a simple audio stream with test data
    auto stream = std::make_unique<AudioStream>(
        nullptr,  // No source
        std::move(sink),
        "test_stream",
        100  // Small queue frames for testing
    );
    StreamShutdownGuard guard{stream.get()};

    // Start the stream
    stream->start();

    auto result = psink->open(AudioDirection::Playback, stream.get());

    ASSERT_TRUE(result.has_value()) << "Open should fail with non-existent device";
    EXPECT_FALSE(result->empty()) << "Error message should not be empty";
}

/**
 * Test that DeviceSink writes audio data correctly to the test device.
 */
TEST_F(DeviceTerminalTest, DeviceSinkWriteAudioData)
{
    REQUIRE_TEST_DEVICE(AudioDirection::Playback);
    auto sink = std::make_unique<DeviceSink>(TEST_DEVICE, make_default_device_proxy());
    auto psink = sink.get();  // Store raw pointer for later use in test

    // Create a simple audio stream with test data
    auto stream = std::make_unique<AudioStream>(
        nullptr,  // No source
        std::move(sink),
        "test_stream",
        100  // Small queue frames for testing
    );
    StreamShutdownGuard guard{stream.get()};

    // Start the stream
    stream->start();
    auto result = psink->open(AudioDirection::Playback, stream.get());
    ASSERT_FALSE(result.has_value()) << "Open should succeed: " <<
    (result.has_value() ? *result : "");

    // Push some silent test audio data to the queue. Silence, since on Windows this plays
    // through a real device.
    std::vector<uint8_t> test_data(100 * 2 * 2, 0);  // 100 frames, 2 channels, 2 bytes per sample
    stream->queue_.push(test_data);
    stream->data_available_.store(true);
    stream->data_available_.notify_one();

    // Let it run briefly to allow writes to the device
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Test passes if no crash occurs
    SUCCEED();
}

/**
 * Test that DeviceSource can read audio data from the test device.
 */
TEST_F(DeviceTerminalTest, DeviceSourceReadAudioData)
{
    REQUIRE_TEST_DEVICE(AudioDirection::Capture);
    auto source = std::make_unique<DeviceSource>(TEST_DEVICE, make_default_device_proxy());
    auto psource = source.get();  // Store raw pointer for later use in test

    // Create a simple audio stream
    auto stream = std::make_unique<AudioStream>(
        std::move(source),
        nullptr,  // No sink
        "test_stream",
        100  // Small queue frames for testing
    );
    StreamShutdownGuard guard{stream.get()};

    auto result = psource->open(AudioDirection::Capture, stream.get());
    ASSERT_FALSE(result.has_value()) << "Open should succeed: " <<
    (result.has_value() ? *result : "");

    // Start the stream
    stream->start();

    // Let it run briefly
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Verify that data was pushed to the queue from the device
    EXPECT_GT(stream->queue_.read_available(),
    0u) << "Queue should have data from device reads";
}

/**
 * Test a complete source-to-sink path using the test device for both.
 */
TEST_F(DeviceTerminalTest, SourceToSinkComplete)
{
    REQUIRE_TEST_DEVICE(AudioDirection::Capture);
    REQUIRE_TEST_DEVICE(AudioDirection::Playback);
    // Create source (capture from the test device)
    auto source = std::make_unique<DeviceSource>(TEST_DEVICE, make_default_device_proxy());
    auto psource = source.get();  // Store raw pointer for later use in test

    // Create sink (playback to the test device)
    auto sink = std::make_unique<DeviceSink>(TEST_DEVICE, make_default_device_proxy());
    auto psink = sink.get();  // Store raw pointer for later use in test

    // Create stream with both source and sink
    auto stream = std::make_unique<AudioStream>(
        std::move(source),
        std::move(sink),
        "test_stream",
        100
    );
    StreamShutdownGuard guard{stream.get()};

    // Open both devices before starting the threads. Once the source pushes audio, the sink
    // thread opens its device itself if it is not open, which would race with an open here.
    auto source_result = psource->open(AudioDirection::Capture, stream.get());
    ASSERT_FALSE(source_result.has_value()) << "Source open should succeed";
    auto sink_result = psink->open(AudioDirection::Playback, stream.get());
    ASSERT_FALSE(sink_result.has_value()) << "Sink open should succeed";
    stream->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Test passes if no crash occurs - data flows from source to sink
    SUCCEED();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
