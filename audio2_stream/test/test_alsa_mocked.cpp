#include <gtest/gtest.h>
#include "audio2_stream/AudioStream.hpp"
#include "audio2_stream/IAlsaDevice.hpp"
#include "audio2_stream/AlsaDeviceImpl.hpp"
#include <memory>
#include <vector>
#include <thread>
#include <chrono>

/**
 * Test fixture for ALSA-related tests using the real ALSA 'null' device.
 * The 'null' device is perfect for testing - it accepts all writes and returns silence on reads.
 */
class AlsaNullTest : public ::testing::Test
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

    // ALSA 'null' device name for testing
  static constexpr const char * ALSA_NULL_DEVICE = "null";
};

/**
 * Test that AlsaSink can be created with the 'null' device and opens successfully.
 */
TEST_F(AlsaNullTest, AlsaSinkOpenSuccess)
{
    auto alsa_device = std::make_unique<AlsaDeviceImpl>();

    AlsaSink sink(ALSA_NULL_DEVICE, 2, 48000, SND_PCM_FORMAT_S16, std::move(alsa_device));

    auto result = sink.open(SND_PCM_STREAM_PLAYBACK);

    EXPECT_FALSE(result.has_value()) << "Open should succeed: " <<
    (result.has_value() ? *result : "");
}

/**
 * Test that AlsaSink handles open failure correctly (using non-existent device).
 */
TEST_F(AlsaNullTest, AlsaSinkOpenFailure)
{
    auto alsa_device = std::make_unique<AlsaDeviceImpl>();

    // Use a non-existent device name
    AlsaSink sink("nonexistent_alsa_device_12345", 2, 48000, SND_PCM_FORMAT_S16,
    std::move(alsa_device));

    auto result = sink.open(SND_PCM_STREAM_PLAYBACK);

    ASSERT_TRUE(result.has_value()) << "Open should fail with non-existent device";
    EXPECT_FALSE(result->empty()) << "Error message should not be empty";
}

/**
 * Test that AlsaSink writes audio data correctly to the 'null' device.
 */
TEST_F(AlsaNullTest, AlsaSinkWriteAudioData)
{
    auto alsa_device = std::make_unique<AlsaDeviceImpl>();
    auto sink = std::make_unique<AlsaSink>(ALSA_NULL_DEVICE, 2, 48000, SND_PCM_FORMAT_S16,
    std::move(alsa_device));

    auto result = sink->open(SND_PCM_STREAM_PLAYBACK);
    ASSERT_FALSE(result.has_value()) << "Open should succeed: " <<
    (result.has_value() ? *result : "");

    // Create a simple audio stream with test data
    auto stream = std::make_unique<AudioStream>(
        SFG_RW_FORMAT,
        nullptr,  // No source
        std::move(sink),
        "test_stream",
        100  // Small queue frames for testing
    );

    // Push some test audio data to the queue
    std::vector<uint8_t> test_data(100 * 2 * 2, 0x55);  // 100 frames, 2 channels, 2 bytes per sample
    stream->queue_.push(test_data);
    stream->data_available_.store(true);
    stream->data_available_.notify_one();

    // Start the stream
    stream->start();

    // Let it run briefly to allow writes to the 'null' device
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Shutdown
    stream->shutdown();

    // Test passes if no crash occurs - 'null' device accepts all writes
    SUCCEED();
}

/**
 * Test that AlsaSource can read audio data from the 'null' device.
 */
TEST_F(AlsaNullTest, AlsaSourceReadAudioData)
{
    auto alsa_device = std::make_unique<AlsaDeviceImpl>();
    auto source = std::make_unique<AlsaSource>(ALSA_NULL_DEVICE, 2, 48000, SND_PCM_FORMAT_S16,
    std::move(alsa_device));

    auto result = source->open(SND_PCM_STREAM_CAPTURE);
    ASSERT_FALSE(result.has_value()) << "Open should succeed: " <<
    (result.has_value() ? *result : "");

    // Create a simple audio stream
    auto stream = std::make_unique<AudioStream>(
        SFG_RW_FORMAT,
        std::move(source),
        nullptr,  // No sink
        "test_stream",
        100  // Small queue frames for testing
    );

    // Start the stream
    stream->start();

    // Let it run briefly - 'null' device returns silence
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Shutdown
    stream->shutdown();

    // Verify that data was pushed to the queue from the 'null' device
    EXPECT_GT(stream->queue_.read_available(),
    0u) << "Queue should have data from 'null' device reads";
}

/**
 * Test a complete source-to-sink path using 'null' devices for both.
 */
TEST_F(AlsaNullTest, SourceToSinkComplete)
{
    // Create source (capture from 'null')
    auto source_device = std::make_unique<AlsaDeviceImpl>();
    auto source = std::make_unique<AlsaSource>(ALSA_NULL_DEVICE, 2, 48000, SND_PCM_FORMAT_S16,
    std::move(source_device));
    auto source_result = source->open(SND_PCM_STREAM_CAPTURE);
    ASSERT_FALSE(source_result.has_value()) << "Source open should succeed";

    // Create sink (playback to 'null')
    auto sink_device = std::make_unique<AlsaDeviceImpl>();
    auto sink = std::make_unique<AlsaSink>(ALSA_NULL_DEVICE, 2, 48000, SND_PCM_FORMAT_S16,
    std::move(sink_device));
    auto sink_result = sink->open(SND_PCM_STREAM_PLAYBACK);
    ASSERT_FALSE(sink_result.has_value()) << "Sink open should succeed";

    // Create stream with both source and sink
    auto stream = std::make_unique<AudioStream>(
        SFG_RW_FORMAT,
        std::move(source),
        std::move(sink),
        "test_stream",
        100
    );

    stream->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    stream->shutdown();

    // Test passes if no crash occurs - data flows from source 'null' to sink 'null'
    SUCCEED();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
