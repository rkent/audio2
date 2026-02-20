#include <gtest/gtest.h>
#include "audio2_stream/AudioStream.hpp"
#include "audio2_stream/IAlsaProxy.hpp"
#include "audio2_stream/AlsaProxyImpl.hpp"
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
    auto alsa_proxy = std::make_unique<AlsaProxyImpl>();
    // TODO: need to set sink channel, samplerate, and format.
    auto sink = std::make_unique<AlsaSink>(ALSA_NULL_DEVICE, std::move(alsa_proxy));
    auto psink = sink.get();  // Store raw pointer for later use in test

    // Create a simple audio stream with test data
    auto stream = std::make_unique<AudioStream>(
        nullptr,  // No source
        std::move(sink),
        "test_stream",
        100  // Small queue frames for testing
    );

    // Start the stream
    printf("Starting stream for AlsaSinkOpenSuccess test\n");
    stream->start();
    printf("Stream started for AlsaSinkOpenSuccess test\n");
    auto result = psink->open(SND_PCM_STREAM_PLAYBACK, stream.get());

    EXPECT_FALSE(result.has_value()) << "Open should succeed: " <<
    (result.has_value() ? *result : "");
    stream->shutdown();
}

/**
 * Test that AlsaSink handles open failure correctly (using non-existent device).
 */
TEST_F(AlsaNullTest, AlsaSinkOpenFailure)
{
    auto alsa_proxy = std::make_unique<AlsaProxyImpl>();

    // Use a non-existent device name
    auto sink = std::make_unique<AlsaSink>("nonexistent_alsa_device_12345", std::move(alsa_proxy));
    auto psink = sink.get();  // Store raw pointer for later use in test
        // Create a simple audio stream with test data
    auto stream = std::make_unique<AudioStream>(
        nullptr,  // No source
        std::move(sink),
        "test_stream",
        100  // Small queue frames for testing
    );

    // Start the stream
    stream->start();

    auto result = psink->open(SND_PCM_STREAM_PLAYBACK, stream.get());

    ASSERT_TRUE(result.has_value()) << "Open should fail with non-existent device";
    EXPECT_FALSE(result->empty()) << "Error message should not be empty";
    stream->shutdown();
}

/**
 * Test that AlsaSink writes audio data correctly to the 'null' device.
 */
TEST_F(AlsaNullTest, AlsaSinkWriteAudioData)
{
    auto alsa_proxy = std::make_unique<AlsaProxyImpl>();
    auto sink = std::make_unique<AlsaSink>(ALSA_NULL_DEVICE, std::move(alsa_proxy));
    auto psink = sink.get();  // Store raw pointer for later use in test

    // Create a simple audio stream with test data
    auto stream = std::make_unique<AudioStream>(
        nullptr,  // No source
        std::move(sink),
        "test_stream",
        100  // Small queue frames for testing
    );

    // Start the stream
    stream->start();
    auto result = psink->open(SND_PCM_STREAM_PLAYBACK, stream.get());
    ASSERT_FALSE(result.has_value()) << "Open should succeed: " <<
    (result.has_value() ? *result : "");

    // Push some test audio data to the queue
    std::vector<uint8_t> test_data(100 * 2 * 2, 0x55);  // 100 frames, 2 channels, 2 bytes per sample
    stream->queue_.push(test_data);
    stream->data_available_.store(true);
    stream->data_available_.notify_one();

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
    auto alsa_proxy = std::make_unique<AlsaProxyImpl>();
    auto source = std::make_unique<AlsaSource>(ALSA_NULL_DEVICE, std::move(alsa_proxy));
    auto psource = source.get();  // Store raw pointer for later use in test

    // Create a simple audio stream
    auto stream = std::make_unique<AudioStream>(
        std::move(source),
        nullptr,  // No sink
        "test_stream",
        100  // Small queue frames for testing
    );

    auto result = psource->open(SND_PCM_STREAM_CAPTURE, stream.get());
    ASSERT_FALSE(result.has_value()) << "Open should succeed: " <<
    (result.has_value() ? *result : "");

    // Start the stream
    stream->start();

    // Let it run briefly - 'null' device returns silence
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Verify that data was pushed to the queue from the 'null' device
    EXPECT_GT(stream->queue_.read_available(),
    0u) << "Queue should have data from 'null' device reads";

    // Shutdown
    stream->shutdown();
}

/**
 * Test a complete source-to-sink path using 'null' devices for both.
 */
TEST_F(AlsaNullTest, SourceToSinkComplete)
{
    // Create source (capture from 'null')
    auto source_proxy = std::make_unique<AlsaProxyImpl>();
    auto source = std::make_unique<AlsaSource>(ALSA_NULL_DEVICE, std::move(source_proxy));
    auto psource = source.get();  // Store raw pointer for later use in test

    // Create sink (playback to 'null')
    auto sink_proxy = std::make_unique<AlsaProxyImpl>();
    auto sink = std::make_unique<AlsaSink>(ALSA_NULL_DEVICE, std::move(sink_proxy));
    auto psink = sink.get();  // Store raw pointer for later use in test

    // Create stream with both source and sink
    auto stream = std::make_unique<AudioStream>(
        std::move(source),
        std::move(sink),
        "test_stream",
        100
    );

    stream->start();
    auto source_result = psource->open(SND_PCM_STREAM_CAPTURE, stream.get());
    ASSERT_FALSE(source_result.has_value()) << "Source open should succeed";
    auto sink_result = psink->open(SND_PCM_STREAM_PLAYBACK, stream.get());
    ASSERT_FALSE(sink_result.has_value()) << "Sink open should succeed";
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
