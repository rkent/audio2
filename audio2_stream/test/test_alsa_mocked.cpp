#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "audio2_stream/AudioStream.hpp"
#include "audio2_stream/IAlsaDevice.hpp"
#include "MockAlsaDevice.hpp"
#include <memory>
#include <vector>
#include <thread>
#include <chrono>

using ::testing::_;
using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgPointee;
using ::testing::Invoke;

/**
 * Test fixture for ALSA-related tests using mocked ALSA devices.
 */
class AlsaMockedTest : public ::testing::Test
{
protected:
    void SetUp() override {
        // Initialize ROS node for testing
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    void TearDown() override {
        // Shutdown happens when the test process ends
    }

    /**
     * Helper to create a mock ALSA device with default success behavior.
     */
    std::unique_ptr<MockAlsaDevice> createDefaultMockDevice(snd_pcm_format_t format = SND_PCM_FORMAT_S16) {
        auto mock = std::make_unique<MockAlsaDevice>();
        
        // Default: open succeeds
        EXPECT_CALL(*mock, open(_, _, _))
            .WillOnce(Invoke([format](AlsaHwParams& hw, AlsaSwParams&, snd_pcm_stream_t) {
                // Simulate format being set
                hw.format = format;
                return std::nullopt;
            }));
        
        // Default: get_handle returns non-null
        EXPECT_CALL(*mock, get_handle())
            .WillRepeatedly(Return(reinterpret_cast<snd_pcm_t*>(0x1234)));
        
        // Default: get_format returns the format
        EXPECT_CALL(*mock, get_format())
            .WillRepeatedly(Return(format));
        
        // Default: get_error returns empty string
        EXPECT_CALL(*mock, get_error())
            .WillRepeatedly(Return(""));
        
        // Default: close does nothing
        EXPECT_CALL(*mock, close())
            .Times(::testing::AtLeast(0));
        
        return mock;
    }
};

/**
 * Test that AlsaSink can be created with a mock device and opens successfully.
 */
TEST_F(AlsaMockedTest, AlsaSinkOpenSuccess)
{
    auto mock_device = createDefaultMockDevice();
    
    AlsaSink sink("mock_device", 2, 48000, SND_PCM_FORMAT_S16, std::move(mock_device));
    
    auto result = sink.open(SND_PCM_STREAM_PLAYBACK);
    
    EXPECT_FALSE(result.has_value()) << "Open should succeed";
}

/**
 * Test that AlsaSink handles open failure correctly.
 */
TEST_F(AlsaMockedTest, AlsaSinkOpenFailure)
{
    auto mock_device = std::make_unique<MockAlsaDevice>();
    
    EXPECT_CALL(*mock_device, open(_, _, _))
        .WillOnce(Return(std::string("Mock open failed")));
    
    AlsaSink sink("mock_device", 2, 48000, SND_PCM_FORMAT_S16, std::move(mock_device));
    
    auto result = sink.open(SND_PCM_STREAM_PLAYBACK);
    
    ASSERT_TRUE(result.has_value()) << "Open should fail";
    EXPECT_EQ(*result, "Mock open failed");
}

/**
 * Test that AlsaSink writes audio data correctly.
 */
TEST_F(AlsaMockedTest, AlsaSinkWriteAudioData)
{
    auto mock_device = createDefaultMockDevice();
    int write_count = 0;
    
    // Expect write to be called and simulate successful writes
    EXPECT_CALL(*mock_device, write(_, _, _, _, _))
        .WillRepeatedly(Invoke([&write_count](int samples, void*, int, snd_pcm_format_t, std::atomic<bool>*) {
            write_count++;
            return samples; // Simulate all samples written
        }));
    
    auto sink = std::make_unique<AlsaSink>("mock_device", 2, 48000, SND_PCM_FORMAT_S16, std::move(mock_device));
    
    auto result = sink->open(SND_PCM_STREAM_PLAYBACK);
    ASSERT_FALSE(result.has_value()) << "Open should succeed";
    
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
    
    // Let it run briefly
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Shutdown
    stream->shutdown();
    
    // Verify that write was called
    EXPECT_GT(write_count, 0) << "Write should have been called";
}

/**
 * Test that AlsaSource can read audio data.
 */
TEST_F(AlsaMockedTest, AlsaSourceReadAudioData)
{
    auto mock_device = createDefaultMockDevice();
    int read_count = 0;
    
    // Expect read to be called and simulate successful reads
    EXPECT_CALL(*mock_device, read(_, _, _, _, _))
        .WillRepeatedly(Invoke([&read_count](int samples, void* data, int, snd_pcm_format_t, std::atomic<bool>* shutdown) {
            if (shutdown && shutdown->load()) {
                return -1; // Simulate shutdown
            }
            read_count++;
            // Fill with test data
            std::memset(data, 0xAA, samples * 2);  // 2 bytes per S16 sample
            return samples;
        }));
    
    auto source = std::make_unique<AlsaSource>("mock_device", 2, 48000, SND_PCM_FORMAT_S16, std::move(mock_device));
    
    auto result = source->open(SND_PCM_STREAM_CAPTURE);
    ASSERT_FALSE(result.has_value()) << "Open should succeed";
    
    // Create a simple audio stream with sink that discards data
    auto stream = std::make_unique<AudioStream>(
        SFG_RW_FORMAT,
        std::move(source),
        nullptr,  // No sink
        "test_stream",
        100  // Small queue frames for testing
    );
    
    // Start the stream
    stream->start();
    
    // Let it run briefly
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Shutdown
    stream->shutdown();
    
    // Verify that read was called
    EXPECT_GT(read_count, 0) << "Read should have been called";
    
    // Verify that data was pushed to the queue
    EXPECT_GT(stream->queue_.read_available(), 0u) << "Queue should have data";
}

/**
 * Test error handling when write fails.
 */
TEST_F(AlsaMockedTest, AlsaSinkWriteError)
{
    auto mock_device = createDefaultMockDevice();
    
    // Simulate write failure
    EXPECT_CALL(*mock_device, write(_, _, _, _, _))
        .WillOnce(Return(-EIO));  // Simulate I/O error
    
    EXPECT_CALL(*mock_device, get_error())
        .WillRepeatedly(Return("ALSA write failed"));
    
    auto sink = std::make_unique<AlsaSink>("mock_device", 2, 48000, SND_PCM_FORMAT_S16, std::move(mock_device));
    
    auto result = sink->open(SND_PCM_STREAM_PLAYBACK);
    ASSERT_FALSE(result.has_value());
    
    auto stream = std::make_unique<AudioStream>(
        SFG_RW_FORMAT,
        nullptr,
        std::move(sink),
        "test_stream",
        100
    );
    
    // Push test data
    std::vector<uint8_t> test_data(100 * 2 * 2, 0x55);
    stream->queue_.push(test_data);
    stream->data_available_.store(true);
    stream->data_available_.notify_one();
    
    stream->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    stream->shutdown();
    
    // Test passes if no crash occurs
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
