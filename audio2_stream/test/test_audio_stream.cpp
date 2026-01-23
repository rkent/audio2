#include <gtest/gtest.h>
#include "audio2_stream/AudioStream.hpp"
#include "audio2_stream/buffer_file.hpp"
#include <memory>
#include <vector>
#include <thread>
#include <chrono>
#include <fstream>

/**
 * Test fixture for AudioStream unit tests.
 */
class AudioStreamTest : public ::testing::Test
{
protected:
    void SetUp() override {
        // Initialize ROS node for testing
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    void TearDown() override {
        // Clean up any test files
        std::remove(test_audio_file_.c_str());
    }

    // Helper to create a simple test audio file
    void createTestAudioFile(const std::string& filename, int samplerate = 48000, int channels = 2) {
        SndfileHandle file(filename.c_str(), SFM_WRITE, SF_FORMAT_WAV | SF_FORMAT_PCM_16, channels, samplerate);
        
        // Write 1 second of sine wave test audio
        std::vector<short> samples(samplerate * channels);
        for (int i = 0; i < samplerate; i++) {
            short value = static_cast<short>(32767 * 0.5 * std::sin(2.0 * M_PI * 440.0 * i / samplerate));
            for (int c = 0; c < channels; c++) {
                samples[i * channels + c] = value;
            }
        }
        
        file.write(samples.data(), samples.size());
    }

    const std::string test_audio_file_ = "/tmp/test_audio.wav";
};

/**
 * Test basic AudioStream construction and destruction.
 */
TEST_F(AudioStreamTest, ConstructDestruct)
{
    auto stream = std::make_unique<AudioStream>(
        SFG_RW_FORMAT,
        nullptr,  // No source
        nullptr,  // No sink
        "test_stream"
    );
    
    EXPECT_FALSE(stream->shutdown_flag_.load());
    EXPECT_FALSE(stream->data_available_.load());
    EXPECT_EQ(stream->description_, "test_stream");
}

/**
 * Test AudioStream queue operations.
 */
TEST_F(AudioStreamTest, QueueOperations)
{
    auto stream = std::make_unique<AudioStream>(
        SFG_RW_FORMAT,
        nullptr,
        nullptr,
        "test_stream",
        100  // 100 frames
    );
    
    // Create test audio data
    std::vector<uint8_t> test_data(100, 0x42);
    
    // Push to queue
    EXPECT_TRUE(stream->queue_.push(test_data));
    EXPECT_EQ(stream->queue_.read_available(), 1u);
    EXPECT_LT(stream->queue_.write_available(), AUDIO_QUEUE_SIZE);
    
    // Pop from queue
    std::vector<uint8_t> retrieved_data;
    EXPECT_TRUE(stream->queue_.pop(retrieved_data));
    EXPECT_EQ(retrieved_data.size(), test_data.size());
    EXPECT_EQ(retrieved_data, test_data);
    EXPECT_EQ(stream->queue_.read_available(), 0u);
}

/**
 * Test AudioStream queue full behavior.
 */
TEST_F(AudioStreamTest, QueueFull)
{
    auto stream = std::make_unique<AudioStream>(
        SFG_RW_FORMAT,
        nullptr,
        nullptr,
        "test_stream"
    );
    
    std::vector<uint8_t> test_data(100, 0x42);
    
    // Fill the queue
    size_t pushed = 0;
    while (stream->queue_.push(test_data)) {
        pushed++;
    }
    
    EXPECT_GT(pushed, 0u);
    EXPECT_EQ(stream->queue_.write_available(), 0u);
    
    // Try to push one more - should fail
    EXPECT_FALSE(stream->queue_.push(test_data));
}

/**
 * Test AudioStream queue empty behavior.
 */
TEST_F(AudioStreamTest, QueueEmpty)
{
    auto stream = std::make_unique<AudioStream>(
        SFG_RW_FORMAT,
        nullptr,
        nullptr,
        "test_stream"
    );
    
    std::vector<uint8_t> retrieved_data;
    
    // Pop from empty queue should fail
    EXPECT_FALSE(stream->queue_.pop(retrieved_data));
    EXPECT_EQ(stream->queue_.read_available(), 0u);
}

/**
 * Test AudioStream shutdown mechanism.
 */
TEST_F(AudioStreamTest, Shutdown)
{
    auto stream = std::make_unique<AudioStream>(
        SFG_RW_FORMAT,
        nullptr,
        nullptr,
        "test_stream"
    );
    
    EXPECT_FALSE(stream->shutdown_flag_.load());
    
    stream->shutdown();
    
    EXPECT_TRUE(stream->shutdown_flag_.load());
    EXPECT_TRUE(stream->data_available_.load());
    EXPECT_TRUE(stream->shutdown_complete_.load());
}

/**
 * Test SndFileSource opens and reads audio file.
 */
TEST_F(AudioStreamTest, SndFileSourceOpen)
{
    createTestAudioFile(test_audio_file_);
    
    SndFileSource source(test_audio_file_);
    auto result = source.open();
    
    EXPECT_FALSE(result.has_value()) << "Open should succeed";
    EXPECT_TRUE(source.sndfileh_);
    EXPECT_EQ(source.sndfileh_.samplerate(), 48000);
    EXPECT_EQ(source.sndfileh_.channels(), 2);
}

/**
 * Test SndFileSource fails to open non-existent file.
 */
TEST_F(AudioStreamTest, SndFileSourceOpenFailure)
{
    SndFileSource source("/nonexistent/file.wav");
    auto result = source.open();
    
    EXPECT_TRUE(result.has_value()) << "Open should fail";
}

/**
 * Test SndFileSource reads and pushes data to queue.
 */
TEST_F(AudioStreamTest, SndFileSourceRead)
{
    createTestAudioFile(test_audio_file_);
    
    auto source = std::make_unique<SndFileSource>(test_audio_file_);
    ASSERT_FALSE(source->open().has_value());
    
    auto stream = std::make_unique<AudioStream>(
        SFG_RW_FORMAT,
        std::move(source),
        nullptr,  // No sink
        "test_stream",
        100  // Small frames for quick test
    );
    
    stream->start();
    
    // Wait for some data to be read
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Should have pushed data to queue
    EXPECT_GT(stream->queue_.read_available(), 0u);
    
    stream->shutdown();
}

/**
 * Test AudioStream with both source and sink (null terminals for simplicity).
 */
TEST_F(AudioStreamTest, SourceAndSink)
{
    createTestAudioFile(test_audio_file_);
    
    auto source = std::make_unique<SndFileSource>(test_audio_file_);
    ASSERT_FALSE(source->open().has_value());
    
    // For this test, we'll just verify the stream can be created and started
    // without a real sink (since we need mocked ALSA for a real sink)
    auto stream = std::make_unique<AudioStream>(
        SFG_RW_FORMAT,
        std::move(source),
        nullptr,
        "test_stream",
        100
    );
    
    stream->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    stream->shutdown();
    
    // Test passes if no crash
    SUCCEED();
}

/**
 * Test format conversion helper functions.
 */
TEST_F(AudioStreamTest, FormatConversion)
{
    // Test sample size calculation
    EXPECT_EQ(sample_size_from_sfg_format(SFG_SHORT), 2);
    EXPECT_EQ(sample_size_from_sfg_format(SFG_INT), 4);
    EXPECT_EQ(sample_size_from_sfg_format(SFG_FLOAT), 4);
    EXPECT_EQ(sample_size_from_sfg_format(SFG_DOUBLE), 8);
}

/**
 * Test buffer vector creation for format conversion.
 */
TEST_F(AudioStreamTest, ConvertBufferCreation)
{
    std::vector<uint8_t> read_buffer;
    std::vector<uint8_t> write_buffer;
    
    int samples = 1000;
    create_convert_vectors(SFG_SHORT, SFG_FLOAT, samples, read_buffer, write_buffer);
    
    EXPECT_EQ(read_buffer.size(), samples * sizeof(short));
    EXPECT_EQ(write_buffer.size(), samples * sizeof(float));
}

/**
 * Test actual type conversion between formats.
 */
TEST_F(AudioStreamTest, TypeConversion)
{
    std::vector<uint8_t> read_buffer;
    std::vector<uint8_t> write_buffer;
    
    int samples = 100;
    create_convert_vectors(SFG_SHORT, SFG_FLOAT, samples, read_buffer, write_buffer);
    
    // Fill read buffer with test data (16-bit shorts)
    short* short_data = reinterpret_cast<short*>(read_buffer.data());
    for (int i = 0; i < samples; i++) {
        short_data[i] = static_cast<short>(i * 100);
    }
    
    // Convert
    int converted = convert_types(SFG_SHORT, SFG_FLOAT, read_buffer.data(), write_buffer.data(), samples);
    
    EXPECT_EQ(converted, samples);
    
    // Verify conversion
    float* float_data = reinterpret_cast<float*>(write_buffer.data());
    for (int i = 0; i < samples; i++) {
        float expected = static_cast<float>(short_data[i]) / 32768.0f;
        EXPECT_NEAR(float_data[i], expected, 0.0001f);
    }
}

/**
 * Test thread safety with concurrent queue access.
 */
TEST_F(AudioStreamTest, ConcurrentQueueAccess)
{
    auto stream = std::make_unique<AudioStream>(
        SFG_RW_FORMAT,
        nullptr,
        nullptr,
        "test_stream"
    );
    
    std::atomic<bool> done{false};
    std::atomic<int> push_count{0};
    std::atomic<int> pop_count{0};
    
    // Producer thread
    std::thread producer([&]() {
        std::vector<uint8_t> data(100, 0x42);
        for (int i = 0; i < 50; i++) {
            if (stream->queue_.push(data)) {
                push_count++;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    });
    
    // Consumer thread
    std::thread consumer([&]() {
        std::vector<uint8_t> data;
        for (int i = 0; i < 50; i++) {
            if (stream->queue_.pop(data)) {
                pop_count++;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(150));
        }
    });
    
    producer.join();
    consumer.join();
    
    EXPECT_GT(push_count.load(), 0);
    EXPECT_GT(pop_count.load(), 0);
    // Note: pop_count may be less than push_count due to timing
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
