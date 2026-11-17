#include <gtest/gtest.h>
#include "audio2_stream/AudioStream.hpp"
#include "audio2_stream/buffer_file.hpp"
#include <memory>
#include <vector>
#include <thread>
#include <chrono>
#include <filesystem>
#include <fstream>

/**
 * Test fixture for AudioStream unit tests.
 */
class AudioStreamTest : public ::testing::Test
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
        // Clean up any test files
    std::remove(test_audio_file_.c_str());
  }

    // Helper to create a simple test audio file
  void createTestAudioFile(const std::string & filename, int samplerate = 48000, int channels = 2)
  {
    SndfileHandle file(filename.c_str(), SFM_WRITE, SF_FORMAT_WAV | SF_FORMAT_PCM_16, channels,
      samplerate);

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

  const std::string test_audio_file_ =
    (std::filesystem::temp_directory_path() / "test_audio.wav").string();
};

/**
 * Test basic AudioStream construction and destruction.
 */
TEST_F(AudioStreamTest, ConstructDestruct)
{
    auto stream = std::make_unique<AudioStream>(
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
 * Test the read type chosen for each sndfile format.
 */
TEST_F(AudioStreamTest, SndfileFormatMapping)
{
  EXPECT_EQ(sfg_format_from_sndfile_format(SF_FORMAT_WAV | SF_FORMAT_PCM_U8), SFG_BYTE);
  EXPECT_EQ(sfg_format_from_sndfile_format(SF_FORMAT_WAV | SF_FORMAT_PCM_16), SFG_SHORT);
  EXPECT_EQ(sfg_format_from_sndfile_format(SF_FORMAT_WAV | SF_FORMAT_PCM_24), SFG_INT);
  EXPECT_EQ(sfg_format_from_sndfile_format(SF_FORMAT_WAV | SF_FORMAT_FLOAT), SFG_FLOAT);
  EXPECT_EQ(sfg_format_from_sndfile_format(SF_FORMAT_WAV | SF_FORMAT_DOUBLE), SFG_DOUBLE);
  EXPECT_EQ(sfg_format_from_sndfile_format(SF_FORMAT_FLAC | SF_FORMAT_PCM_16), SFG_SHORT);
  EXPECT_EQ(sfg_format_from_sndfile_format(SF_FORMAT_OGG | SF_FORMAT_VORBIS), SFG_INT);
  EXPECT_EQ(sfg_format_from_sndfile_format(SF_FORMAT_OGG | SF_FORMAT_OPUS), SFG_INT);
  EXPECT_EQ(sfg_format_from_sndfile_format(SF_FORMAT_MPEG | SF_FORMAT_MPEG_LAYER_III), SFG_SHORT);
  // Apple Lossless: the narrowest type that holds the stored bit depth
  EXPECT_EQ(sfg_format_from_sndfile_format(SF_FORMAT_CAF | SF_FORMAT_ALAC_16), SFG_SHORT);
  EXPECT_EQ(sfg_format_from_sndfile_format(SF_FORMAT_CAF | SF_FORMAT_ALAC_20), SFG_INT);
  EXPECT_EQ(sfg_format_from_sndfile_format(SF_FORMAT_CAF | SF_FORMAT_ALAC_24), SFG_INT);
  EXPECT_EQ(sfg_format_from_sndfile_format(SF_FORMAT_CAF | SF_FORMAT_ALAC_32), SFG_INT);
}

/**
 * Test decoding an Apple Lossless (ALAC in CAF) file through SndFileSource's path,
 * AudioStream::process_fileh. ALAC cannot be decoded until the file is closed, so it is
 * only usable from files, not as an AudioChunk message format.
 */
TEST_F(AudioStreamTest, SndFileSourceDecodesAppleLossless)
{
  const std::string path =
    (std::filesystem::temp_directory_path() / "test_audio_alac.caf").string();
  const int samplerate = 48000;
  const int channels = 2;
  const int frames = samplerate / 4;  // 0.25 s, since process_fileh paces in real time

  // A different tone per channel, so channel order errors show up
  std::vector<short> original(static_cast<size_t>(frames) * channels);
  for (int f = 0; f < frames; ++f) {
    for (int c = 0; c < channels; ++c) {
      original[static_cast<size_t>(f) * channels + c] = static_cast<short>(
        16384 * std::sin(2.0 * M_PI * 440.0 * (c + 1) * f / samplerate));
    }
  }
  {
    SndfileHandle file(path, SFM_WRITE, SF_FORMAT_CAF | SF_FORMAT_ALAC_16, channels, samplerate);
    ASSERT_FALSE(file.error()) << file.strError();
    ASSERT_EQ(file.writef(original.data(), frames), frames);
  }  // closing the file writes the ALAC packet table

  auto source = std::make_unique<SndFileSource>(path);
  SndFileSource * source_raw = source.get();
  ASSERT_FALSE(source->open().has_value());
  EXPECT_EQ(source_raw->sndfileh_.format() & SF_FORMAT_SUBMASK, SF_FORMAT_ALAC_16);

  const int chunk_frames = 2400;  // fewer chunks than the queue holds, so none are dropped
  AudioStream stream(std::move(source), nullptr, "alac_test", chunk_frames);
  testing::internal::CaptureStdout();
  stream.process_fileh(source_raw->sndfileh_);
  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(output.find("Unsupported sndfile sub-format"), std::string::npos);

  // With no sink, chunks are float
  std::vector<float> decoded;
  std::vector<uint8_t> chunk;
  while (stream.queue_.pop(chunk)) {
    const float * values = reinterpret_cast<const float *>(chunk.data());
    decoded.insert(decoded.end(), values, values + chunk.size() / sizeof(float));
  }
  std::remove(path.c_str());

  ASSERT_EQ(decoded.size(), original.size());
  float max_error = 0.0f;
  for (size_t i = 0; i < original.size(); ++i) {
    // False positive: cppcheck does not know the ASSERT_EQ above returns on a size mismatch
    // cppcheck-suppress containerOutOfBounds
    max_error = std::max(max_error, std::fabs(decoded[i] - original[i] / 32768.0f));
  }
  EXPECT_LE(max_error, 1.0f / 32768);
}

/**
 * Test that a stream into a MessageSink, as used by audio2_capture_node and
 * audio2_play_remote_node, can resolve its samplerate and channel count.
 */
TEST_F(AudioStreamTest, MessageSinkStreamParametersResolve)
{
  createTestAudioFile(test_audio_file_, 48000, 2);
  auto node = std::make_shared<rclcpp::Node>("message_sink_test");
  auto publisher = node->create_publisher<audio2_stream_msgs::msg::AudioChunk>("test_chunks", 10);

  auto source = std::make_unique<SndFileSource>(test_audio_file_);
  ASSERT_FALSE(source->open().has_value());
  auto sink = std::make_unique<MessageSink>("test_chunks", SF_FORMAT_DEFAULT, publisher, "test");
  EXPECT_TRUE(sink->are_parms_bound_);

  AudioStream stream(std::move(source), std::move(sink), "message sink test");
  auto result = stream.fix_parms();
  EXPECT_FALSE(result.has_value()) << *result;
  EXPECT_EQ(stream.samplerate_, 48000);
  EXPECT_EQ(stream.channels_, 2);
}

/**
 * Test buffer vector creation for format conversion.
 */
TEST_F(AudioStreamTest, ConvertBufferCreation)
{
    std::vector<uint8_t> read_buffer;
    std::vector<uint8_t> write_buffer;

    int samples = 1000;
    auto result = create_convert_vectors(SFG_SHORT, SFG_FLOAT, samples, read_buffer, write_buffer);
    ASSERT_FALSE(result.has_value()) << "Failed to create convert vectors: " << result.value();

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
    auto result = create_convert_vectors(SFG_SHORT, SFG_FLOAT, samples, read_buffer, write_buffer);
    ASSERT_FALSE(result.has_value()) << "Failed to create convert vectors: " << result.value();

    // Fill read buffer with test data (16-bit shorts)
    short * short_data = reinterpret_cast<short *>(read_buffer.data());
    for (int i = 0; i < samples; i++) {
    short_data[i] = static_cast<short>(i * 100);
    }

    // Convert
    int converted = convert_types(SFG_SHORT, SFG_FLOAT, read_buffer.data(), write_buffer.data(),
    samples);

    EXPECT_EQ(converted, samples);

    // Verify conversion
    float * float_data = reinterpret_cast<float *>(write_buffer.data());
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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
