#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
#include <vector>

#include "audio2_stream/AudioStream.hpp"
#include "audio2_stream/stream_policy.hpp"

using SH = audio2_stream_msgs::msg::StreamHeader;

TEST(DecideHandle, NoOtherStreamsAlwaysStarts)
{
  for (uint8_t handle : {SH::HANDLE_MIX, SH::HANDLE_MUTE, SH::HANDLE_QUEUE, SH::HANDLE_CANCEL,
      SH::HANDLE_IGNORE})
  {
    auto decision = decide_handle(handle, false);
    EXPECT_EQ(decision.admission, Admission::StartNow) << "handle " << int(handle);
    EXPECT_FALSE(decision.cancel_others) << "handle " << int(handle);
    EXPECT_FALSE(decision.unknown_handle) << "handle " << int(handle);
  }
}

TEST(DecideHandle, WithOtherStreams)
{
  EXPECT_EQ(decide_handle(SH::HANDLE_MIX, true).admission, Admission::StartNow);
  EXPECT_EQ(decide_handle(SH::HANDLE_MUTE, true).admission, Admission::StartNow);
  EXPECT_EQ(decide_handle(SH::HANDLE_QUEUE, true).admission, Admission::Enqueue);
  EXPECT_EQ(decide_handle(SH::HANDLE_IGNORE, true).admission, Admission::Drop);

  auto cancel = decide_handle(SH::HANDLE_CANCEL, true);
  EXPECT_EQ(cancel.admission, Admission::StartNow);
  EXPECT_TRUE(cancel.cancel_others);

  for (uint8_t handle : {SH::HANDLE_MIX, SH::HANDLE_MUTE, SH::HANDLE_QUEUE, SH::HANDLE_IGNORE}) {
    EXPECT_FALSE(decide_handle(handle, true).cancel_others) << "handle " << int(handle);
  }
}

TEST(DecideHandle, DefaultIsMix)
{
  EXPECT_EQ(SH().handle, SH::HANDLE_MIX);
}

TEST(DecideHandle, UnknownHandleMixes)
{
  auto decision = decide_handle(200, true);
  EXPECT_EQ(decision.admission, Admission::StartNow);
  EXPECT_FALSE(decision.cancel_others);
  EXPECT_TRUE(decision.unknown_handle);
}

namespace
{

std::unique_ptr<AudioStream> make_stream(uint8_t handle)
{
  auto stream = std::make_unique<AudioStream>(nullptr, nullptr, "policy test");
  stream->handle_ = handle;
  return stream;
}

}  // namespace

TEST(UpdateMutes, MuteSilencesOlderStreams)
{
  auto mix1 = make_stream(SH::HANDLE_MIX);
  auto mix2 = make_stream(SH::HANDLE_MIX);
  auto mute = make_stream(SH::HANDLE_MUTE);
  update_mutes({mix1.get(), mix2.get(), mute.get()});
  EXPECT_TRUE(mix1->muted_.load());
  EXPECT_TRUE(mix2->muted_.load());
  EXPECT_FALSE(mute->muted_.load());
}

TEST(UpdateMutes, NewerStreamIsNotMuted)
{
  auto older = make_stream(SH::HANDLE_MIX);
  auto mute = make_stream(SH::HANDLE_MUTE);
  auto newer = make_stream(SH::HANDLE_MIX);
  update_mutes({older.get(), mute.get(), newer.get()});
  EXPECT_TRUE(older->muted_.load());
  EXPECT_FALSE(mute->muted_.load());
  EXPECT_FALSE(newer->muted_.load());
}

TEST(UpdateMutes, NewerMuteSilencesOlderMute)
{
  auto mute1 = make_stream(SH::HANDLE_MUTE);
  auto mute2 = make_stream(SH::HANDLE_MUTE);
  update_mutes({mute1.get(), mute2.get()});
  EXPECT_TRUE(mute1->muted_.load());
  EXPECT_FALSE(mute2->muted_.load());
}

TEST(UpdateMutes, UnmuteWhenMuteStreamStops)
{
  auto mix = make_stream(SH::HANDLE_MIX);
  auto mute = make_stream(SH::HANDLE_MUTE);
  update_mutes({mix.get(), mute.get()});
  ASSERT_TRUE(mix->muted_.load());

  // Stopping (e.g. cancelled) no longer mutes
  mute->shutdown_flag_.store(true);
  update_mutes({mix.get(), mute.get()});
  EXPECT_FALSE(mix->muted_.load());

  // Finished and removed from the list
  mute->shutdown_flag_.store(false);
  update_mutes({mix.get(), mute.get()});
  ASSERT_TRUE(mix->muted_.load());
  update_mutes({mix.get()});
  EXPECT_FALSE(mix->muted_.load());
}

TEST(RequestShutdown, WithoutThreadsCompletesImmediately)
{
  auto stream = make_stream(SH::HANDLE_MIX);
  stream->request_shutdown();
  EXPECT_TRUE(stream->shutdown_flag_.load());
  EXPECT_TRUE(stream->shutdown_complete_.load());
}

namespace
{

audio2_stream_msgs::msg::AudioChunk::SharedPtr make_eos_chunk()
{
  auto chunk = std::make_shared<audio2_stream_msgs::msg::AudioChunk>();
  chunk->header.sequence = SH::SEQUENCE_EOS;
  return chunk;
}

}  // namespace

// End of stream queues the end marker for the sink instead of shutting down in the callback
TEST(MessageSourceEos, PushesEndMarker)
{
  auto stream = make_stream(SH::HANDLE_MIX);
  MessageSource source("test");
  source.callback(make_eos_chunk(), stream.get());

  EXPECT_FALSE(stream->shutdown_flag_.load());
  ASSERT_EQ(stream->queue_.read_available(), 1u);
  std::vector<uint8_t> marker{1};
  ASSERT_TRUE(stream->queue_.pop(marker));
  EXPECT_TRUE(marker.empty());
}

// With no room for the end marker, end_of_stream_ is set instead and the queued audio kept
TEST(MessageSourceEos, FullQueueSetsEndOfStream)
{
  auto stream = make_stream(SH::HANDLE_MIX);
  while (stream->queue_.push(std::vector<uint8_t>(4, 0))) {
  }
  auto queued = stream->queue_.read_available();
  MessageSource source("test");
  source.callback(make_eos_chunk(), stream.get());

  EXPECT_TRUE(stream->end_of_stream_.load());
  EXPECT_FALSE(stream->shutdown_flag_.load());
  EXPECT_EQ(stream->queue_.read_available(), queued);
}

namespace
{

// A device that takes 5 ms per write, counting writes of non-silent audio
class PacedProxy : public IAudioDeviceProxy
{
public:
  explicit PacedProxy(std::shared_ptr<std::atomic<int>> audio_writes)
  : audio_writes_(std::move(audio_writes)) {}

  std::optional<std::string> open(DeviceParams &, AudioDirection) override
  {
    open_ = true;
    return std::nullopt;
  }
  void close() override {open_ = false;}
  int write(int samples, void * data, int, std::atomic<bool> *) override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (samples > 0 && static_cast<uint8_t *>(data)[0] != 0) {
      ++*audio_writes_;
    }
    return samples;
  }
  int read(int, void *, int, std::atomic<bool> *) override {return -1;}
  bool is_open() const override {return open_;}
  std::string device_name() const override {return "paced";}
  std::string get_error() const override {return "";}
  SampleFormat get_format() const override {return SampleFormat::Float;}

private:
  std::shared_ptr<std::atomic<int>> audio_writes_;
  bool open_ = false;
};

// A started stream with a paced device sink and no source, with `chunks` chunks of audio queued
std::unique_ptr<AudioStream> make_paced_stream(
  std::shared_ptr<std::atomic<int>> audio_writes, int & chunks)
{
  auto sink = std::make_unique<DeviceSink>("paced", std::make_unique<PacedProxy>(audio_writes));
  auto stream = std::make_unique<AudioStream>(nullptr, std::move(sink), "paced stream", 48);
  chunks = 0;
  while (stream->queue_.push(std::vector<uint8_t>(48 * sizeof(float), 0x11))) {
    ++chunks;
  }
  return stream;
}

bool wait_for_complete(AudioStream & stream, std::chrono::milliseconds timeout)
{
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!stream.shutdown_complete_.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return stream.shutdown_complete_.load();
}

}  // namespace

// A stopped (e.g. cancelled) device sink discards its queued audio instead of playing it out
TEST(DeviceSinkStop, ShutdownDiscardsQueuedAudio)
{
  auto audio_writes = std::make_shared<std::atomic<int>>(0);
  int chunks = 0;
  auto stream = make_paced_stream(audio_writes, chunks);
  ASSERT_GT(chunks, 50);
  ASSERT_FALSE(stream->start().has_value());
  stream->data_available_.store(true);
  stream->data_available_.notify_one();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  stream->request_shutdown();
  EXPECT_TRUE(wait_for_complete(*stream, std::chrono::milliseconds(2000)));
  stream->shutdown();
  // Playing out the whole queue would take chunks * 5 ms
  EXPECT_LT(audio_writes->load(), chunks / 2);
}

// A full queue at end of stream: the sink plays all queued audio, then ends the stream
TEST(DeviceSinkStop, EndOfStreamFlagPlaysQueuedAudio)
{
  auto audio_writes = std::make_shared<std::atomic<int>>(0);
  int chunks = 0;
  auto stream = make_paced_stream(audio_writes, chunks);
  stream->push_empty_to_queue();  // No room for the marker
  ASSERT_TRUE(stream->end_of_stream_.load());
  ASSERT_FALSE(stream->start().has_value());

  EXPECT_TRUE(wait_for_complete(*stream, std::chrono::milliseconds(5000)));
  stream->shutdown();
  EXPECT_EQ(audio_writes->load(), chunks);
}

// A MessageSink puts its handle in the header of each published chunk
TEST(MessageSinkHandle, HandleInHeader)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  const std::string file =
    (std::filesystem::temp_directory_path() / "test_stream_policy.wav").string();
  {
    const int samplerate = 48000;
    SndfileHandle out(file.c_str(), SFM_WRITE, SF_FORMAT_WAV | SF_FORMAT_PCM_16, 1, samplerate);
    std::vector<short> samples(samplerate / 4);
    for (std::size_t i = 0; i < samples.size(); i++) {
      samples[i] = static_cast<short>(8000 * std::sin(2.0 * M_PI * 440.0 * i / samplerate));
    }
    out.write(samples.data(), samples.size());
  }

  auto node = std::make_shared<rclcpp::Node>("stream_policy_test");
  const std::string topic = "stream_policy_test_chunks";
  auto publisher = node->create_publisher<audio2_stream_msgs::msg::AudioChunk>(topic, 10);
  std::optional<uint8_t> received_handle;
  auto subscription = node->create_subscription<audio2_stream_msgs::msg::AudioChunk>(
    topic, 10, [&received_handle](const audio2_stream_msgs::msg::AudioChunk::SharedPtr msg) {
      if (!received_handle) {
        received_handle = msg->header.handle;
      }
    });

  auto source = std::make_unique<SndFileSource>(file);
  ASSERT_FALSE(source->open().has_value());
  auto sink = std::make_unique<MessageSink>(topic, SF_FORMAT_DEFAULT, publisher, "test",
    SH::HANDLE_QUEUE);
  AudioStream stream(std::move(source), std::move(sink), "message sink handle test");
  ASSERT_FALSE(stream.start().has_value());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!received_handle && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  stream.shutdown();
  std::remove(file.c_str());

  ASSERT_TRUE(received_handle.has_value());
  EXPECT_EQ(*received_handle, SH::HANDLE_QUEUE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
