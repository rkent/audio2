#ifndef AUDIO2_STREAM_AUDIOSTREAM_HPP
#define AUDIO2_STREAM_AUDIOSTREAM_HPP

#include "audio2_stream/alsaops.hpp"
#include "audio2_stream/buffer_file.hpp"
#include "boost/lockfree/spsc_queue.hpp"
#include <atomic>
#include <vector>
#include <string>
#include <cstdint>
#include <random>

#include "audio2_stream_msgs/msg/audio_chunk.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

const SfgRwFormat SFG_RW_FORMAT = SFG_FLOAT;
const int MAX_HEADER = 128;
const int BUFFER_FRAMES = 480;

// Adapted from https://github.com/autowarefoundation/autoware_utils/tree/main/autoware_utils_uuid
inline unique_identifier_msgs::msg::UUID generate_uuid()
{
  // Generate random number
  unique_identifier_msgs::msg::UUID uuid;
  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(uuid.uuid.begin(), uuid.uuid.end(), bit_eng);

  return uuid;
}

class AudioTerminal;

class AudioStream
{
public:
    AudioStream(
        SfgRwFormat rw_format,
        std::unique_ptr<AudioTerminal> source,
        std::unique_ptr<AudioTerminal> sink
    ) :
        shutdown_flag_(false),
        data_available_(false),
        queue_(AUDIO_QUEUE_SIZE),
        rw_format_(rw_format),
        source_(std::move(source)),
        sink_(std::move(sink)),
        source_thread_(nullptr),
        sink_thread_(nullptr),
        stream_uuid_(generate_uuid())
    {}
    ~AudioStream() { printf("AudioStream::~AudioStream called\n");};

    std::atomic<bool> shutdown_flag_;
    std::atomic<bool> data_available_;
    // TODO: consider making this a unique pointer to reduce copies
    boost::lockfree::spsc_queue<std::vector<uint8_t>> queue_;
    SfgRwFormat rw_format_;
    std::unique_ptr<AudioTerminal> source_;
    std::unique_ptr<AudioTerminal> sink_;
    std::unique_ptr<std::jthread> source_thread_;
    std::unique_ptr<std::jthread> sink_thread_;
    unique_identifier_msgs::msg::UUID stream_uuid_;

    void shutdown();
    void start();
};

class AudioTerminal
{
public:
    AudioTerminal() = default;
    virtual ~AudioTerminal() = default;

    virtual void run(AudioStream * audio_stream) = 0;
};

class SndFileSource : public AudioTerminal
{
public:
    SndFileSource(const std::string & file_path) : file_path_(file_path) {}
    virtual ~SndFileSource() = default;
    std::optional<std::string> open();
    void run(AudioStream * audio_stream) override;
    SndfileHandle sndfileh_;

protected:
    std::string file_path_;
};

class AlsaTerminal : public AudioTerminal
{
public:
    AlsaTerminal(
        std::string alsa_device_name,
        int channels,
        int samplerate,
        snd_pcm_format_t format
    ) :
        alsa_device_name_(alsa_device_name),
        channels_(channels),
        samplerate_(samplerate),
        format_(format),
        alsa_dev_(nullptr)
    {}

    std::optional<std::string> open(snd_pcm_stream_t direction);

    protected:
    std::string alsa_device_name_;
    int channels_;
    int samplerate_;
    snd_pcm_format_t format_;
    std::string error_str_;
    snd_pcm_t* alsa_dev_;

    void close();
};

class AlsaSink : public AlsaTerminal
{
public:
    AlsaSink(
        std::string alsa_device_name,
        int channels,
        int samplerate,
        snd_pcm_format_t format
    ) :
    AlsaTerminal(alsa_device_name, channels, samplerate, format)
    {}

    void run(AudioStream * audio_stream) override;

};

class MessageSink : public AudioTerminal
{
public:
    MessageSink(
        std::string topic,
        int channels,
        int samplerate,
        int sfFormat,
        rclcpp::Publisher<audio2_stream_msgs::msg::AudioChunk>::SharedPtr publisher,
        std::string description
    );

    void run(AudioStream * audio_stream) override;

protected:
    std::string topic_;
    int channels_;
    int samplerate_;
    int sfFormat_;
    rclcpp::Publisher<audio2_stream_msgs::msg::AudioChunk>::SharedPtr publisher_;
    std::string description_;
};

class MessageSource : public AudioTerminal
{
public:
    MessageSource(std::string topic);

    void run(AudioStream * audio_stream) override;
    void callback(const audio2_stream_msgs::msg::AudioChunk::SharedPtr msg, AudioStream * audio_stream);

protected:
    std::string topic_;
};

#endif // AUDIO2_STREAM_AUDIOSTREAM_HPP
