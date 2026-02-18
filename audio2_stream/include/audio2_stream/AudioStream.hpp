#ifndef AUDIO2_STREAM_AUDIOSTREAM_HPP
#define AUDIO2_STREAM_AUDIOSTREAM_HPP

#include "audio2_stream/config.hpp"
#include "audio2_stream/alsaops.hpp"
#include "audio2_stream/IAlsaDevice.hpp"
#include "audio2_stream/buffer_file.hpp"
#include "boost/lockfree/spsc_queue.hpp"
#include <atomic>
#include <vector>
#include <string>
#include <cstdint>
#include <random>
#include <curl/curl.h>
#include <set>

#include "audio2_stream_msgs/msg/audio_chunk.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

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

typedef enum
{
  TTS_CURL,
  TTS_PROGRAM_WAV,
  TTS_PROGRAM_RAW
} TtsMethod;

class AudioConfigRanges
{
public:
  AudioConfigRanges() {
    // Sample rate values
    samplerate_values.insert(8000);
    samplerate_values.insert(16000);
    samplerate_values.insert(22050);
    samplerate_values.insert(24000);
    samplerate_values.insert(32000);
    samplerate_values.insert(44100);
    samplerate_values.insert(48000);
    samplerate_values.insert(96000);
    samplerate_values.insert(192000);

    // Channel values
    channel_values.insert(1);  // Mono
    channel_values.insert(2);  // Stereo

    // Format values
    format_values.insert(SfgRwFormat::SFG_BYTE);
    format_values.insert(SfgRwFormat::SFG_SHORT);
    format_values.insert(SfgRwFormat::SFG_INT);
    format_values.insert(SfgRwFormat::SFG_FLOAT);
    format_values.insert(SfgRwFormat::SFG_DOUBLE);
  }

  std::set<int> samplerate_values;
  std::set<int> channel_values;
  std::set<SfgRwFormat> format_values;
};

// Forward declaration
class AudioTerminal;

class AudioStream
{
public:
  AudioStream(
    SfgRwFormat rw_format,
    std::unique_ptr<AudioTerminal> source,
    std::unique_ptr<AudioTerminal> sink,
    std::string description = "",
    std::size_t queue_frames = STREAM_QUEUE_FRAMES
  )
  : shutdown_flag_(false),
    data_available_(false),
    queue_(AUDIO_QUEUE_SIZE),
    sink_(std::move(sink)),
    source_(std::move(source)),
    sink_thread_(nullptr),
    source_thread_(nullptr),
    stream_uuid_(generate_uuid()),
    description_(description),
    queue_frames_(queue_frames),
    rw_format_(rw_format)
  {
    printf("AudioStream::AudioStream created for %s with queue frames %i\n", description_.c_str(),
      queue_frames_);
  }

  AudioStream(
    std::unique_ptr<AudioTerminal> source,
    std::unique_ptr<AudioTerminal> sink,
    std::string description = "",
    std::size_t queue_frames = STREAM_QUEUE_FRAMES
  )
  : shutdown_flag_(false),
    data_available_(false),
    queue_(AUDIO_QUEUE_SIZE),
    sink_(std::move(sink)),
    source_(std::move(source)),
    sink_thread_(nullptr),
    source_thread_(nullptr),
    stream_uuid_(generate_uuid()),
    description_(description),
    queue_frames_(queue_frames),
    samplerate_(0),
    channels_(0),
    rw_format_(SFG_NONE)
  {
    printf("AudioStream::AudioStream created for %s with queue frames %i\n", description_.c_str(),
      queue_frames_);
  }

  ~AudioStream()
  {
    printf("AudioStream::~AudioStream thread %zu called for %s\n",
      std::hash<std::thread::id>{}(std::this_thread::get_id()) % 10000, description_.c_str());
  }

  std::atomic<bool> shutdown_flag_;
  std::atomic<bool> data_available_;
  std::atomic<bool> shutdown_complete_{false};
    // TODO: consider making this a unique pointer to reduce copies
  boost::lockfree::spsc_queue<std::vector<uint8_t>> queue_;
  std::unique_ptr<AudioTerminal> sink_;
  std::unique_ptr<AudioTerminal> source_;
  std::unique_ptr<std::jthread> sink_thread_;
  std::unique_ptr<std::jthread> source_thread_;
  unique_identifier_msgs::msg::UUID stream_uuid_;
  std::string description_;
  int queue_frames_;
  int samplerate_;
  int channels_;
  SfgRwFormat rw_format_;

  void shutdown();
  std::optional<std::string> start();
  void process_fileh(SndfileHandle & fileh);
  void process_raw(std::vector<uint8_t> & audio_data, int samplerate, int channels, SfgRwFormat r_format);
  std::set<int> combine_samplerates();
  std::set<int> combine_channels();
  std::set<SfgRwFormat> combine_formats();
  std::optional<std::string> merge_parms();
  std::optional<std::string> fix_parms();

};

class AudioTerminal
{
public:
  AudioTerminal() :
  channels_(0),
  samplerate_(0),
  format_(SfgRwFormat::SFG_NONE),
  config_ranges_(std::make_unique<AudioConfigRanges>())
  {}

  virtual ~AudioTerminal() = default;
  unsigned int channels_;
  unsigned int samplerate_;
  SfgRwFormat format_;
  std::unique_ptr<AudioConfigRanges> config_ranges_;

  virtual void run(AudioStream * audio_stream) = 0;
  virtual std::optional<std::string> fix_rate(unsigned int rate)
  {
    if (rate == samplerate_) {
      return std::nullopt; // No change needed
    } else {
      return "Sample rate mismatch: expected " + std::to_string(samplerate_) +
          ", got " + std::to_string(rate);
    }
  }
};

class SndFileSource : public AudioTerminal
{
public:
  SndFileSource(const std::string & file_path)
  : AudioTerminal(), file_path_(file_path) {}
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
    std::unique_ptr<IAlsaDevice> alsa_device = nullptr
  )
  : AudioTerminal(),
    alsa_device_name_(alsa_device_name),
    alsa_format_(SND_PCM_FORMAT_UNKNOWN),
    alsa_device_(std::move(alsa_device)),
    are_parms_fixed_(false)
  {}

  std::optional<std::string> open(snd_pcm_stream_t direction);

  std::string alsa_device_name_;
  snd_pcm_format_t alsa_format_;
  std::unique_ptr<IAlsaDevice> alsa_device_;
  bool are_parms_fixed_;

  void close();
};

class AlsaSink : public AlsaTerminal
{
public:
  AlsaSink(
    std::string alsa_device_name,
    std::unique_ptr<IAlsaDevice> alsa_device = nullptr
  )
  :AlsaTerminal(alsa_device_name, std::move(alsa_device))
  {}

  void run(AudioStream * audio_stream) override;
  std::optional<std::string> fix_rate(unsigned int rate) override;

};

class AlsaSource : public AlsaTerminal
{
public:
  AlsaSource(
    std::string alsa_device_name,
    std::unique_ptr<IAlsaDevice> alsa_device = nullptr
  )
  :AlsaTerminal(alsa_device_name, std::move(alsa_device))
  {}

  ~AlsaSource()
  {
    printf("AlsaSource::~AlsaSource called\n");
  }

  void run(AudioStream * audio_stream) override;

};

class MessageSink : public AudioTerminal
{
public:
  MessageSink(
    std::string topic,
    int channels,
    unsigned int samplerate,
    int sfFormat,
    rclcpp::Publisher<audio2_stream_msgs::msg::AudioChunk>::SharedPtr publisher,
    std::string description
  ) : AudioTerminal(),
    topic_(topic),
    channels_(channels),
    sfFormat_(sfFormat),
    publisher_(publisher),
    description_(description)
  {
    samplerate_ = samplerate;
  }

  ~MessageSink()
  {
    printf("MessageSink::~MessageSink called\n");
  }

  void run(AudioStream * audio_stream) override;

protected:
  std::string topic_;
  int channels_;
  int sfFormat_;
  rclcpp::Publisher<audio2_stream_msgs::msg::AudioChunk>::SharedPtr publisher_;
  std::string description_;
};

class MessageSource : public AudioTerminal
{
public:
  MessageSource(std::string topic) :
    AudioTerminal(), topic_(topic)
  {}

  ~MessageSource()
  {
    printf("MessageSource::~MessageSource called\n");
  }

  void run(AudioStream * audio_stream) override;
  void callback(
    const audio2_stream_msgs::msg::AudioChunk::SharedPtr msg,
    AudioStream * audio_stream);

protected:
  std::string topic_;
};

class TtsSource : public AudioTerminal
{
public:
  TtsSource(
    const std::string & name,
    const std::string & text,
    const std::string & voice,
    const std::string & model,
    const std::string & format
  ) : AudioTerminal(),
      name_(name),
      text_(text),
      voice_(voice),
      model_(model),
      format_(format)
  {}

  virtual ~TtsSource() {
    curl_slist_free_all(headers_);
  };

  void run(AudioStream * audio_stream) override;
  std::optional<std::string> fetch_tts_curl(std::vector<uint8_t> & audio_data);
  std::optional<std::string> fetch_tts_program(std::vector<uint8_t> & audio_data);
  std::optional<std::string> initialize();

protected:
  std::string name_;
  std::string text_;
  std::string voice_;
  std::string model_;
  std::string format_;
  struct curl_slist * headers_ = nullptr;
  std::string json_str_;
  std::string url_;
  TtsMethod tts_method_;

};

#endif // AUDIO2_STREAM_AUDIOSTREAM_HPP
