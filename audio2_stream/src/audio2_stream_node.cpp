#include <csignal>
#include <cstdio>
#include <cstring>
#include <sndfile.h>
#include <alsa/asoundlib.h>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <fstream>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "audio2_stream/alsaops.hpp"
#include "audio2_stream/buffer_file.hpp"
#include "audio2_stream/AudioStream.hpp"
#include "boost/lockfree/spsc_queue.hpp"

#include "audio2_stream_msgs/msg/play_file.hpp"
#include "audio2_stream_msgs/msg/audio_chunk.hpp"

static auto rcl_logger = rclcpp::get_logger("audio2_stream");

#define ALSA_FORMAT SND_PCM_FORMAT_S16
const int ALSA_CHANNELS = 2;
const int ALSA_SAMPLERATE = 48000;

#define SF_FORMAT_DEFAULT (SF_FORMAT_WAV | SF_FORMAT_PCM_16)

class AudioStreamsNode : public rclcpp::Node {
public:
    AudioStreamsNode() : Node("audio_streams_node") {
        // Subscriber for local PlayFile messages

        play_file_local_subscriber_ = this->create_subscription<audio2_stream_msgs::msg::PlayFile>(
            "play_file_local", 10,
            std::bind(&AudioStreamsNode::play_file_local_callback, this, std::placeholders::_1));

        play_file_remote_subscriber_ = this->create_subscription<audio2_stream_msgs::msg::PlayFile>(
            "play_file_remote", 10,
            std::bind(&AudioStreamsNode::play_file_remote_callback, this, std::placeholders::_1));

        // Streams and subscriber for audio chunks
        message_source_ = std::make_unique<MessageSource>(
            "audio_stream_chunks"
        );
        chunk_subscriber_ = this->create_subscription<audio2_stream_msgs::msg::AudioChunk>(
            "audio_stream_chunks", 10,
            std::bind(&AudioStreamsNode::audio_chunk_callback, this, std::placeholders::_1));

        // Register shutdown callback
        rclcpp::on_shutdown(
            std::bind(&AudioStreamsNode::stop_streams_callback, this));

        // Timer to check and clean up finished streams
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&AudioStreamsNode::check_streams_callback, this));
        RCLCPP_INFO(rcl_logger, "AudioStreamsNode initialized.");
    }

    void check_streams_callback()
    {
        std::erase_if(audio_streams_, [](const std::unique_ptr<AudioStream>& stream) {
            return stream->shutdown_flag_.load();
        });
    }

    void stop_streams_callback()
    {
        for (auto& stream : audio_streams_) {
            stream->shutdown();
        }
        audio_streams_.clear();
    }

    void audio_chunk_callback(const audio2_stream_msgs::msg::AudioChunk::SharedPtr msg) {
        RCLCPP_INFO(rcl_logger, "Received audio chunk with %zu bytes", msg->data.size());
        // Here you can handle incoming audio chunks if needed
        auto uuid = msg->header.uuid;
        AudioStream * audio_stream_raw = nullptr;
        // Locate the audio_stream with the matching UUID
        for (auto& stream : audio_streams_) {
            if (stream->stream_uuid_ == uuid) {
                // Pass the chunk to the stream's source
                // message_source_->callback(msg, stream.get());
                audio_stream_raw = stream.get();
                break;
            }
        }
        if (!audio_stream_raw) {
            // Create a matching audio stream
            // Open the playback device
            std::unique_ptr<AlsaSink> alsa_sink = std::make_unique<AlsaSink>(
                ALSA_DEVICE_NAME,
                ALSA_CHANNELS,
                ALSA_SAMPLERATE,
                ALSA_FORMAT
            );
            auto alsa_open_result = alsa_sink->open(SND_PCM_STREAM_PLAYBACK);
            if (alsa_open_result.has_value()) {
                RCLCPP_ERROR(rcl_logger, "Cannot open ALSA device for new stream: %s", alsa_open_result->c_str());
                return;
            }
            auto audio_stream = std::make_unique<AudioStream>(
                sfg_format_from_alsa_format(ALSA_FORMAT),
                nullptr,
                std::move(alsa_sink)
            );
            audio_stream->stream_uuid_ = uuid;
            audio_stream->start();
            audio_stream_raw = audio_stream.get();
            audio_streams_.push_back(std::move(audio_stream));
            RCLCPP_INFO_STREAM(rcl_logger, "\n\nCreated new audio stream for UUID");
        }
        message_source_->callback(msg, audio_stream_raw);
    }

    void play_file_local_callback(const audio2_stream_msgs::msg::PlayFile::SharedPtr msg) {
        RCLCPP_INFO(rcl_logger, "Received PlayFile message for local: path=%s, play_type=%d",
            msg->path.c_str(), msg->play_type);
        auto file_path = msg->path;

        // Open the local file
        std::unique_ptr<SndFileSource> snd_file_source = std::make_unique<SndFileSource>(file_path);
        auto open_result = snd_file_source->open();
        if (open_result.has_value()) {
            RCLCPP_ERROR(rcl_logger, "Cannot open file <%s>: %s", file_path.c_str(), open_result.value().c_str());
            return;
        }

        int channels = snd_file_source->sndfileh_.channels();
        int samplerate = snd_file_source->sndfileh_.samplerate();
        int sndfile_format = snd_file_source->sndfileh_.format();
        RCLCPP_INFO(rcl_logger, "Opened file %s: channels=%d, samplerate=%d, format=0x%X",
            file_path.c_str(), channels, samplerate, sndfile_format);

        // Open the playback device
        std::unique_ptr<AlsaSink> alsa_sink = std::make_unique<AlsaSink>(
            ALSA_DEVICE_NAME,
            channels,
            samplerate,
            ALSA_FORMAT
        );
        auto alsa_open_result = alsa_sink->open(SND_PCM_STREAM_PLAYBACK);
        if (alsa_open_result.has_value()) {
            RCLCPP_ERROR(rcl_logger, "Cannot open ALSA device: %s", alsa_open_result->c_str());
            return;
        }

        SfgRwFormat rw_format = sfg_format_from_alsa_format(ALSA_FORMAT);
        auto audio_stream = std::make_unique<AudioStream>(
            rw_format,
            std::move(snd_file_source),
            std::move(alsa_sink)
        );

        audio_stream->start();
        audio_streams_.push_back(std::move(audio_stream));

        RCLCPP_INFO(rcl_logger, "Enqueued file %s", file_path.c_str());

    }

    void play_file_remote_callback(const audio2_stream_msgs::msg::PlayFile::SharedPtr msg) {
        RCLCPP_INFO(rcl_logger, "Received PlayFile message for remote: path=%s, play_type=%d",
            msg->path.c_str(), msg->play_type);
        auto file_path = msg->path;

        // Open the local file
        std::unique_ptr<SndFileSource> snd_file_source = std::make_unique<SndFileSource>(file_path);
        auto open_result = snd_file_source->open();
        if (open_result.has_value()) {
            RCLCPP_ERROR(rcl_logger, "Cannot open file <%s>: %s", file_path.c_str(), open_result.value().c_str());
            return;
        }

        // Create the message publisher
        auto publisher = this->create_publisher<audio2_stream_msgs::msg::AudioChunk>(msg->topic, 10);
        int channels = snd_file_source->sndfileh_.channels();
        int samplerate = snd_file_source->sndfileh_.samplerate();
        int sndfile_format = snd_file_source->sndfileh_.format();
        RCLCPP_INFO(rcl_logger, "Opened file %s: channels=%d, samplerate=%d, format=0x%X",
            file_path.c_str(), channels, samplerate, sndfile_format);

        // Open the playback device
        std::unique_ptr<MessageSink> message_sink = std::make_unique<MessageSink>(
            msg->topic,
            channels,
            samplerate,
            SF_FORMAT_DEFAULT,
            publisher,
            file_path
        );

        auto audio_stream = std::make_unique<AudioStream>(
            SFG_RW_FORMAT,
            std::move(snd_file_source),
            std::move(message_sink)
        );

        audio_stream->start();
        audio_streams_.push_back(std::move(audio_stream));

        RCLCPP_INFO(rcl_logger, "Enqueued file for remote playback '%s'", file_path.c_str());
    }
private:
    rclcpp::Subscription<audio2_stream_msgs::msg::PlayFile>::SharedPtr play_file_local_subscriber_;
    rclcpp::Subscription<audio2_stream_msgs::msg::PlayFile>::SharedPtr play_file_remote_subscriber_;
    rclcpp::Subscription<audio2_stream_msgs::msg::AudioChunk>::SharedPtr chunk_subscriber_;
    std::vector<std::unique_ptr<AudioStream>> audio_streams_;
    std::unique_ptr<MessageSource> message_source_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, [[maybe_unused]] char ** argv)
{
    RCLCPP_INFO(rcl_logger, "Starting AudioStreamsNode...");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AudioStreamsNode>();
    rclcpp::spin(node);
    RCLCPP_INFO(rcl_logger, "AudioStreamsNode shutting down...");
    rclcpp::shutdown();
    return 0;
}
