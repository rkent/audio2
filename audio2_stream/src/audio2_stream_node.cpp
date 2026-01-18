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

static std::string format_timestamp() {
    auto time_point = std::chrono::steady_clock::now();
    auto time_since_epoch = time_point.time_since_epoch();
    auto hours = std::chrono::duration_cast<std::chrono::hours>(time_since_epoch) % 24;
    auto minutes = std::chrono::duration_cast<std::chrono::minutes>(time_since_epoch) % 60;
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch) % 60;
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch) % 1000;
    
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "%02ld:%02ld:%02ld:%03ld",
                hours.count(), minutes.count(), seconds.count(), milliseconds.count());
    return std::string(buffer);
}

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
        // TODO: Figure out best QOS settings for audio chunk subscription
        chunk_subscriber_ = this->create_subscription<audio2_stream_msgs::msg::AudioChunk>(
            "audio_stream_chunks", AUDIO_CHUNK_QOS,
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
        std::size_t hash_id = std::hash<std::thread::id>{}(std::this_thread::get_id()) % 10000;
        printf("audio_chunk_callback: thread %zu Received audio chunk with %zu bytes sequence %u at %s\n", hash_id, msg->data.size(), msg->header.sequence, format_timestamp().c_str());
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

                // We need to do an initial open of the source to get the format
            VIO_SOUNDFILE_HANDLE tr_handle;
            tr_handle.vio_data.data = reinterpret_cast<char *>(msg->data.data());
            tr_handle.vio_data.length = msg->data.size();
            tr_handle.vio_data.offset = 0;
            tr_handle.vio_data.capacity =  msg->data.size();

            if (auto err = open_sndfile_from_buffer2(tr_handle, SFM_READ)) {
                RCLCPP_ERROR(rcl_logger, "Failed to open sound file for reading from buffer: %s", err->c_str());
                return;
            }

            // Create a matching audio stream
            // Open the playback device
            std::unique_ptr<AlsaSink> alsa_sink = std::make_unique<AlsaSink>(
                ALSA_DEVICE_NAME,
                tr_handle.fileh.channels(),
                tr_handle.fileh.samplerate(),
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
                std::move(alsa_sink),
                std::string("Stream for topic callback")
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
            std::move(alsa_sink),
            std::string("Local playback of ") + file_path
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
        auto publisher = this->create_publisher<audio2_stream_msgs::msg::AudioChunk>(msg->topic, AUDIO_CHUNK_QOS);
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
            std::move(message_sink),
            std::string("Remote playback of ") + file_path
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
    RCLCPP_INFO(rcl_logger, "Starting AudioStreamsNode thread %zu at %s", 
                std::hash<std::thread::id>{}(std::this_thread::get_id()) % 10000,
                format_timestamp().c_str());
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AudioStreamsNode>();
    rclcpp::spin(node);
    RCLCPP_INFO(rcl_logger, "AudioStreamsNode shutting down...");
    rclcpp::shutdown();
    return 0;
}
