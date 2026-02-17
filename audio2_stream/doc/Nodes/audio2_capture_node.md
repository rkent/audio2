# audio2_capture_node

## Description

ROS2 node for capturing audio from an ALSA device and publishing it as audio chunks on a ROS2 topic. The node supports configurable audio format, sample rate, and channels. Audio capture can be started automatically on initialization or triggered programmatically.

## Publishers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `audio_stream_chunks` (configurable) | `audio2_stream_msgs/msg/AudioChunk` | Publishes captured audio data as chunks |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `alsa_device_name` | string | `"default"` | ALSA device name for audio capture |
| `alsa_format` | integer | `14` (SND_PCM_FORMAT_FLOAT) | ALSA audio format for capture (snd_pcm_format_t value) |
| `channels` | integer | `2` | Number of audio channels for capture |
| `samplerate` | integer | `48000` | Sample rate for audio capture (Hz) |
| `stream_queue_frames` | integer | `480` | Number of frames per audio chunk in the stream |
| `audio_topic` | string | `"audio_stream_chunks"` | Topic name for publishing captured audio chunks |
| `auto_start` | bool | `true` | Auto-start capturing on node initialization |

## Example Usage

```bash
# Run with default parameters (auto-start capture on default device)
ros2 run audio2_stream audio2_capture_node

# Run with custom device and mono audio
ros2 run audio2_stream audio2_capture_node --ros-args \
  -p alsa_device_name:="hw:1,0" \
  -p channels:=1

# Run with custom sample rate and topic
ros2 run audio2_stream audio2_capture_node --ros-args \
  -p samplerate:=44100 \
  -p audio_topic:="microphone_audio"

# Run without auto-start
ros2 run audio2_stream audio2_capture_node --ros-args \
  -p auto_start:=false
```

## Notes

- The `alsa_format` parameter corresponds to ALSA's `snd_pcm_format_t` enum values (14 = SND_PCM_FORMAT_FLOAT)
- Audio chunks are published with a UUID that identifies the audio stream
- The node automatically cleans up finished audio streams
