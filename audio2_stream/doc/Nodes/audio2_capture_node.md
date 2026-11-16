# audio2_capture_node

## Description

ROS2 node for capturing audio from an audio device (ALSA on Linux, WASAPI on Windows) and publishing it as audio chunks on a ROS2 topic. The node supports configurable audio format, sample rate, and channels. Audio capture can be started automatically on initialization or triggered programmatically.

## Publishers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `audio_stream_chunks` (configurable) | `audio2_stream_msgs/msg/AudioChunk` | Publishes captured audio data as chunks |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `device_name` | string | `"default"` | Audio device for capture. Linux: ALSA device name. Windows: `"default"` for the default endpoint, or part of a device's friendly name, or its endpoint ID |
| `sample_format` | integer | `14` (`SampleFormat::Float`) | Audio sample format for capture (`SampleFormat` value) |
| `channels` | integer | `2` | Number of audio channels for capture |
| `samplerate` | integer | `48000` | Sample rate for audio capture (Hz) |
| `stream_queue_frames` | integer | `480` | Number of frames per audio chunk in the stream |
| `audio_topic` | string | `"audio_stream_chunks"` | Topic name for publishing captured audio chunks |
| `auto_start` | bool | `true` | Auto-start capturing on node initialization |
| `handle` | integer | `0` | `StreamHeader.HANDLE_*` value sent in each chunk, telling players how to handle the captured stream relative to other streams (0 = mix, 1 = mute others, 2 = queue, 3 = cancel others, 4 = ignore if others are playing) |

## Example Usage

```bash
# Run with default parameters (auto-start capture on default device)
ros2 run audio2_stream audio2_capture_node

# Run with custom device and mono audio
ros2 run audio2_stream audio2_capture_node --ros-args \
  -p device_name:="hw:1,0" \
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

- On Windows, WASAPI shared mode converts the device's audio to the requested sample rate and channel count. An unknown `device_name` logs the available device names
- The `sample_format` parameter is a `SampleFormat` value (`sample_format.hpp`), numbered like ALSA's `snd_pcm_format_t` (14 = `SampleFormat::Float`)
- Audio chunks are published with a UUID that identifies the audio stream
- The node automatically cleans up finished audio streams
