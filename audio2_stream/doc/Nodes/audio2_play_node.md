# audio2_play_node

## Description

ROS2 node for playing audio through an audio device (ALSA on Linux, WASAPI on Windows). This node can play audio from multiple sources including local files, TTS (Text-to-Speech) requests, and audio chunks received over ROS2 topics. The node manages multiple concurrent audio streams and handles stream cleanup automatically.

## Subscribers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `tts_request` | `audio2_stream_msgs/msg/TtsRequest` | Receives TTS synthesis requests and plays the generated audio |
| `play_file_local` | `audio2_stream_msgs/msg/PlayFile` | Receives requests to play audio files locally |
| `audio_stream_chunks` | `audio2_stream_msgs/msg/AudioChunk` | Receives and plays audio chunks from remote sources |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `device_name` | string | `"default"` | Audio device for playback. Linux: ALSA device name. Windows: `"default"` for the default endpoint, or part of a device's friendly name, or its endpoint ID |
| `sample_format` | integer | `14` (`SampleFormat::Float`) | Audio sample format for playback (`SampleFormat` value) |
| `stream_queue_frames` | integer | `480` | Number of frames per audio chunk in the stream |

## Example Usage

```bash
# Run with default parameters
ros2 run audio2_stream audio2_play_node

# Run with a custom ALSA device (Linux)
ros2 run audio2_stream audio2_play_node --ros-args \
  -p device_name:="plughw:CARD=PCH,DEV=0"

# Run on a specific Windows output, matched by part of its name
ros2 run audio2_stream audio2_play_node --ros-args \
  -p device_name:="Speakers"

# Run with custom queue size
ros2 run audio2_stream audio2_play_node --ros-args \
  -p stream_queue_frames:=1024

# Test TTS playback
ros2 topic pub /tts_request audio2_stream_msgs/msg/TtsRequest \
  "{text: 'Hello world', provider: {name: 'piper', voice: 'en_US-bryce-medium'}}"

# Test file playback
ros2 topic pub /play_file_local audio2_stream_msgs/msg/PlayFile \
  "{path: '/path/to/audio.wav', play_type: 0}"
```

## Notes

- The node pre-opens the audio device to reduce latency on first playback
- On Windows, WASAPI shared mode converts any sample rate and channel count to the device mix format. An unknown `device_name` logs the available device names
- Multiple audio streams can play concurrently, managed by unique UUIDs
- The node automatically handles format conversions for TTS sources
- Finished audio streams are automatically cleaned up every 20ms
- The `sample_format` parameter value 14 corresponds to `SampleFormat::Float` (`sample_format.hpp`), numbered like ALSA's `snd_pcm_format_t`
