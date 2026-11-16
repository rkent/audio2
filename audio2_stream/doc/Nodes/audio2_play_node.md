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

## Stream Handling

Each stream carries a `handle` (in `PlayFile`, `TtsRequest`, and the `StreamHeader` of each `AudioChunk`) that says what happens when it arrives while other streams are playing. Values are the `StreamHeader.HANDLE_*` constants:

| Value | Name | Behavior |
|-------|------|----------|
| `0` | `HANDLE_MIX` (default) | Play at once, mixed with the other streams |
| `1` | `HANDLE_MUTE` | Play at once and silence all older streams while this one plays. Muted streams keep running (their audio is replaced by silence, not paused) and become audible again when it ends |
| `2` | `HANDLE_QUEUE` | Wait until all playing and earlier queued streams have finished, then play. Chunks of a queued chunk stream are buffered until it starts |
| `3` | `HANDLE_CANCEL` | Stop all playing and queued streams, then play. Later chunks of cancelled chunk streams are discarded |
| `4` | `HANDLE_IGNORE` | Do not play if any stream is playing or queued. Later chunks of an ignored chunk stream are discarded |

An unknown value is treated as `HANDLE_MIX`, with a warning. A chunk stream's handle is taken from its first chunk.

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

# Play a file after any current playback finishes
ros2 topic pub --once /play_file_local audio2_stream_msgs/msg/PlayFile \
  "{path: '/path/to/audio.wav', handle: 2}"
```

## Notes

- The node pre-opens the audio device to reduce latency on first playback
- On Windows, WASAPI shared mode converts any sample rate and channel count to the device mix format. An unknown `device_name` logs the available device names
- Multiple audio streams can play concurrently, managed by unique UUIDs. See [Stream Handling](#stream-handling)
- The node automatically handles format conversions for TTS sources
- Finished audio streams are automatically cleaned up every 20ms
- The `sample_format` parameter value 14 corresponds to `SampleFormat::Float` (`sample_format.hpp`), numbered like ALSA's `snd_pcm_format_t`
