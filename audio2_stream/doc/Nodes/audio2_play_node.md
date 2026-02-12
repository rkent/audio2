# audio2_play_node

## Node Name
`audio2_play_node`

## Description
The audio2_play_node is responsible for audio playback on ALSA devices. It can play audio from multiple sources: local files, text-to-speech (TTS) requests, and audio chunk streams from remote sources. The node manages multiple concurrent audio streams and pre-opens the ALSA device to reduce playback latency.

## Subscriptions

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `tts_request` | `audio2_stream_msgs/msg/TtsRequest` | Receives text-to-speech requests with provider configuration and text to be synthesized |
| `play_file_local` | `audio2_stream_msgs/msg/PlayFile` | Receives requests to play audio files from the local filesystem |
| `audio_stream_chunks` | `audio2_stream_msgs/msg/AudioChunk` | Receives audio data chunks for streaming playback |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `alsa_device_name` | string | `"default"` | ALSA device name for audio playback. Must be a valid ALSA device name. |
| `alsa_format` | int | `SND_PCM_FORMAT_FLOAT` | ALSA audio format for playback. Must be a valid `snd_pcm_format_t` integer value. |
| `stream_queue_frames` | int | `480` | Number of frames per audio chunk in the stream. Must be a positive integer. |

## Example Usage

```bash
# Run with default parameters
ros2 run audio2_stream audio2_play_node

# Run with custom ALSA device
ros2 run audio2_stream audio2_play_node --ros-args -p alsa_device_name:="hw:0,0"

# Run with custom parameters
ros2 run audio2_stream audio2_play_node --ros-args \
  -p alsa_device_name:="hw:0,0" \
  -p stream_queue_frames:=2048
```

## Notes
- The node pre-opens the ALSA playback device to reduce latency on the first playback request
- Supports concurrent playback of multiple audio streams
- Automatically cleans up completed audio streams
- For audio chunks, the node creates a new stream for each unique UUID in the message header
