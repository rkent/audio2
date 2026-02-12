# audio2_capture_node

## Node Name
`audio2_capture_node`

## Description
The audio2_capture_node captures audio from ALSA devices and publishes it as audio chunks to a ROS2 topic. It can be configured to automatically start capturing on initialization or wait for explicit start commands.

## Publishers

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `audio_stream_chunks` (configurable) | `audio2_stream_msgs/msg/AudioChunk` | Publishes captured audio data chunks. The topic name is configurable via the `audio_topic` parameter. |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `alsa_device_name` | string | `"default"` | ALSA device name for audio capture. Must be a valid ALSA device name. |
| `alsa_format` | int | `SND_PCM_FORMAT_FLOAT` | ALSA audio format for capture. Must be a valid `snd_pcm_format_t` integer value. |
| `channels` | int | `2` | Number of audio channels for capture. Typically 1 (mono) or 2 (stereo). |
| `samplerate` | int | `48000` | Sample rate for audio capture in Hz (e.g., 44100, 48000). |
| `stream_queue_frames` | int | `480` | Number of frames per audio chunk in the stream. Must be a positive integer. |
| `audio_topic` | string | `"audio_stream_chunks"` | Topic name for publishing captured audio chunks. Must be a valid ROS topic name. |
| `auto_start` | bool | `true` | Auto-start capturing on node initialization. Set to false to manually trigger capture start. |

## Example Usage

```bash
# Run with default parameters (auto-starts capture)
ros2 run audio2_stream audio2_capture_node

# Run with custom ALSA device and sample rate
ros2 run audio2_stream audio2_capture_node --ros-args \
  -p alsa_device_name:="hw:1,0" \
  -p samplerate:=44100 \
  -p channels:=1

# Run without auto-start
ros2 run audio2_stream audio2_capture_node --ros-args \
  -p auto_start:=false \
  -p audio_topic:="my_audio_stream"
```

## Notes
- When `auto_start` is true, the node begins capturing immediately upon initialization
- The node automatically manages stream lifecycle and cleanup
- Captured audio is published in chunks as `AudioChunk` messages
