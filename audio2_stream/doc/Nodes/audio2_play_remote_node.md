# audio2_play_remote_node

## Description

ROS2 node for streaming audio files to remote playback devices via ROS2 topics. This node reads local audio files and publishes them as audio chunks that can be played by other audio playback nodes. It enables distributed audio playback across multiple machines in a ROS2 network.

## Publishers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| (dynamic) | `audio2_stream_msgs/msg/AudioChunk` | Publishes audio file data as chunks (topic specified in PlayFile message) |

## Subscribers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `play_file_remote` | `audio2_stream_msgs/msg/PlayFile` | Receives requests to stream audio files remotely |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `stream_queue_frames` | integer | `480` | Number of frames per audio chunk in the stream |

## Example Usage

```bash
# Run with default parameters
ros2 run audio2_stream audio2_play_remote_node

# Run with custom queue size
ros2 run audio2_stream audio2_play_remote_node --ros-args \
  -p stream_queue_frames:=1024

# Request remote playback of a file
ros2 topic pub /play_file_remote audio2_stream_msgs/msg/PlayFile \
  "{path: '/path/to/audio.wav', topic: 'remote_audio_stream', play_type: 0}"
```

## Notes

- The topic name for publishing audio chunks is specified in the `PlayFile` message
- Multiple files can be streamed concurrently on different topics
- The node automatically handles audio file format conversions
- Finished streams are cleaned up automatically every 20ms
- This node is typically used in conjunction with `audio2_play_node` on remote machines
