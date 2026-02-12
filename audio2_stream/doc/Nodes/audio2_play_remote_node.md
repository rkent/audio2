# audio2_play_remote_node

## Node Name
`audio2_play_remote_node`

## Description
The audio2_play_remote_node reads audio files from the local filesystem and publishes them as audio chunk streams to ROS2 topics for remote playback. This allows audio files to be streamed to other nodes or systems over the ROS2 network.

## Subscriptions

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `play_file_remote` | `audio2_stream_msgs/msg/PlayFile` | Receives requests to play audio files and stream them to a specified topic |

## Publishers

| Topic | Message Type | Description |
|-------|-------------|-------------|
| Dynamic (from PlayFile message) | `audio2_stream_msgs/msg/AudioChunk` | Publishes audio data chunks to the topic specified in the PlayFile message |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `stream_queue_frames` | int | `480` | Number of frames per audio chunk in the stream. Must be a positive integer. |

## Example Usage

```bash
# Run with default parameters
ros2 run audio2_stream audio2_play_remote_node

# Run with custom stream queue frames
ros2 run audio2_stream audio2_play_remote_node --ros-args \
  -p stream_queue_frames:=4096
```

## Notes
- The topic name for publishing audio chunks is specified in each `PlayFile` message
- Supports multiple concurrent file streams
- Automatically cleans up completed streams
- The node reads the audio file format from the file and streams it with appropriate encoding
