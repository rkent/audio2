# file_looper

## Node Name
`file_looper`

## Description
The file_looper node is a test utility that loops through audio file data, converting it between different formats and playing it through ALSA. It demonstrates audio format conversion and streaming capabilities by reading chunks from a file, converting them to a different format (configurable in code), and playing them back.

## Parameters
None (uses command-line arguments)

## Example Usage

```bash
# Run with ROS2 (requires passing file path as argument)
ros2 run audio2_stream file_looper /path/to/audio/file.wav

# Direct execution
./file_looper /path/to/audio/file.wav
```

## Notes
- This is a test/utility node, not intended for production use
- Takes a single audio file path as a command-line argument
- Loops through the audio file, converting between formats
- The target format (TOPIC_FORMAT) is defined at compile time and can be:
  - WAV with PCM_32
  - OGG with Vorbis
  - OGG with Opus
  - MPEG Layer III
- Useful for testing audio format conversion and streaming pipelines
- Directly interfaces with ALSA for playback
