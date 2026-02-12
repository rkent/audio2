# file_play

## Node Name
`file_play`

## Description
The file_play node is a test utility that plays audio files directly to an ALSA device. It reads audio data from a file and streams it through the ALSA audio subsystem. This node is primarily used for testing and debugging audio playback functionality.

## Parameters
None (uses command-line arguments)

## Example Usage

```bash
# Run with ROS2 (requires passing file path as argument)
ros2 run audio2_stream file_play /path/to/audio/file.wav

# Direct execution
./file_play /path/to/audio/file.wav
```

## Notes
- This is a test/utility node, not intended for production use
- Takes a single audio file path as a command-line argument
- Plays the audio file once and exits
- Directly opens and configures the ALSA device based on the audio file properties
- Supports various audio file formats readable by libsndfile
