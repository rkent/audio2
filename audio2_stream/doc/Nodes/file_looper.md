# file_looper

## Description

Test/utility node for looping audio file playback through an ALSA device. This node reads an audio file, converts it to a specified format, and plays it back continuously until interrupted. It is primarily used for testing audio streaming and format conversion functionality.

## Parameters

None (file path is provided as command line argument)

## Example Usage

```bash
# Play an audio file in a loop
ros2 run audio2_stream file_looper /path/to/audio.wav

# Play an OGG file
ros2 run audio2_stream file_looper /path/to/audio.ogg

# Play an OPUS file
ros2 run audio2_stream file_looper /path/to/audio.opus
```

## Notes

- The audio file path must be provided as the first command line argument
- The node loops the file continuously until stopped with Ctrl+C
- Supports various audio formats through libsndfile (WAV, OGG, OPUS, MP3, etc.)
- Uses a default ALSA device and format defined at compile time
- This is primarily a test utility, not intended for production use
