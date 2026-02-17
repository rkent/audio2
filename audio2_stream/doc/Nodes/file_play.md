# file_play

## Description

Simple test/utility node for playing an audio file once through an ALSA device. This node opens an audio file, configures the ALSA device based on the file's properties, and plays the audio to completion. It is primarily used for testing basic audio playback functionality.

## Parameters

None (file path is provided as command line argument)

## Example Usage

```bash
# Play an audio file
ros2 run audio2_stream file_play /path/to/audio.wav

# Play an OGG file
ros2 run audio2_stream file_play /path/to/audio.ogg

# Play an OPUS file
ros2 run audio2_stream file_play /path/to/audio.opus
```

## Notes

- The audio file path must be provided as the first command line argument
- The file is played once and the node exits when playback completes
- Supports various audio formats through libsndfile (WAV, OGG, OPUS, MP3, etc.)
- Uses a default ALSA device and format defined at compile time
- The ALSA device is automatically configured to match the file's sample rate and channels
- This is primarily a test utility, not intended for production use
