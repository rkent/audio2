# audio2_stream

ROS 2 nodes for playing and capturing audio: sound files, text-to-speech, and audio streamed
between nodes over topics. Audio devices are accessed through ALSA on Linux and WASAPI on
Windows. Sound files are read with libsndfile (WAV, FLAC, Ogg Vorbis, Ogg Opus, MP3, AIFF and
others).

## Nodes

| Node | Purpose |
|---|---|
| [`audio2_play_node`](doc/Nodes/audio2_play_node.md) | Plays sound files, text-to-speech requests and streamed audio on an audio device |
| [`audio2_capture_node`](doc/Nodes/audio2_capture_node.md) | Captures audio from a device and publishes it as a stream |
| [`audio2_play_remote_node`](doc/Nodes/audio2_play_remote_node.md) | Streams a sound file over a topic, for another node to play |

## Building

From the root of your workspace:

```bash
colcon build --packages-up-to audio2_stream
source install/setup.bash
```

## Running the play node

`launch/audio2_play.launch.py` starts `audio2_play_node` on the default audio device:

```bash
ros2 launch audio2_stream audio2_play.launch.py
```

To use a different device, pass `device_name`:

```bash
# Linux: an ALSA device name (list them with `aplay -L`)
ros2 launch audio2_stream audio2_play.launch.py device_name:=plughw:CARD=PCH,DEV=0

# Windows: part of the device's name, as shown in the Sound settings
ros2 launch audio2_stream audio2_play.launch.py device_name:=Speakers
```

| Launch argument | Default | Meaning |
|---|---|---|
| `device_name` | `default` | Playback device; `default` is the system default |
| `sample_format` | `14` | Device sample format, as a `SampleFormat` value (14 = float) |
| `stream_queue_frames` | `480` | Frames per audio chunk |

To see the arguments from the command line: `ros2 launch audio2_stream audio2_play.launch.py --show-args`.

### Playing something

With the node running, in another terminal (after `source install/setup.bash`):

Play a sound file, here one from the `audio2_samples` package:

```bash
ros2 topic pub --once /play_file_local audio2_stream_msgs/msg/PlayFile \
  "{path: '$(ros2 pkg prefix --share audio2_samples)/test_formats/test.wav'}"
```

Speak some text with espeak (`espeak-ng` or `espeak` must be installed):

```bash
ros2 topic pub --once /tts_request audio2_stream_msgs/msg/TtsRequest "{text: 'Hello from audio two'}"
```

A `handle` field says what happens if other audio is playing: `0` mixes (the default), `1` mutes the others, `2` queues behind them, `3` cancels them, and `4` skips this one. For example, to say something after the current playback finishes:

```bash
ros2 topic pub --once /tts_request audio2_stream_msgs/msg/TtsRequest "{text: 'Next', handle: 2}"
```

See [doc/Nodes/audio2_play_node.md](doc/Nodes/audio2_play_node.md#stream-handling) for details.

Stop the node with Ctrl-C.

## Testing

```bash
colcon test --packages-select audio2_stream
colcon test-result --verbose
```

See [TEST_README.md](TEST_README.md) for details of the tests.
