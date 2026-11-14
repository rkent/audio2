# audio2
Audio transport and processing for ROS2

## Packages

| Package | Description |
|---|---|
| [`audio2_stream`](audio2_stream/README.md) | Nodes that play and capture audio on Linux (ALSA) and Windows (WASAPI): sound files, text-to-speech, and audio streamed between nodes over topics |
| `audio2_stream_msgs` | Messages for `audio2_stream`: audio chunks, play-file requests and text-to-speech requests |
| [`audio2_samples`](audio2_samples/README.md) | Short public domain (CC0) audio files in many formats and sample rates, for tests and demos |
| `alsa_gain` | Node that sets the gain of audio sources and sinks through ALSA mixer controls (Linux) |
| `alsa_gain_msgs` | Messages for `alsa_gain` |
| `piper_tts_server` | HTTP text-to-speech server using [Piper](https://github.com/rhasspy/piper); `audio2_stream` reaches it through its `piper-http` TTS template |
| `tiny_tts_server` | OpenAI-compatible HTTP text-to-speech server using TinyTTS; `audio2_stream` reaches it through its `tiny-tts-http` TTS template |

To get started, see [audio2_stream](audio2_stream/README.md), which covers building and running
the play node.

## Licenses

Packages are licensed individually; see each `package.xml`. Most are Apache-2.0,
`piper_tts_server` is GPL-3.0-only, and `audio2_samples` is CC0-1.0.
