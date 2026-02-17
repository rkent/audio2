# piper_tts_server

## Description

ROS2 node that runs the Piper TTS (Text-to-Speech) HTTP server using ROS parameters. This node provides a Flask-based web API for text-to-speech synthesis using the Piper voice models. It supports multiple voices, automatic voice downloading, and various synthesis configuration options.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `host` | string | `"0.0.0.0"` | Host address for the HTTP server |
| `port` | integer | `5000` | Port number for the HTTP server |
| `model` | string | `"en_US-bryce-medium"` | Default voice model to use for TTS |
| `speaker` | integer | `-1` | Speaker ID for multi-speaker models (-1 for default) |
| `length_scale` | double | `-1.0` | Speech speed adjustment (-1.0 to use model default) |
| `noise_scale` | double | `-1.0` | Noise level for synthesis (-1.0 to use model default) |
| `noise_w_scale` | double | `-1.0` | Noise width scale for synthesis (-1.0 to use model default) |
| `cuda` | bool | `false` | Enable CUDA acceleration for synthesis |
| `sentence_silence` | double | `0.0` | Silence duration (seconds) between sentences |
| `data_dir` | string[] | `[str(Path.cwd() / 'piper_tts_voices')]` | Directories to search for voice models |
| `download_dir` | string | `""` | Directory for downloading voice models (defaults to first data_dir) |
| `debug` | bool | `false` | Enable debug logging |
| `auto_download_voices` | bool | `true` | Automatically download requested voices if not found locally |

## Example Usage

```bash
# Run with default parameters
ros2 run piper_tts_server piper_tts_server

# Run with custom model and port
ros2 run piper_tts_server piper_tts_server --ros-args \
  -p model:="en_US-joe-medium" \
  -p port:=8080

# Run with CUDA acceleration
ros2 run piper_tts_server piper_tts_server --ros-args \
  -p cuda:=true

# Run with custom data directory
ros2 run piper_tts_server piper_tts_server --ros-args \
  -p data_dir:="['/path/to/voices']"
```

## HTTP API Endpoints

The node exposes the following HTTP endpoints:

- `POST /` - Synthesize speech from text (returns WAV audio)
- `GET /voices` - List downloaded voices
- `GET /all-voices` - List all available Piper voices from HuggingFace
- `GET /download?voice=<model_id>` - Download a voice
- `POST /download` - Download a voice (JSON body: `{"voice": "model_id"}`)
