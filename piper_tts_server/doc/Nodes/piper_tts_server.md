# piper_tts_server

## Node Name
`piper_tts_server`

## Description
The piper_tts_server node runs a Flask-based HTTP server for the Piper Text-to-Speech (TTS) engine. It provides a web API for synthesizing speech from text using various voice models. The server supports voice management, including downloading voices from HuggingFace and listing available voices. All configuration is done through ROS2 parameters.

## HTTP API Endpoints

The node exposes a Flask web server with the following endpoints:

### POST /
Synthesize audio from text
- **Request Body**: JSON with fields:
  - `text` (required): Text to synthesize
  - `voice` (optional): Voice model ID to use
  - `speaker_id` (optional): Speaker ID for multi-speaker models
  - `speaker` (optional): Speaker name for multi-speaker models
  - `length_scale` (optional): Speech rate adjustment
  - `noise_scale` (optional): Variation in speech
  - `noise_w_scale` (optional): Variation in speech rhythm
- **Response**: WAV audio file (bytes)

### GET /voices
List all downloaded voices
- **Response**: JSON dictionary of voice configurations

### GET /all-voices
List all available Piper voices from HuggingFace
- **Response**: JSON from Piper voices repository

### GET /download
Download a voice using query parameters
- **Query Parameters**:
  - `voice` (required): Voice model ID to download
  - `force_redownload` (optional): Force re-download if already exists
- **Response**: JSON voice configuration

### POST /download
Download a voice
- **Request Body**: JSON with fields:
  - `voice` (required): Voice model ID to download
  - `force_redownload` (optional): Force re-download if already exists
- **Response**: JSON voice configuration

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `host` | string | `"0.0.0.0"` | Host address for the HTTP server to bind to |
| `port` | int | `5000` | Port number for the HTTP server |
| `model` | string | `"en_US-bryce-medium"` | Default voice model to use for TTS |
| `speaker` | int | `-1` | Default speaker ID for multi-speaker models (-1 means not set) |
| `length_scale` | double | `-1.0` | Default speech rate adjustment (-1.0 means use voice default) |
| `noise_scale` | double | `-1.0` | Default variation in speech (-1.0 means use voice default) |
| `noise_w_scale` | double | `-1.0` | Default variation in speech rhythm (-1.0 means use voice default) |
| `cuda` | bool | `false` | Enable CUDA acceleration for TTS synthesis |
| `sentence_silence` | double | `0.0` | Duration of silence between sentences in seconds |
| `data_dir` | string[] | `["piper_tts_voices"]` | List of directories to search for voice model files |
| `download_dir` | string | `""` | Directory for downloading voice models (defaults to first data_dir) |
| `debug` | bool | `false` | Enable debug logging |
| `auto_download_voices` | bool | `true` | Automatically download voice models if not found locally |

## Example Usage

```bash
# Run with default parameters
ros2 run piper_tts_server piper_tts_server

# Run with custom port and model
ros2 run piper_tts_server piper_tts_server --ros-args \
  -p port:=8080 \
  -p model:="en_US-joe-medium"

# Run with custom host, port, and data directory
ros2 run piper_tts_server piper_tts_server --ros-args \
  -p host:="localhost" \
  -p port:=5001 \
  -p data_dir:="['/path/to/voices', '/another/path']" \
  -p auto_download_voices:=true

# Run with CUDA acceleration
ros2 run piper_tts_server piper_tts_server --ros-args \
  -p cuda:=true \
  -p model:="en_US-bryce-medium"
```

## Notes
- The server runs using the Waitress WSGI server for production use
- Voice models are cached after first load for better performance
- If a requested voice is not found and `auto_download_voices` is true, the node will automatically download it from HuggingFace
- The server continues running until the node is shut down
- Synthesis parameters can be specified per-request or use the node's defaults
