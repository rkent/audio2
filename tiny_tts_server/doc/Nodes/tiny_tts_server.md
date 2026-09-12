# tiny_tts_server

## Description

ROS2 node that runs the TinyTTS (Text-to-Speech) HTTP server using ROS parameters.
This node provides a Flask-based web API for text-to-speech synthesis using the
[TinyTTS](https://github.com/tronghieuit/tiny-tts) engine — an ultra-lightweight
(~1.6M parameter) English TTS model that runs comfortably on CPU.

The HTTP API is compatible with the OpenAI text-to-speech protocol used by the
`audio2_play_node` in `audio2_stream`.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `host` | string | `"0.0.0.0"` | Host address for the HTTP server |
| `port` | integer | `5001` | Port number for the HTTP server |
| `checkpoint` | string | `""` | Path to `G.pth` checkpoint file (auto-downloads from HuggingFace if empty) |
| `speaker` | string | `"MALE"` | Default speaker ID (`"MALE"` is the only built-in speaker) |
| `speed` | double | `1.0` | Default speech speed (1.0=normal, 1.5=faster, 0.7=slower) |
| `cuda` | bool | `false` | Enable CUDA GPU acceleration |
| `debug` | bool | `false` | Enable debug logging |

## Example Usage

```bash
# Run with default parameters (auto-downloads checkpoint on first run)
ros2 run tiny_tts_server tiny_tts_server

# Run on a custom port with a local checkpoint
ros2 run tiny_tts_server tiny_tts_server --ros-args \
  -p port:=5001 \
  -p checkpoint:="/path/to/G.pth"

# Run with CUDA acceleration
ros2 run tiny_tts_server tiny_tts_server --ros-args \
  -p cuda:=true

# Run with slower, more deliberate speech
ros2 run tiny_tts_server tiny_tts_server --ros-args \
  -p speed:=0.8
```

## HTTP API Endpoints

The node exposes the following HTTP endpoints:

### `POST /` — Synthesize speech

Accepts a JSON body. Supports both the OpenAI TTS field names and the piper_tts_server
field names for compatibility:

| Field | Type | Description |
|-------|------|-------------|
| `input` or `text` | string | **Required.** Text to synthesize |
| `voice` or `speaker` | string | Speaker name (default: `"MALE"`) |
| `speed` | float | Speech speed multiplier (default: `1.0`) |

Returns `audio/wav` bytes.

**Example request:**
```bash
curl -X POST http://localhost:5001/ \
  -H 'Content-Type: application/json' \
  -d '{"input": "Hello world", "speed": 1.0}' \
  --output hello.wav
```

### `GET /speakers` — List available speakers

Returns a JSON object listing the available speaker IDs.

```json
{"speakers": ["MALE"]}
```

### `GET /health` — Health check

```json
{"status": "ok", "engine": "tiny-tts"}
```

## Audio Output Format

- **Sample rate:** 44,100 Hz
- **Channels:** Mono (1)
- **Bit depth:** 16-bit PCM
- **Container:** WAV
