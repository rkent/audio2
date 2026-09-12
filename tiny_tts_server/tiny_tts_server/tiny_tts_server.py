#!/usr/bin/env python3
# Copyright (c) 2026 R. Kent James <kent@caspia.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Flask web server with HTTP API for TinyTTS."""

import io
import json
import logging
import wave
from typing import Any, Optional

import numpy as np
import torch
from flask import Flask, request

import rclpy
from rclpy.node import Node

from tiny_tts import TinyTTS
from tiny_tts.nn import commons
from tiny_tts.text import phonemes_to_ids
from tiny_tts.text.english import grapheme_to_phoneme, normalize_text
from tiny_tts.utils.config import ADD_BLANK, SAMPLING_RATE, SPK2ID

_LOGGER = logging.getLogger(__name__)

# WAV constants for TinyTTS output: 44100 Hz, mono, 16-bit PCM
_SAMPLE_RATE = SAMPLING_RATE
_SAMPLE_WIDTH = 2   # bytes (int16)
_CHANNELS = 1


def _synthesize_to_wav(
    model: TinyTTS,
    text: str,
    speaker: str = 'MALE',
    speed: float = 1.0,
) -> bytes:
    """Run TinyTTS inference and return WAV bytes (no disk I/O)."""
    _LOGGER.debug("Synthesizing: %s", text)
    normalized = normalize_text(text)
    phones, tones, _ = grapheme_to_phoneme(normalized)
    phone_ids, tone_ids, lang_ids = phonemes_to_ids(phones, tones, 'EN')

    if ADD_BLANK:
        phone_ids = commons.insert_blanks(phone_ids, 0)
        tone_ids = commons.insert_blanks(tone_ids, 0)
        lang_ids = commons.insert_blanks(lang_ids, 0)

    device = model.device
    x = torch.LongTensor(phone_ids).unsqueeze(0).to(device)
    x_lengths = torch.LongTensor([len(phone_ids)]).to(device)
    tone = torch.LongTensor(tone_ids).unsqueeze(0).to(device)
    language = torch.LongTensor(lang_ids).unsqueeze(0).to(device)

    if speaker not in SPK2ID:
        _LOGGER.warning(
            "Speaker '%s' not found, using 'MALE'. Available: %s",
            speaker, list(SPK2ID.keys()),
        )
        sid = torch.LongTensor([0]).to(device)
    else:
        sid = torch.LongTensor([SPK2ID[speaker]]).to(device)

    bert = torch.zeros(1024, len(phone_ids)).to(device).unsqueeze(0)
    ja_bert = torch.zeros(768, len(phone_ids)).to(device).unsqueeze(0)
    length_scale = 1.0 / speed

    with torch.no_grad():
        audio, *_ = model.model.infer(
            x, x_lengths, sid, tone, language, bert, ja_bert,
            noise_scale=0.667,
            noise_scale_w=0.8,
            length_scale=length_scale,
        )

    audio_np = audio[0, 0].cpu().numpy()
    # Clamp to [-1, 1] then convert float32 → int16
    audio_np = np.clip(audio_np, -1.0, 1.0)
    audio_int16 = (audio_np * 32767).astype(np.int16)

    with io.BytesIO() as buf:
        with wave.open(buf, 'wb') as wf:
            wf.setnchannels(_CHANNELS)
            wf.setsampwidth(_SAMPLE_WIDTH)
            wf.setframerate(_SAMPLE_RATE)
            wf.writeframes(audio_int16.tobytes())
        return buf.getvalue()


class TinyTtsServerNode(Node):
    """ROS2 Node that runs the TinyTTS HTTP server using ROS parameters."""

    def __init__(self) -> None:
        super().__init__('tiny_tts_server')

        # Declare ROS parameters
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 5001)
        self.declare_parameter('checkpoint', '')
        self.declare_parameter('speaker', 'MALE')
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('cuda', False)
        self.declare_parameter('debug', False)

        # Read parameter values
        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        checkpoint_str = (
            self.get_parameter('checkpoint').get_parameter_value().string_value
        )
        default_speaker = (
            self.get_parameter('speaker').get_parameter_value().string_value
        )
        default_speed = (
            self.get_parameter('speed').get_parameter_value().double_value
        )
        use_cuda = self.get_parameter('cuda').get_parameter_value().bool_value
        debug = self.get_parameter('debug').get_parameter_value().bool_value

        logging.basicConfig(level=logging.DEBUG if debug else logging.INFO)
        self.get_logger().info('TinyTTS server starting...')

        # Determine device
        if use_cuda and torch.cuda.is_available():
            device = 'cuda'
        elif use_cuda:
            self.get_logger().warning(
                'CUDA requested but not available, falling back to CPU'
            )
            device = 'cpu'
        else:
            device = 'cpu'

        # Load TinyTTS model
        checkpoint_path: Optional[str] = checkpoint_str if checkpoint_str else None
        self.get_logger().info(
            f'Loading TinyTTS model (device={device}, '
            f'checkpoint={checkpoint_path or "auto"})'
        )
        tts = TinyTTS(checkpoint_path=checkpoint_path, device=device)
        self.get_logger().info('TinyTTS model loaded.')

        # Create Flask web server
        app = Flask(__name__)

        @app.route('/', methods=['POST'])
        def app_synthesize() -> bytes:
            """Synthesize audio from text (OpenAI-compatible TTS endpoint).

            Expected JSON body::

                {
                    "text": "Hello world",
                    "speaker": "MALE",   # optional
                    "speed": 1.0         # optional, 1.5=faster, 0.7=slower
                }

            Returns WAV audio bytes.
            """
            print("Received request:", request.data)
            data = json.loads(request.data)
            text: str = data.get('input', data.get('text', '')).strip()
            if not text:
                raise ValueError('No text provided')

            _LOGGER.debug(data)

            speaker: str = data.get('voice', data.get('speaker', default_speaker))
            # Map OpenAI voice names to TinyTTS speakers where possible
            if speaker.upper() in SPK2ID:
                speaker = speaker.upper()
            elif speaker not in SPK2ID:
                _LOGGER.warning(
                    "Unknown speaker '%s', using default '%s'", speaker, default_speaker
                )
                speaker = default_speaker

            speed: float = float(data.get('speed', default_speed))

            wav_bytes = _synthesize_to_wav(tts, text, speaker=speaker, speed=speed)
            from flask import Response
            return Response(wav_bytes, mimetype='audio/wav')

        @app.route('/speakers', methods=['GET'])
        def app_speakers():
            """List available speakers."""
            return {'speakers': list(SPK2ID.keys())}

        @app.route('/health', methods=['GET'])
        def app_health():
            """Health check endpoint."""
            return {'status': 'ok', 'engine': 'tiny-tts'}

        self._app = app
        self._host = host
        self._port = port

    def start_server(self) -> None:
        """Start the waitress WSGI server (blocking)."""
        from waitress import serve
        self.get_logger().info(
            f'TinyTTS server listening on {self._host}:{self._port}'
        )
        serve(self._app, host=self._host, port=self._port)


def ros_main(args: Any = None) -> None:
    """ROS2 entry point for TinyTtsServerNode."""
    rclpy.init(args=args)
    node = TinyTtsServerNode()
    node.start_server()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    ros_main()
