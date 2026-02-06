#!/usr/bin/env python3
# Copyright (c) 2026 R. Kent James <kent@caspia.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

# Adapted from https://github.com/OHF-Voice/piper1-gpl/blob/main/src/piper/http_server.py
# to run as a ROS node rather than a CLI.

"""Flask web server with HTTP API for Piper."""

import argparse
import io
import json
import logging
import wave
from pathlib import Path
import threading
from typing import Any, Dict, List, Optional
from urllib.request import urlopen

from flask import Flask, request

import rclpy
from rclpy.node import Node


from piper import PiperVoice, SynthesisConfig
from piper.download_voices import VOICES_JSON, download_voice

_LOGGER = logging.getLogger()


def main() -> None:
    """Run HTTP server."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0", help="HTTP server host")
    parser.add_argument("--port", type=int, default=5000, help="HTTP server port")
    #
    parser.add_argument("-m", "--model", required=True, help="Path to Onnx model file")
    #
    parser.add_argument("-s", "--speaker", type=int, help="Id of speaker (default: 0)")
    parser.add_argument(
        "--length-scale", "--length_scale", type=float, help="Phoneme length"
    )
    parser.add_argument(
        "--noise-scale", "--noise_scale", type=float, help="Generator noise"
    )
    parser.add_argument(
        "--noise-w-scale",
        "--noise_w_scale",
        "--noise-w",
        "--noise_w",
        type=float,
        help="Phoneme width noise",
    )
    #
    parser.add_argument("--cuda", action="store_true", help="Use GPU")
    #
    parser.add_argument(
        "--sentence-silence",
        "--sentence_silence",
        type=float,
        default=0.0,
        help="Seconds of silence after each sentence",
    )
    #
    parser.add_argument(
        "--data-dir",
        "--data_dir",
        action="append",
        default=[str(Path.cwd())],
        help="Data directory to check for downloaded models (default: current directory)",
    )
    parser.add_argument(
        "--download-dir",
        "--download_dir",
        help="Path to download voices (default: first data dir)",
    )
    #
    parser.add_argument(
        "--debug", action="store_true", help="Print DEBUG messages to console"
    )
    args = parser.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.debug else logging.INFO)
    _LOGGER.debug(args)

    if not args.download_dir:
        # Download voices to first data directory if not specified
        args.download_dir = args.data_dir[0]

    download_dir = Path(args.download_dir)

    # Download voice if file doesn't exist
    model_path = Path(args.model)
    if not model_path.exists():
        # Look in data directories
        voice_name = args.model
        for data_dir in args.data_dir:
            maybe_model_path = Path(data_dir) / f"{voice_name}.onnx"
            _LOGGER.debug("Checking '%s'", maybe_model_path)
            if maybe_model_path.exists():
                model_path = maybe_model_path
                break

    if not model_path.exists():
        raise ValueError(
            f"Unable to find voice: {model_path} (use piper.download_voices)"
        )

    default_model_id = model_path.name.rstrip(".onnx")

    # Load voice
    default_voice = PiperVoice.load(model_path, use_cuda=args.cuda)
    loaded_voices: Dict[str, PiperVoice] = {default_model_id: default_voice}

    # Create web server
    app = Flask(__name__)

    @app.route("/voices", methods=["GET"])
    def app_voices() -> Dict[str, Any]:
        """List downloaded voices.

        Outputs a JSON object with the format:
        {
          "<voice name>": { <voice config> },
          ...
        }

        for each voice in your data directories.
        """
        voices_dict: Dict[str, Any] = {}
        config_paths: List[Path] = [Path(f"{model_path}.json")]

        for data_dir in args.data_dir:
            for onnx_path in Path(data_dir).glob("*.onnx"):
                config_path = Path(f"{onnx_path}.json")
                if config_path.exists():
                    config_paths.append(config_path)

        for config_path in config_paths:
            model_id = config_path.name.rstrip(".onnx.json")
            if model_id in voices_dict:
                continue

            with open(config_path, "r", encoding="utf-8") as config_file:
                voices_dict[model_id] = json.load(config_file)

        return voices_dict

    @app.route("/all-voices", methods=["GET"])
    def app_all_voices() -> Dict[str, Any]:
        """List all Piper voices.

        Outputs voices.json from the piper-voices repo on HuggingFace.
        See: https://huggingface.co/rhasspy/piper-voices
        """
        with urlopen(VOICES_JSON) as response:
            return json.load(response)

    @app.route("/download", methods=["POST"])
    def app_download() -> str:
        """Download a voice.

        Downloads the .onnx and .onnx.json file from piper-voices repo on HuggingFace.
        See: https://huggingface.co/rhasspy/piper-voices

        Expects a JSON object with the format:
        {
          "voice": "<voice name>",   (required)
          "force_redownload": false  (optional)
        }

        Returns the name of the voice.
        Voice format must be <language>-<name>-<quality> like "en_US-lessac-medium".
        """
        data = json.loads(request.data)
        model_id = data.get("voice")
        if not model_id:
            raise ValueError("voice is required")

        force_redownload = data.get("force_redownload", False)
        download_voice(model_id, download_dir, force_redownload=force_redownload)

        return model_id

    @app.route("/", methods=["POST"])
    def app_synthesize() -> bytes:
        """Synthesize audio from text.

        Expects a JSON object with the format:
        {
          "text": "Text to speak.",      (required)
          "voice": "<voice name>",       (optional)
          "speaker": "<speaker name>",   (optional)
          "speaker_id": "<speaker id>",  (optional, overrides speaker)
          "length_scale": 1.0,           (optional)
          "noise_scale": 0.667,          (optional)
          "length_w_scale": 0.8          (optional)
        }
        """
        data = json.loads(request.data)
        text = data.get("text", "").strip()
        if not text:
            raise ValueError("No text provided")

        _LOGGER.debug(data)

        model_id = data.get("voice", default_model_id)
        voice = loaded_voices.get(model_id)
        if voice is None:
            for data_dir in args.data_dir:
                maybe_model_path = Path(data_dir) / f"{model_id}.onnx"
                if maybe_model_path.exists():
                    _LOGGER.debug("Loading voice %s", model_id)
                    voice = PiperVoice.load(maybe_model_path, use_cuda=args.cuda)
                    loaded_voices[model_id] = voice
                    break

        if voice is None:
            _LOGGER.warning("Voice not found: %s. Using default voice.", model_id)
            voice = default_voice

        speaker_id: Optional[int] = data.get("speaker_id")
        if (voice.config.num_speakers > 1) and (speaker_id is None):
            speaker = data.get("speaker")
            if speaker:
                speaker_id = voice.config.speaker_id_map.get(speaker)

            if speaker_id is None:
                _LOGGER.warning(
                    "Speaker not found: '%s' in %s",
                    speaker,
                    voice.config.speaker_id_map.keys(),
                )
                speaker_id = args.speaker or 0

        if (speaker_id is not None) and (speaker_id > voice.config.num_speakers):
            speaker_id = 0

        syn_config = SynthesisConfig(
            speaker_id=speaker_id,
            length_scale=float(
                data.get(
                    "length_scale",
                    (
                        args.length_scale
                        if args.length_scale is not None
                        else voice.config.length_scale
                    ),
                )
            ),
            noise_scale=float(
                data.get(
                    "noise_scale",
                    (
                        args.noise_scale
                        if args.noise_scale is not None
                        else voice.config.noise_scale
                    ),
                )
            ),
            noise_w_scale=float(
                data.get(
                    "noise_w_scale",
                    (
                        args.noise_w_scale
                        if args.noise_w_scale is not None
                        else voice.config.noise_w_scale
                    ),
                )
            ),
        )

        _LOGGER.debug("Synthesizing text: '%s' with config=%s", text, syn_config)
        with io.BytesIO() as wav_io:
            wav_file: wave.Wave_write = wave.open(wav_io, "wb")
            with wav_file:
                wav_params_set = False
                for i, audio_chunk in enumerate(voice.synthesize(text, syn_config)):
                    if not wav_params_set:
                        wav_file.setframerate(audio_chunk.sample_rate)
                        wav_file.setsampwidth(audio_chunk.sample_width)
                        wav_file.setnchannels(audio_chunk.sample_channels)
                        wav_params_set = True

                    if i > 0:
                        wav_file.writeframes(
                            bytes(
                                int(
                                    voice.config.sample_rate * args.sentence_silence * 2
                                )
                            )
                        )

                    wav_file.writeframes(audio_chunk.audio_int16_bytes)

            return wav_io.getvalue()

    app.run(host=args.host, port=args.port)


class PiperTtsServerNode(Node):
    """ROS2 Node that runs the Piper TTS HTTP server using ROS parameters."""

    def __init__(self) -> None:
        super().__init__('piper_tts_server')

        # Declare ROS parameters with defaults matching the argparse CLI
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 5000)
        self.declare_parameter('model', '')
        self.declare_parameter('speaker', -1)
        self.declare_parameter('length_scale', -1.0)
        self.declare_parameter('noise_scale', -1.0)
        self.declare_parameter('noise_w_scale', -1.0)
        self.declare_parameter('cuda', False)
        self.declare_parameter('sentence_silence', 0.0)
        self.declare_parameter('data_dir', [str(Path.cwd() / 'piper_tts_voices')])
        self.declare_parameter('download_dir', '')
        self.declare_parameter('debug', False)
        self.declare_parameter('auto_download_voices', True)

        # Read parameter values
        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        model = self.get_parameter('model').get_parameter_value().string_value
        speaker_param = self.get_parameter('speaker').get_parameter_value().integer_value
        length_scale_param = self.get_parameter('length_scale').get_parameter_value().double_value
        noise_scale_param = self.get_parameter('noise_scale').get_parameter_value().double_value
        noise_w_scale_param = (
            self.get_parameter('noise_w_scale').get_parameter_value().double_value
        )
        use_cuda = self.get_parameter('cuda').get_parameter_value().bool_value
        sentence_silence = (
            self.get_parameter('sentence_silence').get_parameter_value().double_value
        )
        data_dirs = (
            self.get_parameter('data_dir').get_parameter_value().string_array_value
        )
        download_dir_str = (
            self.get_parameter('download_dir').get_parameter_value().string_value
        )
        debug = self.get_parameter('debug').get_parameter_value().bool_value
        auto_download_voices = self.get_parameter('auto_download_voices').get_parameter_value().bool_value
        # Use sentinel values to distinguish "not set" from a real value
        speaker: Optional[int] = None if speaker_param < 0 else speaker_param
        length_scale: Optional[float] = (
            None if length_scale_param < 0.0 else length_scale_param
        )
        noise_scale: Optional[float] = (
            None if noise_scale_param < 0.0 else noise_scale_param
        )
        noise_w_scale: Optional[float] = (
            None if noise_w_scale_param < 0.0 else noise_w_scale_param
        )

        logging.basicConfig(level=logging.DEBUG if debug else logging.INFO)

        #if not model:
        #    self.get_logger().fatal('Parameter "model" is required')
        #    raise RuntimeError('Parameter "model" is required')

        if not download_dir_str:
            download_dir_str = data_dirs[0]
        download_dir = Path(download_dir_str)
        download_dir.mkdir(parents=True, exist_ok=True)

        # Create data directories if they don't exist
        for data_dir in data_dirs:
            Path(data_dir).mkdir(parents=True, exist_ok=True)

        # Locate the model file
        default_model_id = None
        loaded_voices: Dict[str, PiperVoice] = {}
        default_voice = None
        model_path = None

        if model:
            model_path = Path(model)
            if not model_path.exists():
                voice_name = model
                for data_dir in data_dirs:
                    maybe_model_path = Path(data_dir) / f'{voice_name}.onnx'
                    _LOGGER.debug("Checking '%s'", maybe_model_path)
                    if maybe_model_path.exists():
                        model_path = maybe_model_path
                        break

            if not model_path.exists():
                raise ValueError(
                    f'Unable to find voice: {model_path} (use piper.download_voices)'
                )

            default_model_id = model_path.name.rstrip('.onnx')

            # Load voice
            default_voice = PiperVoice.load(model_path, use_cuda=use_cuda)
            loaded_voices: Dict[str, PiperVoice] = {default_model_id: default_voice}

        # Create Flask web server
        app = Flask(__name__)

        @app.route('/voices', methods=['GET'])
        def app_voices() -> Dict[str, Any]:
            """List downloaded voices."""
            voices_dict: Dict[str, Any] = {}
            config_paths: List[Path] = []
            if model_path:
                config_paths.append(Path(f'{model_path}.json'))

            for data_dir in data_dirs:
                for onnx_path in Path(data_dir).glob('*.onnx'):
                    config_path = Path(f'{onnx_path}.json')
                    if config_path.exists():
                        config_paths.append(config_path)

            for config_path in config_paths:
                model_id = config_path.name.rstrip('.onnx.json')
                if model_id in voices_dict:
                    continue
                with open(config_path, 'r', encoding='utf-8') as config_file:
                    voices_dict[model_id] = json.load(config_file)

            return voices_dict

        @app.route('/all-voices', methods=['GET'])
        def app_all_voices() -> Dict[str, Any]:
            """List all Piper voices from HuggingFace."""
            with urlopen(VOICES_JSON) as response:
                return json.load(response)

        @app.route('/download', methods=['GET'])
        def app_download_get() -> Dict[str, Any]:
            """Download a voice using query parameters."""
            model_id = request.args.get('voice')
            if not model_id:
                raise ValueError('voice is required')
            force_redownload = request.args.get('force_redownload', 'false').lower() == 'true'
            download_voice(model_id, download_dir, force_redownload=force_redownload)

            config_path = download_dir / f'{model_id}.onnx.json'
            if config_path.exists():
                with open(config_path, 'r', encoding='utf-8') as config_file:
                    return json.load(config_file)
            return {}

        @app.route('/download', methods=['POST'])
        def app_download() -> Dict[str, Any]:
            """Download a voice."""
            data = json.loads(request.data)
            model_id = data.get('voice')
            if not model_id:
                raise ValueError('voice is required')
            force_redownload = data.get('force_redownload', False)
            download_voice(model_id, download_dir, force_redownload=force_redownload)

            config_path = download_dir / f'{model_id}.onnx.json'
            if config_path.exists():
                with open(config_path, 'r', encoding='utf-8') as config_file:
                    return json.load(config_file)
            return {}

        @app.route('/', methods=['POST'])
        def app_synthesize() -> bytes:
            """Synthesize audio from text."""
            data = json.loads(request.data)
            text = data.get('text', '').strip()
            if not text:
                raise ValueError('No text provided')

            _LOGGER.debug(data)

            req_model_id = data.get('voice', default_model_id)
            voice = loaded_voices.get(req_model_id)
            if voice is None:
                for data_dir in data_dirs:
                    maybe_path = Path(data_dir) / f'{req_model_id}.onnx'
                    if maybe_path.exists():
                        _LOGGER.debug('Loading voice %s', req_model_id)
                        voice = PiperVoice.load(maybe_path, use_cuda=use_cuda)
                        loaded_voices[req_model_id] = voice
                        break

            if voice is None and auto_download_voices and req_model_id:
                try:
                    _LOGGER.info('Auto-downloading voice: %s', req_model_id)
                    download_voice(req_model_id, download_dir, force_redownload=False)
                    maybe_path = download_dir / f'{req_model_id}.onnx'
                    if maybe_path.exists():
                        _LOGGER.debug('Loading downloaded voice %s', req_model_id)
                        voice = PiperVoice.load(maybe_path, use_cuda=use_cuda)
                        loaded_voices[req_model_id] = voice
                except Exception as e:
                    _LOGGER.error('Failed to download voice %s: %s', req_model_id, e)

            if voice is None:
                _LOGGER.warning(
                    'Voice not found: %s. Using default voice.', req_model_id
                )
                voice = default_voice

            speaker_id: Optional[int] = data.get('speaker_id')
            if (voice.config.num_speakers > 1) and (speaker_id is None):
                req_speaker = data.get('speaker')
                if req_speaker:
                    speaker_id = voice.config.speaker_id_map.get(req_speaker)
                if speaker_id is None:
                    _LOGGER.warning(
                        "Speaker not found: '%s' in %s",
                        req_speaker,
                        voice.config.speaker_id_map.keys(),
                    )
                    speaker_id = speaker if speaker is not None else 0

            if (speaker_id is not None) and (
                speaker_id > voice.config.num_speakers
            ):
                speaker_id = 0

            syn_config = SynthesisConfig(
                speaker_id=speaker_id,
                length_scale=float(
                    data.get(
                        'length_scale',
                        (
                            length_scale
                            if length_scale is not None
                            else voice.config.length_scale
                        ),
                    )
                ),
                noise_scale=float(
                    data.get(
                        'noise_scale',
                        (
                            noise_scale
                            if noise_scale is not None
                            else voice.config.noise_scale
                        ),
                    )
                ),
                noise_w_scale=float(
                    data.get(
                        'noise_w_scale',
                        (
                            noise_w_scale
                            if noise_w_scale is not None
                            else voice.config.noise_w_scale
                        ),
                    )
                ),
            )

            _LOGGER.debug(
                "Synthesizing text: '%s' with config=%s", text, syn_config
            )
            with io.BytesIO() as wav_io:
                wav_file: wave.Wave_write = wave.open(wav_io, 'wb')
                with wav_file:
                    wav_params_set = False
                    for i, audio_chunk in enumerate(
                        voice.synthesize(text, syn_config)
                    ):
                        if not wav_params_set:
                            wav_file.setframerate(audio_chunk.sample_rate)
                            wav_file.setsampwidth(audio_chunk.sample_width)
                            wav_file.setnchannels(audio_chunk.sample_channels)
                            wav_params_set = True

                        if i > 0:
                            wav_file.writeframes(
                                bytes(
                                    int(
                                        voice.config.sample_rate
                                        * sentence_silence
                                        * 2
                                    )
                                )
                            )

                        wav_file.writeframes(audio_chunk.audio_int16_bytes)

                return wav_io.getvalue()

        self._app = app
        self._host = host
        self._port = port

    def start_server(self) -> None:
        """Start the Flask server in a background thread."""
        self._server_thread = threading.Thread(
            target=self._app.run,
            kwargs={'host': self._host, 'port': self._port},
            daemon=True,
        )
        self._server_thread.start()
        self.get_logger().info(
            f'Piper TTS server started on {self._host}:{self._port}'
        )


def ros_main(args: Any = None) -> None:
    """ROS2 entry point for PiperTtsServerNode."""
    rclpy.init(args=args)
    node = PiperTtsServerNode()
    node.start_server()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
