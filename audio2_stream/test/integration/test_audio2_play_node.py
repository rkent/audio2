"""
Integration tests for audio2_play_node.

These tests verify the ROS 2 node's behavior including:
- Topic subscription and message handling
- TTS request processing
- File playback requests
- Audio chunk streaming
- Parameter configuration
"""

import math
import os
from pathlib import Path
import struct
import time
import unittest

from audio2_stream_msgs.msg import (
    AudioChunk, PlayFile, TtsProvider, TtsRequest
)

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class TestAudio2PlayNode(unittest.TestCase):
    """Integration tests for audio2_play_node."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 for testing."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 after all tests."""
        rclpy.shutdown()

    def setUp(self):
        """Create test node for each test."""
        self.test_node = Node('test_audio2_play_node')

        # Generate test audio file if needed
        self.test_fixtures_path = self._get_test_fixtures_path()
        self._ensure_test_fixtures()

    def tearDown(self):
        """Clean up test node."""
        self.test_node.destroy_node()

    def _get_test_fixtures_path(self):
        """Get the path to test fixtures directory."""
        # Construct path relative to this test file
        test_dir = Path(__file__).parent
        fixtures_path = test_dir / 'test_fixtures'

        if fixtures_path.exists():
            return str(fixtures_path)

        # Fallback to /tmp for generated files
        return '/tmp'

    def _ensure_test_fixtures(self):
        """Ensure test audio files exist."""
        test_audio_path = os.path.join(
            self.test_fixtures_path,
            'test_audio.wav',
        )

        if not os.path.exists(test_audio_path):
            # Generate test audio file
            self._generate_test_audio(test_audio_path)

    def _generate_test_audio(self, filepath):
        """Generate a simple test audio file."""
        sample_rate = 48000
        duration = 0.5  # seconds
        frequency = 440.0  # A4
        channels = 2
        amplitude = 0.3

        num_samples = int(sample_rate * duration)

        with open(filepath, 'wb') as f:
            # WAV header
            f.write(b'RIFF')
            f.write(struct.pack('<I', 0))  # File size placeholder
            f.write(b'WAVE')

            # fmt chunk
            f.write(b'fmt ')
            f.write(struct.pack('<I', 16))
            f.write(struct.pack('<H', 1))  # PCM
            f.write(struct.pack('<H', channels))
            f.write(struct.pack('<I', sample_rate))
            f.write(struct.pack('<I', sample_rate * channels * 2))
            f.write(struct.pack('<H', channels * 2))
            f.write(struct.pack('<H', 16))  # 16-bit

            # data chunk
            f.write(b'data')
            data_size = num_samples * channels * 2
            f.write(struct.pack('<I', data_size))

            # Generate sine wave
            for i in range(num_samples):
                value = amplitude * math.sin(
                    2.0 * math.pi * frequency * i / sample_rate)
                pcm = int(value * 32767)
                pcm = max(-32768, min(32767, pcm))
                for _ in range(channels):
                    f.write(struct.pack('<h', pcm))

            # Update file size
            file_size = f.tell()
            f.seek(4)
            f.write(struct.pack('<I', file_size - 8))

    def _wait_for_subscriber(self, publisher, timeout=5.0):
        """
        Wait until the publisher has at least one subscriber.

        Returns True when a subscriber connects before timeout, otherwise
        returns False.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if publisher.get_subscription_count() > 0:
                return True
            time.sleep(0.1)
            rclpy.spin_once(self.test_node, timeout_sec=0)
        return False

    def test_node_starts_successfully(self):
        """Test that the audio2_play_node starts and is discoverable."""
        # Give node time to start
        time.sleep(1.0)

        # Check that node exists
        node_names = self.test_node.get_node_names()
        self.assertIn('audio2_play_node_test', node_names,
                      'audio2_play_node should be running')

    def test_tts_request_topic_subscription(self):
        """Test that node subscribes to tts_request topic."""
        tts_pub = self.test_node.create_publisher(
            TtsRequest,
            '/tts_request',
            10
        )

        # Wait for node to subscribe
        self.assertTrue(
            self._wait_for_subscriber(tts_pub, timeout=5.0),
            'Node should subscribe to /tts_request topic'
        )

        self.test_node.destroy_publisher(tts_pub)

    def test_tts_request_message_accepted(self):
        """Test that node accepts and processes TTS request messages."""
        tts_pub = self.test_node.create_publisher(
            TtsRequest,
            '/tts_request',
            10
        )

        self.assertTrue(self._wait_for_subscriber(tts_pub, timeout=5.0))

        # Create and publish TTS request
        msg = TtsRequest()
        msg.text = 'Hello integration test'
        msg.provider = TtsProvider()
        msg.provider.name = 'piper'
        msg.provider.voice = 'en_US-joe-medium'
        msg.provider.model = ''
        msg.provider.format = ''

        # Publish message
        tts_pub.publish(msg)

        # Give node time to process
        for _ in range(10):
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # More sophisticated test would monitor node state or use a service

        self.test_node.destroy_publisher(tts_pub)

    def test_play_file_local_topic_subscription(self):
        """Test that node subscribes to play_file_local topic."""
        play_pub = self.test_node.create_publisher(
            PlayFile,
            '/play_file_local',
            10
        )

        self.assertTrue(
            self._wait_for_subscriber(play_pub, timeout=5.0),
            'Node should subscribe to /play_file_local topic'
        )

        self.test_node.destroy_publisher(play_pub)

    def test_play_file_local_message_accepted(self):
        """Test that node accepts file playback requests."""
        play_pub = self.test_node.create_publisher(
            PlayFile,
            '/play_file_local',
            10
        )

        self.assertTrue(self._wait_for_subscriber(play_pub, timeout=5.0))

        # Create and publish play file request
        msg = PlayFile()
        test_audio = os.path.join(self.test_fixtures_path, 'test_audio.wav')
        msg.path = test_audio
        msg.play_type = 0  # Assuming 0 is a valid play type

        play_pub.publish(msg)

        # Give node time to process
        for _ in range(10):
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
        self.test_node.destroy_publisher(play_pub)

    def test_audio_stream_chunks_topic_subscription(self):
        """Test that node subscribes to audio_stream_chunks topic."""
        # Use QoS matching what the node expects
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        chunk_pub = self.test_node.create_publisher(
            AudioChunk,
            '/audio_stream_chunks',
            qos_profile
        )

        self.assertTrue(
            self._wait_for_subscriber(chunk_pub, timeout=5.0),
            'Node should subscribe to /audio_stream_chunks topic'
        )

        self.test_node.destroy_publisher(chunk_pub)

    def test_audio_chunk_streaming(self):
        """Test that node receives and processes audio chunk messages."""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        chunk_pub = self.test_node.create_publisher(
            AudioChunk,
            '/audio_stream_chunks',
            qos_profile
        )

        self.assertTrue(self._wait_for_subscriber(chunk_pub, timeout=5.0))

        # Send multiple audio chunks
        stream_uuid = 'test-integration-stream-001'
        total_chunks = 5

        for chunk_num in range(total_chunks):
            msg = AudioChunk()
            msg.uuid = stream_uuid
            msg.chunk_number = chunk_num
            msg.total_chunks = total_chunks
            msg.sample_rate = 48000
            msg.channels = 2
            msg.format = 'float32'

            # Create dummy audio data:
            # 480 frames * 2 channels * 4 bytes per float
            frame_count = 480
            bytes_per_sample = 4
            data_size = frame_count * msg.channels * bytes_per_sample
            msg.data = bytes([0] * data_size)

            chunk_pub.publish(msg)

            # Spin to process
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            time.sleep(0.05)  # Small delay between chunks

        # Give time for final processing
        for _ in range(5):
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        self.test_node.destroy_publisher(chunk_pub)

    def test_multiple_concurrent_streams(self):
        """Test handling of multiple audio streams simultaneously."""
        # This test publishes chunks for multiple different stream UUIDs
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        chunk_pub = self.test_node.create_publisher(
            AudioChunk,
            '/audio_stream_chunks',
            qos_profile
        )

        self.assertTrue(self._wait_for_subscriber(chunk_pub, timeout=5.0))

        # Create multiple streams
        stream_uuids = [
            'concurrent-stream-001',
            'concurrent-stream-002',
            'concurrent-stream-003'
        ]

        chunks_per_stream = 3

        # Interleave chunks from different streams
        for chunk_num in range(chunks_per_stream):
            for stream_uuid in stream_uuids:
                msg = AudioChunk()
                msg.uuid = stream_uuid
                msg.chunk_number = chunk_num
                msg.total_chunks = chunks_per_stream
                msg.sample_rate = 48000
                msg.channels = 2
                msg.format = 'float32'
                msg.data = bytes([0] * 480 * 2 * 4)

                chunk_pub.publish(msg)
                rclpy.spin_once(self.test_node, timeout_sec=0.05)

        # Give time for processing
        for _ in range(10):
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        self.test_node.destroy_publisher(chunk_pub)

    def test_invalid_file_path_handled_gracefully(self):
        """Test that invalid file paths don't crash the node."""
        play_pub = self.test_node.create_publisher(
            PlayFile,
            '/play_file_local',
            10
        )

        self.assertTrue(self._wait_for_subscriber(play_pub, timeout=5.0))

        # Publish request with non-existent file
        msg = PlayFile()
        msg.path = '/nonexistent/path/to/audio.wav'
        msg.play_type = 0

        play_pub.publish(msg)

        # Node should handle gracefully without crashing
        for _ in range(10):
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Verify node is still running
        node_names = self.test_node.get_node_names()
        self.assertIn('audio2_play_node_test', node_names,
                      'Node should still be running after invalid file')

        self.test_node.destroy_publisher(play_pub)


if __name__ == '__main__':
    unittest.main()
