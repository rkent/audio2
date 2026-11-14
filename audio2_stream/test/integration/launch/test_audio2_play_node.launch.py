"""Launch file for audio2_play_node integration tests."""

import array
import io
import math
import os
import struct
import sys
import time
import unittest
import wave

from audio2_stream_msgs.msg import AudioChunk, StreamHeader

import launch

from launch_ros.actions import Node as LaunchNode

import launch_testing
from launch_testing.actions import ReadyToTest

import rclpy
from rclpy.node import Node

from unique_identifier_msgs.msg import UUID


def make_wav_chunk(samplerate, channels, frames):
    """Return a 16-bit PCM WAV file of a sine wave as bytes, like one AudioChunk's data."""
    frame_samples = []
    for frame in range(frames):
        value = int(8000 * math.sin(2 * math.pi * 440 * frame / samplerate))
        frame_samples.append(struct.pack('<h', value) * channels)
    buffer = io.BytesIO()
    with wave.open(buffer, 'wb') as wav:
        wav.setnchannels(channels)
        wav.setsampwidth(2)
        wav.setframerate(samplerate)
        wav.writeframes(b''.join(frame_samples))
    return buffer.getvalue()


def generate_test_description():
    """
    Generate the launch description for the integration test.

    Launches audio2_play_node with a null ALSA device to avoid hardware
    dependencies.
    """
    # Launch the node under test with null ALSA device
    node_under_test = LaunchNode(
        package='audio2_stream',
        executable='audio2_play_node',
        name='audio2_play_node_test',
        parameters=[{
            'device_name': 'null',  # ALSA null device; on Windows no such device exists
            'sample_format': 14,  # SampleFormat::Float
            'stream_queue_frames': 480
        }],
        output='screen',
        emulate_tty=True,
    )

    return launch.LaunchDescription([
        node_under_test,
        # Tells launch when to start the tests
        ReadyToTest()
    ]), {'node_under_test': node_under_test}


class TestAudio2PlayNodeBasic(unittest.TestCase):
    """Basic integration tests for audio2_play_node in the launch file."""

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
        self.test_node = Node('test_audio2_play_basic')

    def tearDown(self):
        """Clean up test node."""
        self.test_node.destroy_node()

    def test_node_exists(self):
        """Test that the audio2_play_node is discoverable."""
        import time
        # Give node time to initialize
        time.sleep(2.0)

        # Check that node exists
        node_names = self.test_node.get_node_names()
        self.assertIn('audio2_play_node_test', node_names,
                      'audio2_play_node should be running')

    @unittest.skipIf(sys.platform == 'win32', 'needs the ALSA null device')
    def test_chunk_stream_opens_device_in_its_format(self, proc_output, node_under_test):
        """Test that a stream of audio chunks opens the device at the chunks' rate and channels."""
        publisher = self.test_node.create_publisher(AudioChunk, 'audio_stream_chunks', 10)
        deadline = time.time() + 10.0
        while publisher.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.1)
        self.assertGreater(publisher.get_subscription_count(), 0,
                           'audio2_play_node should subscribe to audio_stream_chunks')

        # Neither format is the device's preferred 24000 Hz mono
        for samplerate, channels in ((48000, 2), (16000, 1)):
            uuid = UUID(uuid=list(os.urandom(16)))
            for sequence in range(5):
                message = AudioChunk()
                message.header.uuid = uuid
                message.header.sequence = sequence
                message.data = array.array('B', make_wav_chunk(samplerate, channels, 480))
                publisher.publish(message)
                time.sleep(0.01)
            end_of_stream = AudioChunk()
            end_of_stream.header.uuid = uuid
            end_of_stream.header.sequence = StreamHeader.SEQUENCE_EOS
            publisher.publish(end_of_stream)

            # Printed by the node's library to stdout; assertWaitFor reads stderr by default
            proc_output.assertWaitFor(
                f'DeviceTerminal::open called with params: device=null, '
                f'channels={channels}, samplerate={samplerate}',
                process=node_under_test, timeout=10, stream='stdout')


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Test that the node exits cleanly after shutdown."""

    def test_exit_code(self, proc_info, node_under_test):
        """Verify that the node exits with code 0 or SIGINT."""
        from launch_testing.asserts import assertExitCodes
        # Accept both 0 (clean exit) and -2 (SIGINT) as valid
        allowed = [0, -2]
        if sys.platform == 'win32':
            # Windows cannot deliver SIGINT, so launch terminates the process with exit code 1
            allowed.append(1)
        assertExitCodes(proc_info, allowed, process=node_under_test)
