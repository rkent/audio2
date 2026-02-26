"""Launch file for audio2_play_node integration tests."""

import unittest

import launch

from launch_ros.actions import Node as LaunchNode

import launch_testing
from launch_testing.actions import ReadyToTest

import rclpy
from rclpy.node import Node

# from audio2_stream_msgs.msg import (
#    AudioChunk, PlayFile, TtsProvider, TtsRequest
# )


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
            'alsa_device_name': 'null',  # Use ALSA null device for testing
            'alsa_format': 14,  # SND_PCM_FORMAT_FLOAT
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


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Test that the node exits cleanly after shutdown."""

    def test_exit_code(self, proc_info, node_under_test):
        """Verify that the node exits with code 0 or SIGINT."""
        from launch_testing.asserts import assertExitCodes
        # Accept both 0 (clean exit) and -2 (SIGINT) as valid
        assertExitCodes(proc_info, [0, -2], process=node_under_test)
