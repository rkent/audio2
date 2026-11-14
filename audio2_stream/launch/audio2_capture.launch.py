"""Launch audio2_capture_node, capturing from the default audio device."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start audio2_capture_node, with its parameters available as launch arguments."""
    arguments = [
        DeclareLaunchArgument(
            'device_name',
            default_value='default',
            description='Capture device: "default" for the system default; on Linux an ALSA '
                        'device name, on Windows part of a device name or its endpoint ID'),
        DeclareLaunchArgument(
            'sample_format',
            default_value='14',
            description='Device sample format, as a SampleFormat value (14 = float)'),
        DeclareLaunchArgument(
            'channels',
            default_value='2',
            description='Number of channels to capture'),
        DeclareLaunchArgument(
            'samplerate',
            default_value='48000',
            description='Capture sample rate in Hz'),
        DeclareLaunchArgument(
            'stream_queue_frames',
            default_value='480',
            description='Frames per audio chunk in a stream'),
        DeclareLaunchArgument(
            'audio_topic',
            default_value='audio_stream_chunks',
            description='Topic the captured audio chunks are published on'),
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='Start capturing as soon as the node starts'),
    ]

    capture_node = Node(
        package='audio2_stream',
        executable='audio2_capture_node',
        name='audio2_capture_node',
        # Explicit types, since launch otherwise parses values as YAML: a device named
        # "null" or "on" would not arrive as a string
        parameters=[{
            'device_name': ParameterValue(LaunchConfiguration('device_name'), value_type=str),
            'sample_format': ParameterValue(LaunchConfiguration('sample_format'), value_type=int),
            'channels': ParameterValue(LaunchConfiguration('channels'), value_type=int),
            'samplerate': ParameterValue(LaunchConfiguration('samplerate'), value_type=int),
            'stream_queue_frames': ParameterValue(
                LaunchConfiguration('stream_queue_frames'), value_type=int),
            'audio_topic': ParameterValue(LaunchConfiguration('audio_topic'), value_type=str),
            'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=bool),
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription(arguments + [capture_node])
