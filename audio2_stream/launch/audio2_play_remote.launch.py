"""Launch audio2_play_remote_node, which streams sound files over topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start audio2_play_remote_node, with its parameters available as launch arguments."""
    arguments = [
        DeclareLaunchArgument(
            'stream_queue_frames',
            default_value='480',
            description='Frames per audio chunk in a stream'),
    ]

    play_remote_node = Node(
        package='audio2_stream',
        executable='audio2_play_remote_node',
        name='audio2_play_remote_node',
        parameters=[{
            'stream_queue_frames': ParameterValue(
                LaunchConfiguration('stream_queue_frames'), value_type=int),
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription(arguments + [play_remote_node])
