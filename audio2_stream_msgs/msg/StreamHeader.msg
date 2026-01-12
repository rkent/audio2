# This header is sent with every audio chunk.

# How to handle this stream with respect to other streams.
uint8 HANDLE_QUEUE=0   # Queue behind any existing streams (default).
uint8 HANDLE_MUTE=1    # Mute any existing streams while this stream plays.
uint8 HANDLE_MIX=2     # Mix with any existing streams.
uint8 HANDLE_CANCEL=3  # Cancel any existing streams and play this one.
uint8 HANDLE_IGNORE=4  # Ignore (don't play) this stream if others are playing.
uint8 handle 0

# Description of this stream, for example filename of a sound effect, or "microphone".
string description

# Volume to apply at sink
float32 volume

# UUID to identify this stream. For example, the same file played multiple times
# would have a different UUID for each playback.
unique_identifier_msgs/UUID uuid

# Sequence of this audio chunk in the stream identified by the uuid.
uint32 SEQUENCE_EOS=0xFFFFFFFF  # End of stream marker.
uint32 sequence

# ROS2 timestamp when this chunk started. For a capture, this would be the 
# time this audio was captured. For playback, this would be when this message was sent.
builtin_interfaces/Time chunk_start_time
