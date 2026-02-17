# alsa_gain_node

## Description

ROS2 node for monitoring and controlling ALSA mixer volume levels. This node publishes the current gain (volume) and mute status of an ALSA mixer control at a regular interval and responds to requests to change the gain or mute state.

## Publishers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `alsa_gain` | `alsa_gain_msgs/msg/AlsaGain` | Publishes current volume percentage and mute status |

## Subscribers

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `alsa_gain_set` | `alsa_gain_msgs/msg/AlsaGain` | Receives requests to set volume percentage or mute status |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `control` | string | `"Master"` | Name of the ALSA mixer control to monitor/control |
| `device` | string | `"default"` | ALSA device identifier |
| `publish_rate` | double | `5.0` | Rate (in seconds) at which to publish gain information |

## Example Usage

```bash
# Run with default parameters (Master control on default device)
ros2 run alsa_gain alsa_gain_node

# Run with custom mixer control
ros2 run alsa_gain alsa_gain_node --ros-args \
  -p control:="PCM"

# Run with custom device and faster publish rate
ros2 run alsa_gain alsa_gain_node --ros-args \
  -p device:="hw:0" \
  -p publish_rate:=2.0
```

## Notes

- The node will only respond to `alsa_gain_set` messages that match the configured `control` and `device` parameters
- Volume changes are only applied if they differ from the current value to minimize ALSA calls
- The node uses a separate thread to monitor mixer events and can publish changes immediately when they occur
