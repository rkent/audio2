# alsa_gain_node

## Node Name
`alsa_gain_node`

## Description
The alsa_gain_node monitors and controls ALSA mixer volume settings. It publishes the current volume and mute status periodically and can receive commands to adjust the volume or mute state. The node runs a separate thread to monitor mixer changes and uses a guard condition to trigger immediate publishing when changes occur.

## Subscriptions

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `alsa_gain_set` | `alsa_gain_msgs/msg/AlsaGain` | Receives commands to set volume percentage and mute state |

## Publishers

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `alsa_gain` | `alsa_gain_msgs/msg/AlsaGain` | Publishes current volume percentage and mute status |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `control` | string | `"Master"` | ALSA mixer control name to monitor/control (e.g., "Master", "PCM", "Speaker") |
| `device` | string | `"default"` | ALSA device identifier to use |
| `publish_rate` | double | `5.0` | Rate in seconds at which to publish gain information periodically |

## Message Format

The `AlsaGain` message contains:
- `control`: The ALSA mixer control name
- `device`: The ALSA device name
- `percent`: Array of volume percentages (one per channel)
- `muted`: Boolean indicating mute status

## Example Usage

```bash
# Run with default parameters (Master control on default device)
ros2 run alsa_gain alsa_gain_node

# Run with custom control and device
ros2 run alsa_gain alsa_gain_node --ros-args \
  -p control:="PCM" \
  -p device:="hw:0"

# Run with custom publish rate
ros2 run alsa_gain alsa_gain_node --ros-args \
  -p control:="Speaker" \
  -p publish_rate:=2.0

# Example: Set volume using ros2 topic
ros2 topic pub /alsa_gain_set alsa_gain_msgs/msg/AlsaGain \
  "{control: 'Master', device: 'default', percent: [75], muted: false}"
```

## Notes
- The node runs a separate thread (`MixerThread`) to monitor ALSA mixer changes
- When the mixer state changes (detected by the mixer thread), the node publishes immediately via a guard condition
- Additionally, the node publishes periodically at the rate specified by `publish_rate`
- When receiving set commands, the node only applies changes if the requested control and device match the configured parameters
- Volume changes are only applied if they differ from the current state to minimize unnecessary ALSA operations
- The node supports both single-channel and multi-channel audio devices
- If a single volume value is provided in a set command for a multi-channel device, it applies that value to all channels
