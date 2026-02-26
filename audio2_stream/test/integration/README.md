# Integration Tests for audio2_play_node

This directory contains integration tests for the `audio2_play_node` ROS 2 node.

## Overview

The integration tests verify the full functionality of the audio2_play_node including:

- **Topic Subscriptions**: Verifies the node subscribes to the correct topics
- **TTS Request Handling**: Tests text-to-speech request processing
- **File Playback**: Tests audio file playback functionality
- **Audio Chunk Streaming**: Tests receiving and processing audio chunk messages
- **Multiple Concurrent Streams**: Tests handling multiple audio streams simultaneously
- **Error Handling**: Tests graceful handling of invalid inputs

## Test Structure

```
test/integration/
├── README.md                                    # This file
├── test_audio2_play_node.py                    # Main integration test suite
├── launch/
│   └── test_audio2_play_node.launch.py         # Launch file for tests
└── test_fixtures/
    ├── generate_test_audio.py                  # Script to generate test audio files
    ├── test_audio.wav                          # Generated test file (1s, stereo)
    ├── test_audio_short.wav                    # Generated test file (0.5s, stereo)
    └── test_audio_mono.wav                     # Generated test file (0.5s, mono)
```

## Running the Tests

### Prerequisites

All tests run inside the Docker container. The integration tests use the ALSA "null" device to avoid hardware dependencies.

### Build with Tests

```bash
# Build the package with integration tests enabled
docker/run.sh kilted colcon build --packages-select audio2_stream --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install
```

### Run All Tests

```bash
# Run all tests (unit + integration)
docker/run.sh kilted colcon test --packages-select audio2_stream

# View test results
docker/run.sh kilted colcon test-result --verbose
```

### Run Only Integration Tests

```bash
# Run only integration tests
docker/run.sh kilted colcon test --packages-select audio2_stream --pytest-args -k integration

# View detailed output
docker/run.sh kilted colcon test-result --verbose
```

### Run Specific Test Cases

```bash
# Run a specific test method
docker/run.sh kilted colcon test --packages-select audio2_stream --pytest-args -k test_tts_request_message_accepted

# Run with verbose output
docker/run.sh kilted colcon test --packages-select audio2_stream --pytest-args "-v -k integration"
```

## Test Cases

### test_node_starts_successfully
Verifies that the audio2_play_node launches successfully and is discoverable via ROS 2 node introspection.

### test_tts_request_topic_subscription
Checks that the node subscribes to the `/tts_request` topic.

### test_tts_request_message_accepted
Publishes a TTS request message and verifies the node processes it without crashing.

### test_play_file_local_topic_subscription
Checks that the node subscribes to the `/play_file_local` topic.

### test_play_file_local_message_accepted
Publishes a file playback request and verifies it's handled correctly.

### test_audio_stream_chunks_topic_subscription
Checks that the node subscribes to the `/audio_stream_chunks` topic.

### test_audio_chunk_streaming
Sends multiple audio chunks for a single stream and verifies they're processed.

### test_multiple_concurrent_streams
Tests handling of multiple interleaved audio streams with different UUIDs.

### test_invalid_file_path_handled_gracefully
Verifies the node doesn't crash when given an invalid file path.

## Generating Test Fixtures

The test audio files are generated automatically when needed, but you can regenerate them manually:

```bash
cd test/integration/test_fixtures
python3 generate_test_audio.py
```

This creates:
- `test_audio.wav`: 1 second, 48kHz, stereo, 440Hz sine wave
- `test_audio_short.wav`: 0.5 seconds, 48kHz, stereo, 880Hz sine wave
- `test_audio_mono.wav`: 0.5 seconds, 48kHz, mono, 440Hz sine wave

## Troubleshooting

### Tests Fail to Find Node

If tests report that `audio2_play_node_test` is not found:
- Ensure the package is built: `docker/run.sh kilted colcon build --packages-select audio2_stream`
- Check that the node is in the install space: `ls install/audio2_stream/lib/audio2_stream/`

### ALSA Errors

The tests use the ALSA "null" device which should always work. If you see ALSA-related errors:
- Verify that `alsa_device_name` parameter is set to "null" in the launch file
- Check Docker container has ALSA support

### Timeout Errors

If tests timeout:
- Increase the `TIMEOUT` value in `CMakeLists.txt` (currently 60 seconds)
- Check if the node is hanging during initialization
- Run with verbose output to see where it's stuck

### Topic Connection Issues

If publishers don't connect to the node:
- Verify the topic names match: `/tts_request`, `/play_file_local`, `/audio_stream_chunks`
- Check QoS settings match between publisher and subscriber
- Increase the timeout in `_wait_for_subscriber()` method

## CI/CD Integration

These tests are designed to run in CI/CD pipelines:

```yaml
# Example GitHub Actions workflow snippet
- name: Build ROS 2 Packages
  run: docker/run.sh kilted colcon build --packages-select audio2_stream

- name: Run Tests
  run: docker/run.sh kilted colcon test --packages-select audio2_stream

- name: Show Test Results
  run: docker/run.sh kilted colcon test-result --verbose
  if: always()
```

## Adding New Tests

To add new integration tests:

1. Add a new test method to `test_audio2_play_node.py`:
   ```python
   def test_my_new_feature(self):
       """Test description here."""
       # Test implementation
   ```

2. Follow the naming convention: `test_<feature_name>`

3. Use the helper methods:
   - `_wait_for_subscriber()`: Wait for topic connections
   - `_ensure_test_fixtures()`: Ensure test files exist
   - `rclpy.spin_once()`: Process ROS callbacks

4. Rebuild and run tests:
   ```bash
   docker/run.sh kilted colcon build --packages-select audio2_stream
   docker/run.sh kilted colcon test --packages-select audio2_stream
   ```

## Coverage Analysis

To analyze test coverage:

```bash
# Build with coverage flags
docker/run.sh kilted colcon build --packages-select audio2_stream \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="--coverage"

# Run tests
docker/run.sh kilted colcon test --packages-select audio2_stream

# Generate coverage report
docker/run.sh kilted bash -c "cd build/audio2_stream && gcov *.gcno"
```

## Related Documentation

- [Unit Tests](../TEST_README.md): Information about unit tests
- [Node Documentation](../../doc/Nodes/audio2_play_node.md): audio2_play_node details
- [ROS 2 launch_testing](https://github.com/ros2/launch/tree/rolling/launch_testing): Official documentation
