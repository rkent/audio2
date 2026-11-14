# Audio2 Stream Testing Framework

This document describes the testing infrastructure for the audio2_stream package.

## Overview

The audio2_stream package now includes comprehensive unit tests using GTest and GMock. The codebase has been refactored to support dependency injection, enabling testing without hardware dependencies.

## Architecture Changes

### Dependency Injection for Audio Device Operations

Audio device operations are abstracted behind the `IAudioDeviceProxy` interface, with one
backend per platform:

- **IAudioDeviceProxy**: Interface defining device operations (preopen, open, close, read, write)
- **AlsaProxyImpl**: Linux backend that wraps ALSA
- **WasapiProxyImpl**: Windows backend that wraps WASAPI
- **make_default_device_proxy()**: Creates the backend for the current platform

### Device Terminal Classes

- **DeviceTerminal**: Accepts `std::unique_ptr<IAudioDeviceProxy>` in constructor
- **DeviceSink**: Accepts an optional proxy; uses `make_default_device_proxy()` if none provided
- **DeviceSource**: Accepts an optional proxy; uses `make_default_device_proxy()` if none provided

## Test Structure

### Test Files

```
repos/audio2/audio2_stream/test/
├── test_audio_stream.cpp        # Unit tests for AudioStream queue and format conversion
├── test_device_terminal.cpp     # DeviceSink/DeviceSource against the platform backend
└── test_wasapi.cpp              # WASAPI backend tests (Windows only)
```

### Test Coverage

#### test_audio_stream.cpp
- AudioStream construction and destruction
- Queue push/pop operations
- Queue full/empty behavior
- Shutdown mechanism
- SndFileSource file opening and reading
- Format conversion functions
- Concurrent queue access (thread safety)

#### test_device_terminal.cpp
- DeviceSink open success/failure
- DeviceSink audio data writing
- DeviceSource audio data reading
- Source-to-sink streaming

Uses the ALSA `null` device on Linux. On Windows it uses the default WASAPI endpoint, and
tests needing an endpoint are skipped on machines without one.

#### test_wasapi.cpp (Windows only)
- Sample format mapping
- Unknown device errors that list the available devices
- Playback with sample rate conversion, pre-open, and drain on close
- Capture, including leftover packet handling and the shutdown flag

## Building with Tests

### Prerequisites

The following test dependencies are required (already in package.xml):
- `ament_cmake_gtest`
- `ament_cmake_gmock`

### Build Commands

```bash
# Build all packages
colcon build

# Build only audio2_stream with tests
colcon build --packages-select audio2_stream

# Build with debug symbols and thread sanitizer
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Running Tests

```bash
# Run all tests for audio2_stream
colcon test --packages-select audio2_stream

# View test results
colcon test-result --verbose

# Run tests manually (after building)
./build/audio2_stream/test_audio_stream
./build/audio2_stream/test_device_terminal
```

## Thread Safety Validation

The build includes thread sanitizer support when building in Debug mode:

```bash
# Build with thread sanitizer
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select audio2_stream

# Run tests with thread sanitizer active
colcon test --packages-select audio2_stream
```

Thread sanitizer will detect:
- Data races
- Deadlocks
- Use of uninitialized memory in threaded contexts
- Improper synchronization

## Adding New Tests

### For Non-Hardware Tests

Add tests to `test_audio_stream.cpp`:

```cpp
TEST_F(AudioStreamTest, YourTestName)
{
    // Arrange
    auto stream = std::make_unique<AudioStream>(...);

    // Act
    // ... perform operations

    // Assert
    EXPECT_EQ(expected, actual);
}
```

### For Audio Device Tests

Add tests to `test_device_terminal.cpp`. A test can also inject its own `IAudioDeviceProxy`
implementation (for example a GMock mock) into `DeviceSink`/`DeviceSource`:

```cpp
TEST_F(DeviceTerminalTest, YourDeviceTest)
{
    REQUIRE_TEST_DEVICE(AudioDirection::Playback);
    auto sink = std::make_unique<DeviceSink>(TEST_DEVICE, make_default_device_proxy());
    auto stream = std::make_unique<AudioStream>(nullptr, std::move(sink), "test_stream", 100);
    StreamShutdownGuard guard{stream.get()};  // shuts down even if an ASSERT fails

    // Test behavior
    // ...
}
```

## Continuous Integration

For CI/CD pipelines, tests can be run without hardware:

```bash
# In CI environment
colcon build --packages-select audio2_stream
colcon test --packages-select audio2_stream
colcon test-result --verbose
```

On Linux, device tests use the ALSA `null` device, so they pass without audio hardware.
On Windows, tests that need an audio endpoint are skipped when none is present.

## Debugging Tests

### Run Individual Tests

```bash
# Run with GTest filters
./build/audio2_stream/test_audio_stream --gtest_filter="AudioStreamTest.QueueOperations"

# Run with verbose output
./build/audio2_stream/test_audio_stream --gtest_verbose=1
```

### Debug with GDB

```bash
gdb ./build/audio2_stream/test_audio_stream
(gdb) break test_audio_stream.cpp:42
(gdb) run
```

### Memory Checking with Valgrind

```bash
valgrind --leak-check=full ./build/audio2_stream/test_audio_stream
```

## Future Enhancements

Potential test improvements:

1. **Integration tests** with snd-aloop (Linux virtual audio device)
2. **Benchmark tests** for queue performance
3. **Fuzzing** for audio format conversions
4. **Property-based testing** for audio transformations
5. **ROS2 node integration tests** using launch_testing

## Troubleshooting

### Build Failures

If builds fail with missing symbols:
```bash
# Clean and rebuild
colcon build --packages-select audio2_stream --cmake-clean-cache
```

### Test Failures

If tests fail:
1. Check test output: `colcon test-result --verbose`
2. Verify ROS2 is initialized: Tests call `rclcpp::init()` in SetUp()
3. For thread sanitizer warnings, review synchronization logic

### Mock Expectations Not Met

If GMock expectations fail:
- Verify the mock setup matches actual usage
- Check call counts: Use `.Times()` matchers
- Review thread timing: Add synchronization in tests if needed
