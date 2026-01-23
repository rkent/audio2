# Audio2 Stream Testing Framework

This document describes the testing infrastructure for the audio2_stream package.

## Overview

The audio2_stream package now includes comprehensive unit tests using GTest and GMock. The codebase has been refactored to support dependency injection, enabling testing without hardware dependencies.

## Architecture Changes

### Dependency Injection for ALSA Operations

The ALSA hardware operations have been abstracted behind the `IAlsaDevice` interface:

- **IAlsaDevice**: Interface defining ALSA device operations (open, close, read, write)
- **AlsaDeviceImpl**: Real implementation that wraps actual ALSA hardware
- **MockAlsaDevice**: GMock-based mock for unit testing

### Refactored Classes

- **AlsaTerminal**: Now accepts `std::unique_ptr<IAlsaDevice>` in constructor
- **AlsaSink**: Accepts optional IAlsaDevice; creates AlsaDeviceImpl if none provided
- **AlsaSource**: Accepts optional IAlsaDevice; creates AlsaDeviceImpl if none provided

## Test Structure

### Test Files

```
repos/audio2/audio2_stream/test/
├── MockAlsaDevice.hpp           # GMock implementation of IAlsaDevice
├── test_audio_stream.cpp        # Unit tests for AudioStream queue and format conversion
└── test_alsa_mocked.cpp         # Unit tests for ALSA operations with mocking
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

#### test_alsa_mocked.cpp
- AlsaSink open success/failure
- AlsaSink audio data writing
- AlsaSource audio data reading
- Error handling for write failures
- Thread synchronization with mock devices

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
./build/audio2_stream/test_alsa_mocked
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

### For ALSA-Related Tests

Add tests to `test_alsa_mocked.cpp`:

```cpp
TEST_F(AlsaMockedTest, YourMockedTest)
{
    // Create mock device with expectations
    auto mock_device = std::make_unique<MockAlsaDevice>();
    EXPECT_CALL(*mock_device, write(_, _, _, _, _))
        .WillOnce(Return(samples));
    
    // Create AlsaSink/AlsaSource with mock
    auto sink = std::make_unique<AlsaSink>(..., std::move(mock_device));
    
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

All ALSA-dependent tests use mocks, so they will pass without audio hardware.

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
