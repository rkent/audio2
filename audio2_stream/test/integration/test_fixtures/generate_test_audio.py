#!/usr/bin/env python3
"""
Generate test audio fixture for integration tests.

Creates a simple WAV file with a sine wave tone.
"""

import math
import os
import struct


def generate_sine_wave_wav(
    filename,
    duration=1.0,
    sample_rate=48000,
    frequency=440.0,
    channels=2,
    amplitude=0.5,
):
    """
    Generate a WAV file containing a sine wave tone.

    The output is 16-bit PCM audio with the requested duration, sample rate,
    frequency, channel count, and amplitude.
    """
    num_samples = int(sample_rate * duration)

    # WAV file header
    with open(filename, 'wb') as f:
        # RIFF header
        f.write(b'RIFF')
        # File size (will update later)
        f.write(struct.pack('<I', 0))
        f.write(b'WAVE')

        # fmt subchunk
        f.write(b'fmt ')
        f.write(struct.pack('<I', 16))  # Subchunk size
        f.write(struct.pack('<H', 1))   # Audio format (1 = PCM)
        f.write(struct.pack('<H', channels))  # Number of channels
        f.write(struct.pack('<I', sample_rate))  # Sample rate
        byte_rate = sample_rate * channels * 2  # 2 bytes per sample (16-bit)
        f.write(struct.pack('<I', byte_rate))  # Byte rate
        block_align = channels * 2
        f.write(struct.pack('<H', block_align))  # Block align
        f.write(struct.pack('<H', 16))  # Bits per sample

        # data subchunk
        f.write(b'data')
        data_size = num_samples * channels * 2
        f.write(struct.pack('<I', data_size))

        # Generate and write audio samples
        for i in range(num_samples):
            # Generate sine wave sample
            sample_value = amplitude * math.sin(
                2.0 * math.pi * frequency * i / sample_rate
            )
            # Convert to 16-bit PCM
            pcm_value = int(sample_value * 32767)
            # Clamp to 16-bit range
            pcm_value = max(-32768, min(32767, pcm_value))

            # Write for each channel
            for _ in range(channels):
                f.write(struct.pack('<h', pcm_value))

        # Update file size in header
        file_size = f.tell()
        f.seek(4)
        f.write(struct.pack('<I', file_size - 8))


if __name__ == '__main__':
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Generate test audio files
    test_audio = os.path.join(script_dir, 'test_audio.wav')
    generate_sine_wave_wav(test_audio, duration=1.0, frequency=440.0)
    print(f'Generated: {test_audio}')

    # Generate a shorter test file
    short_audio = os.path.join(script_dir, 'test_audio_short.wav')
    generate_sine_wave_wav(short_audio, duration=0.5, frequency=880.0)
    print(f'Generated: {short_audio}')

    # Generate a mono test file
    mono_audio = os.path.join(script_dir, 'test_audio_mono.wav')
    generate_sine_wave_wav(mono_audio, duration=0.5, channels=1)
    print(f'Generated: {mono_audio}')

    print('Test fixtures generated successfully!')
