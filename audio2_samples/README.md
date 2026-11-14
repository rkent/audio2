# audio2_samples

Short audio files for testing and demonstrating the audio2 packages. Every file is
public domain under [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/), so it can be
used and redistributed without attribution. Each folder has its own license file.

## Contents

### `test_formats/`

One 1.34 s clip in a range of sample rates and encodings, for exercising format handling.
All are mono 16-bit unless noted.

| File | Format |
|---|---|
| `test.wav` | WAV PCM, 48000 Hz |
| `test_8000.wav`, `test_24000.wav`, `test_26000.wav`, `test_44100.wav` | WAV PCM at the rate in the name |
| `test_stereo.wav` | WAV PCM, 48000 Hz, stereo (both channels identical) |
| `test.flac` | FLAC, 48000 Hz (decodes bit-identical to `test.wav`) |
| `test.ogg` | Ogg Vorbis, 48000 Hz |
| `test.opus` | Ogg Opus, 48000 Hz |
| `test.mp3` | MP3, 48000 Hz |
| `test.aiff` | AIFF PCM (big-endian), 48000 Hz (decodes bit-identical to `test.wav`) |
| `test.caf` | Apple Lossless (ALAC) in CAF, 48000 Hz (decodes bit-identical to `test.wav`) |

### `kenney/`

Sound effects from [Kenney](https://kenney.nl) asset packs, in Ogg Vorbis: `congratulations`,
`forceField_000`, `impactMetal_003`, `laser1`, `spaceEngineLow_002` and `switch34`.
Credit to Kenney is welcome but not required (see `kenney/License.txt`).

## Using the files from another package

Declare the dependency in `package.xml` (`test_depend` if only tests use the files):

```xml
<exec_depend>audio2_samples</exec_depend>
```

The files are installed to `share/audio2_samples/<folder>/`. Find them through the ament index
rather than a fixed path.

C++ (link `ament_index_cpp::ament_index_cpp`):

```cpp
#include <ament_index_cpp/get_package_share_directory.hpp>

const std::string path =
  ament_index_cpp::get_package_share_directory("audio2_samples") + "/test_formats/test.wav";
```

Python:

```python
import os
from ament_index_python.packages import get_package_share_directory

path = os.path.join(get_package_share_directory('audio2_samples'), 'test_formats', 'test.wav')
```

Launch files:

```python
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

path = PathJoinSubstitution([FindPackageShare('audio2_samples'), 'test_formats', 'test.wav'])
```

Command line:

```bash
ls $(ros2 pkg prefix --share audio2_samples)/test_formats
```

## Adding files

Only add files that are public domain or CC0, record where each one came from in its folder's
license file, and keep them short. A new folder must also be added to the `install(DIRECTORY ...)`
line in `CMakeLists.txt`.
