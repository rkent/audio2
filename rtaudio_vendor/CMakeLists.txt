cmake_minimum_required(VERSION 3.10)
project(rtaudio_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

# Always build the rkent fork (alsa branch): it probes ALSA by PCM device,
# which the Ubuntu librtaudio-dev package does not, so no SATISFIED check.
ament_vendor(rtaudio_vendor
  VCS_URL https://github.com/rkent/rtaudio.git
  VCS_VERSION alsa
  CMAKE_ARGS
    -DRTAUDIO_BUILD_TESTING=OFF
    -DRTAUDIO_API_ALSA=ON
    -DRTAUDIO_API_PULSE=ON
    -DRTAUDIO_API_JACK=OFF
)

ament_package()
