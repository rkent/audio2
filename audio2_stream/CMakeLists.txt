cmake_minimum_required(VERSION 3.20)
project(audio2_stream)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20, needed for std::atomic with boost lockfree queue
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# Find sndfile library from debian package, it does not seem to have a CMake package
find_library(SNDFILE_LIBRARIES NAMES sndfile REQUIRED)
find_path(SNDFILE_INCLUDE_DIRS sndfile.h REQUIRED)
if(WIN32)
  # Keep windows.h (pulled in by curl) from defining min/max macros
  add_compile_definitions(NOMINMAX)
  # MSVC only defines M_PI and friends when asked
  add_compile_definitions(_USE_MATH_DEFINES)
else()
  find_package(ALSA REQUIRED)
endif()
find_package(Boost REQUIRED)
find_package(audio2_stream_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Add all exported headers to include/<package_name> otherwise other packages cannot see it
# This makes headers available to other packages that depend on yours.
# Only headers for code compiled on this platform (see below) are installed; use
# make_default_device_proxy() rather than naming a backend class directly.
if(WIN32)
  set(UNBUILT_HEADERS AlsaProxyImpl.hpp alsaops.hpp)
else()
  set(UNBUILT_HEADERS WasapiProxyImpl.hpp)
endif()
set(UNBUILT_HEADER_EXCLUDES)
foreach(header ${UNBUILT_HEADERS})
  list(APPEND UNBUILT_HEADER_EXCLUDES PATTERN ${header} EXCLUDE)
endforeach()
install(DIRECTORY include/ DESTINATION include ${UNBUILT_HEADER_EXCLUDES})
install(DIRECTORY templates/ DESTINATION share/${PROJECT_NAME}/templates)

# 1. Define the shared library
# Add all source files of the library to add_library
add_library(
    ${PROJECT_NAME}
    SHARED
    src/lib/buffer_file.cpp
    src/lib/AudioStream.cpp
    src/lib/subprocess.cpp
)
# Selects dllexport in visibility_control.hpp while building the library; users get dllimport
target_compile_definitions(${PROJECT_NAME} PRIVATE "AUDIO2_STREAM_BUILDING_LIBRARY")
# Audio device backend; each defines make_default_device_proxy()
if(WIN32)
  target_sources(${PROJECT_NAME} PRIVATE src/lib/WasapiProxyImpl.cpp)
  target_link_libraries(${PROJECT_NAME} PRIVATE ole32)
else()
  target_sources(${PROJECT_NAME} PRIVATE src/lib/AlsaProxyImpl.cpp src/lib/alsaops.cpp)
  target_link_libraries(${PROJECT_NAME} PRIVATE ALSA::ALSA)
endif()

#set(SOURCE_FILES
#  src/alsaops.cpp
#  src/buffer_file.cpp
#)
add_executable(audio2_play_remote_node src/audio2_play_remote_node.cpp)
add_executable(audio2_play_node src/audio2_play_node.cpp)
add_executable(audio2_capture_node src/audio2_capture_node.cpp)

target_link_libraries(audio2_stream
  PRIVATE
    ${SNDFILE_LIBRARIES}
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    nlohmann_json::nlohmann_json
    CURL::libcurl
    ${audio2_stream_msgs_TARGETS}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# ALSA-only test tool, which uses alsaops directly rather than an IAudioDeviceProxy
if(NOT WIN32)
  add_executable(loop_file src/loop_file.cpp)
  target_link_libraries(loop_file
    PRIVATE
      ${PROJECT_NAME}
      ${SNDFILE_LIBRARIES}
      ALSA::ALSA
      rclcpp::rclcpp
      ${audio2_stream_msgs_TARGETS}
  )
  install(TARGETS loop_file DESTINATION lib/${PROJECT_NAME})
endif()

target_link_libraries(audio2_play_remote_node
  PRIVATE
    ${PROJECT_NAME}
    ${SNDFILE_LIBRARIES}
    rclcpp::rclcpp
    ${audio2_stream_msgs_TARGETS}
)

target_link_libraries(audio2_play_node
  PRIVATE
    ${PROJECT_NAME}
    ${SNDFILE_LIBRARIES}
    CURL::libcurl
    rclcpp::rclcpp
    ${audio2_stream_msgs_TARGETS}
)

target_link_libraries(audio2_capture_node
  PRIVATE
    ${PROJECT_NAME}
    ${SNDFILE_LIBRARIES}
    rclcpp::rclcpp
    ${audio2_stream_msgs_TARGETS}
)


# Attach sndfile headers to the library (SYSTEM, propagated to everything linking it) rather
# than directory-wide. A directory-wide include of a shared prefix such as the pixi/conda
# Library/include would shadow the vendored gtest headers, which ament adds as SYSTEM.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${SNDFILE_INCLUDE_DIRS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  list(APPEND AMENT_LINT_AUTO_FILE_EXCLUDE "include/inja/inja.hpp")
  ament_lint_auto_find_test_dependencies()

  # Add GTest and GMock for unit testing
  find_package(ament_cmake_gtest REQUIRED)

  # Enable thread sanitizer for thread safety validation (unit tests only)
  # Note: ThreadSanitizer can cause issues in Docker/integration test environments
  #set(THREAD_SANITIZER_COMPILE_FLAGS "-fsanitize=thread" "-g" "-fno-omit-frame-pointer")
  #set(THREAD_SANITIZER_LINK_FLAGS "-fsanitize=thread")

  # Unit tests for AudioStream
  ament_add_gtest(test_audio_stream
    test/test_audio_stream.cpp
  )
  #target_compile_options(test_audio_stream PRIVATE ${THREAD_SANITIZER_COMPILE_FLAGS})
  #target_link_options(test_audio_stream PRIVATE ${THREAD_SANITIZER_LINK_FLAGS})
  target_link_libraries(test_audio_stream
    ${PROJECT_NAME}
    ${SNDFILE_LIBRARIES}
    rclcpp::rclcpp
    ${audio2_stream_msgs_TARGETS}
  )
  target_include_directories(test_audio_stream PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # Round trip through the in-memory sound file encoding used for AudioChunk messages
  ament_add_gtest(test_audio_chunk_codec
    test/test_audio_chunk_codec.cpp
  )
  target_link_libraries(test_audio_chunk_codec
    ${PROJECT_NAME}
    ${SNDFILE_LIBRARIES}
  )

  # Unit tests for run_process. It is internal to the library (not exported), so compile it in.
  ament_add_gtest(test_subprocess
    test/test_subprocess.cpp
    src/lib/subprocess.cpp
  )
  target_include_directories(test_subprocess PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/lib)
  if(WIN32)
    # Child program for the Windows run_process tests, standing in for cat, echo and sh
    add_executable(subprocess_helper test/subprocess_helper.cpp)
    add_dependencies(test_subprocess subprocess_helper)
    target_compile_definitions(test_subprocess PRIVATE
      SUBPROCESS_HELPER="$<TARGET_FILE:subprocess_helper>")
  endif()

  # Unit tests for DeviceSink/DeviceSource against the platform audio backend
  ament_add_gtest(test_device_terminal
    test/test_device_terminal.cpp
  )
  #target_compile_options(test_device_terminal PRIVATE ${THREAD_SANITIZER_COMPILE_FLAGS})
  #target_link_options(test_device_terminal PRIVATE ${THREAD_SANITIZER_LINK_FLAGS})
  target_link_libraries(test_device_terminal
    ${PROJECT_NAME}
    ${SNDFILE_LIBRARIES}
    rclcpp::rclcpp
    ${audio2_stream_msgs_TARGETS}
  )
  target_include_directories(test_device_terminal PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  if(WIN32)
    # Unit tests for the WASAPI backend
    ament_add_gtest(test_wasapi
      test/test_wasapi.cpp
    )
    target_link_libraries(test_wasapi
      ${PROJECT_NAME}
      rclcpp::rclcpp
      ${audio2_stream_msgs_TARGETS}
    )
    target_include_directories(test_wasapi PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
  endif()

  # Integration tests using launch_testing
  find_package(launch_testing_ament_cmake REQUIRED)

  # Add integration test for audio2_play_node
  add_launch_test(
    test/integration/launch/test_audio2_play_node.launch.py
    TARGET test_audio2_play_node_integration
    TIMEOUT 60
  )

  # Install integration test files
  install(
    DIRECTORY test/integration/
    DESTINATION share/${PROJECT_NAME}/test/integration
    USE_SOURCE_PERMISSIONS
  )

  # Install test fixtures
  install(
    DIRECTORY test/integration/test_fixtures/
    DESTINATION share/${PROJECT_NAME}/test/fixtures
  )
endif()

install(TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  audio2_play_remote_node
  audio2_play_node
  audio2_capture_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
