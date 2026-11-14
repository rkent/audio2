#ifndef AUDIO2_STREAM_TIMESTAMP_HPP
#define AUDIO2_STREAM_TIMESTAMP_HPP

#include <chrono>
#include <cstdio>
#include <string>

// utility function to format timestamps for debugging
inline std::string format_timestamp()
{
  auto time_point = std::chrono::steady_clock::now();
  auto time_since_epoch = time_point.time_since_epoch();
  auto hours = std::chrono::duration_cast<std::chrono::hours>(time_since_epoch) % 24;
  auto minutes = std::chrono::duration_cast<std::chrono::minutes>(time_since_epoch) % 60;
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch) % 60;
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch) %
    1000;

  char buffer[256];
  snprintf(buffer, sizeof(buffer), "%02ld:%02ld:%02ld:%03ld",
                hours.count(), minutes.count(), seconds.count(), milliseconds.count());
  return std::string(buffer);
}

#endif  // AUDIO2_STREAM_TIMESTAMP_HPP
