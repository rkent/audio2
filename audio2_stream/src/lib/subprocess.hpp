#ifndef AUDIO2_STREAM_SUBPROCESS_HPP
#define AUDIO2_STREAM_SUBPROCESS_HPP

// Internal to the audio2_stream library; not installed.

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

/**
 * Run a program directly (no shell), write input to its stdin, and collect its stdout.
 *
 * Arguments are passed to the program as-is, so they need no shell quoting, and
 * untrusted data passed as input can never be interpreted as a command.
 * The program is looked up on the PATH. Its stderr is inherited from this process.
 * stdin is written from a separate thread, so a program that produces output before
 * reading all its input cannot deadlock.
 *
 * \param argv      Program name followed by its arguments. Must not be empty.
 * \param input     Data written to the program's stdin, which is then closed.
 * \param output    Receives everything the program writes to stdout (appended).
 * \param exit_code Set to the program's exit code; 128 + signal number if it was
 *                  killed by a signal (POSIX).
 * \return          std::nullopt if the program ran (check exit_code), or an error
 *                  message if it could not be started.
 */
std::optional<std::string> run_process(
  const std::vector<std::string> & argv,
  const std::string & input,
  std::vector<uint8_t> & output,
  int & exit_code);

#endif  // AUDIO2_STREAM_SUBPROCESS_HPP
