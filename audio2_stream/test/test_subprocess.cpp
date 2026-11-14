#include <gtest/gtest.h>
#include "subprocess.hpp"

#include <string>
#include <vector>

namespace
{

std::string as_string(const std::vector<uint8_t> & bytes)
{
  return std::string(bytes.begin(), bytes.end());
}

}  // namespace

TEST(RunProcess, MissingProgramIsAnError)
{
  std::vector<uint8_t> output;
  int exit_code = 0;
  auto error = run_process({"audio2_no_such_program"}, "", output, exit_code);
  EXPECT_TRUE(error.has_value());
}

TEST(RunProcess, EmptyArgvIsAnError)
{
  std::vector<uint8_t> output;
  int exit_code = 0;
  EXPECT_TRUE(run_process({}, "", output, exit_code).has_value());
}

#ifndef _WIN32

TEST(RunProcess, ShellMetacharactersInInputAreLiteral)
{
  const std::string input = "hello \"; echo pwned; $(id) `id` & | > /tmp/x\n'quoted' \\ end";
  std::vector<uint8_t> output;
  int exit_code = -1;
  auto error = run_process({"cat"}, input, output, exit_code);
  ASSERT_FALSE(error.has_value()) << *error;
  EXPECT_EQ(exit_code, 0);
  EXPECT_EQ(as_string(output), input);
}

TEST(RunProcess, ArgumentsAreNotParsedByAShell)
{
  std::vector<uint8_t> output;
  int exit_code = -1;
  auto error = run_process({"echo", "a b", "$(id)", ";", "c"}, "", output, exit_code);
  ASSERT_FALSE(error.has_value()) << *error;
  EXPECT_EQ(as_string(output), "a b $(id) ; c\n");
}

TEST(RunProcess, ReportsExitCode)
{
  std::vector<uint8_t> output;
  int exit_code = 0;
  auto error = run_process({"sh", "-c", "exit 3"}, "", output, exit_code);
  ASSERT_FALSE(error.has_value()) << *error;
  EXPECT_EQ(exit_code, 3);
}

TEST(RunProcess, LargeInputAndOutputDoNotDeadlock)
{
  // Much larger than a pipe buffer in both directions
  std::string input(4 * 1024 * 1024, 'x');
  for (std::size_t i = 0; i < input.size(); i += 101) {
    input[i] = static_cast<char>('a' + i % 26);
  }
  std::vector<uint8_t> output;
  int exit_code = -1;
  auto error = run_process({"cat"}, input, output, exit_code);
  ASSERT_FALSE(error.has_value()) << *error;
  EXPECT_EQ(exit_code, 0);
  EXPECT_EQ(as_string(output), input);
}

TEST(RunProcess, ChildThatIgnoresInputDoesNotKillUs)
{
  // The child exits without reading, so writing its stdin gets EPIPE (and SIGPIPE)
  std::string input(4 * 1024 * 1024, 'x');
  std::vector<uint8_t> output;
  int exit_code = -1;
  auto error = run_process({"true"}, input, output, exit_code);
  ASSERT_FALSE(error.has_value()) << *error;
  EXPECT_EQ(exit_code, 0);
  EXPECT_TRUE(output.empty());
}

#else  // _WIN32

// These use subprocess_helper (built alongside this test) in place of cat, echo and sh,
// since Windows has no equivalents that parse arguments with the C runtime rules.

namespace
{

// A string of the given length with varied content
std::string make_input(std::size_t size)
{
  std::string input(size, 'x');
  for (std::size_t i = 0; i < input.size(); i += 101) {
    input[i] = static_cast<char>('a' + i % 26);
  }
  return input;
}

}  // namespace

TEST(RunProcess, InputPassesThroughUnchanged)
{
  // Shell metacharacters, CR/LF, NUL and non-ASCII bytes must all arrive as-is
  std::string input = "hello \"; echo pwned & del x | more > nul ^ %PATH%\r\n'q' \\ end\n";
  input += std::string("\0\x01\xff\x80\r\x1a tail", 13);
  std::vector<uint8_t> output;
  int exit_code = -1;
  auto error = run_process({SUBPROCESS_HELPER, "cat"}, input, output, exit_code);
  ASSERT_FALSE(error.has_value()) << *error;
  EXPECT_EQ(exit_code, 0);
  EXPECT_EQ(as_string(output), input);
}

TEST(RunProcess, ArgumentsArriveUnchanged)
{
  // Each case exercises a rule of Windows command-line quoting
  const std::vector<std::string> args = {
    "plain",
    "a b",              // space
    "",                 // empty argument
    "say \"hi\"",       // embedded quotes
    "C:\\dir\\",        // trailing backslash
    "C:\\dir with space\\",  // trailing backslash inside quotes
    "back\\\\\"slash",  // backslashes before a quote
    "\\\\server\\share",  // leading backslashes
    "tab\there",
    "& | < > ^ % ; $(id)",  // shell metacharacters, literal without a shell
  };
  std::vector<std::string> argv = {SUBPROCESS_HELPER, "args"};
  argv.insert(argv.end(), args.begin(), args.end());
  std::vector<uint8_t> output;
  int exit_code = -1;
  auto error = run_process(argv, "", output, exit_code);
  ASSERT_FALSE(error.has_value()) << *error;
  EXPECT_EQ(exit_code, 0);

  std::string expected;
  for (const auto & arg : args) {
    expected += arg;
    expected.push_back('\0');
  }
  EXPECT_EQ(as_string(output), expected);
}

TEST(RunProcess, ReportsExitCode)
{
  for (int code : {0, 3, 300}) {
    std::vector<uint8_t> output;
    int exit_code = -1;
    auto error = run_process({SUBPROCESS_HELPER, "exit", std::to_string(code)}, "", output,
      exit_code);
    ASSERT_FALSE(error.has_value()) << *error;
    EXPECT_EQ(exit_code, code);
  }
}

TEST(RunProcess, ProgramIsFoundOnPath)
{
  // "cmd" is resolved through the search path, and ".exe" is added
  std::vector<uint8_t> output;
  int exit_code = -1;
  auto error = run_process({"cmd", "/c", "exit 7"}, "", output, exit_code);
  ASSERT_FALSE(error.has_value()) << *error;
  EXPECT_EQ(exit_code, 7);
}

TEST(RunProcess, LargeInputAndOutputDoNotDeadlock)
{
  // Much larger than a pipe buffer in both directions
  const std::string input = make_input(4 * 1024 * 1024);
  std::vector<uint8_t> output;
  int exit_code = -1;
  auto error = run_process({SUBPROCESS_HELPER, "cat"}, input, output, exit_code);
  ASSERT_FALSE(error.has_value()) << *error;
  EXPECT_EQ(exit_code, 0);
  EXPECT_EQ(as_string(output), input);
}

TEST(RunProcess, OutputBeforeReadingInputDoesNotDeadlock)
{
  // The child fills its stdout pipe before reading stdin, while we are still writing stdin
  const std::size_t flood = 4 * 1024 * 1024;
  const std::string input = make_input(4 * 1024 * 1024);
  std::vector<uint8_t> output;
  int exit_code = -1;
  auto error = run_process({SUBPROCESS_HELPER, "flood", std::to_string(flood)}, input, output,
    exit_code);
  ASSERT_FALSE(error.has_value()) << *error;
  EXPECT_EQ(exit_code, 0);
  EXPECT_EQ(as_string(output), std::string(flood, 'f') + input);
}

TEST(RunProcess, ChildThatIgnoresInputDoesNotFail)
{
  // The child exits without reading, so writing its stdin fails with a broken pipe
  const std::string input = make_input(4 * 1024 * 1024);
  std::vector<uint8_t> output;
  int exit_code = -1;
  auto error = run_process({SUBPROCESS_HELPER, "ignore"}, input, output, exit_code);
  ASSERT_FALSE(error.has_value()) << *error;
  EXPECT_EQ(exit_code, 0);
  EXPECT_TRUE(output.empty());
}

#endif  // _WIN32
