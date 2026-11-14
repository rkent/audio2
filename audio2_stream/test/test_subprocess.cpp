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

#endif  // _WIN32
