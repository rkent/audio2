// Child program for test_subprocess, standing in for cat, echo, sh -c "exit N" and true.
// Usage:
//   subprocess_helper cat          copy stdin to stdout unchanged
//   subprocess_helper args A B...  write each argument followed by a NUL byte
//   subprocess_helper exit N       exit with code N
//   subprocess_helper ignore       exit 0 without reading stdin
//   subprocess_helper flood N      write N bytes to stdout before reading stdin, then copy stdin

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

namespace
{

void copy_stdin_to_stdout()
{
  std::vector<char> buffer(65536);
  std::size_t count;
  while ((count = std::fread(buffer.data(), 1, buffer.size(), stdin)) > 0) {
    std::fwrite(buffer.data(), 1, count, stdout);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
#ifdef _WIN32
  // No CR/LF translation, so bytes pass through exactly
  _setmode(_fileno(stdin), _O_BINARY);
  _setmode(_fileno(stdout), _O_BINARY);
#endif
  if (argc < 2) {
    return 99;
  }
  const char * mode = argv[1];
  if (std::strcmp(mode, "cat") == 0) {
    copy_stdin_to_stdout();
  } else if (std::strcmp(mode, "args") == 0) {
    for (int i = 2; i < argc; i++) {
      std::fwrite(argv[i], 1, std::strlen(argv[i]) + 1, stdout);
    }
  } else if (std::strcmp(mode, "exit") == 0 && argc == 3) {
    return std::atoi(argv[2]);
  } else if (std::strcmp(mode, "ignore") == 0) {
    return 0;
  } else if (std::strcmp(mode, "flood") == 0 && argc == 3) {
    std::vector<char> data(static_cast<std::size_t>(std::atol(argv[2])), 'f');
    std::fwrite(data.data(), 1, data.size(), stdout);
    std::fflush(stdout);
    copy_stdin_to_stdout();
  } else {
    return 99;
  }
  return 0;
}
