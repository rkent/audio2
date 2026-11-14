#include "subprocess.hpp"

#include <thread>

#ifdef _WIN32

#include <windows.h>
#include <algorithm>
#include <format>

namespace
{

std::string last_error(const std::string & what)
{
  return std::format("{} failed (Windows error {})", what, GetLastError());
}

// Quote one argument so the program's C runtime parses it back unchanged
// (CommandLineToArgvW rules: backslashes are literal except before a quote).
std::string quote_arg(const std::string & arg)
{
  if (!arg.empty() && arg.find_first_of(" \t\n\v\"") == std::string::npos) {
    return arg;
  }
  std::string quoted = "\"";
  for (auto it = arg.begin(); ; ++it) {
    std::size_t backslashes = 0;
    while (it != arg.end() && *it == '\\') {
      ++it;
      ++backslashes;
    }
    if (it == arg.end()) {
      // Double them so they do not escape the closing quote
      quoted.append(backslashes * 2, '\\');
      break;
    }
    if (*it == '"') {
      quoted.append(backslashes * 2 + 1, '\\');
    } else {
      quoted.append(backslashes, '\\');
    }
    quoted.push_back(*it);
  }
  quoted.push_back('"');
  return quoted;
}

void close_handle(HANDLE & handle)
{
  if (handle) {
    CloseHandle(handle);
    handle = nullptr;
  }
}

}  // namespace

std::optional<std::string> run_process(
  const std::vector<std::string> & argv,
  const std::string & input,
  std::vector<uint8_t> & output,
  int & exit_code)
{
  if (argv.empty()) {
    return std::string("run_process: no program given");
  }

  // Pipes for the child's stdin and stdout. Only the child's ends are inheritable.
  SECURITY_ATTRIBUTES sa{sizeof(sa), nullptr, TRUE};
  HANDLE in_read = nullptr, in_write = nullptr, out_read = nullptr, out_write = nullptr;
  if (!CreatePipe(&in_read, &in_write, &sa, 0)) {
    return last_error("CreatePipe");
  }
  if (!CreatePipe(&out_read, &out_write, &sa, 0)) {
    auto error = last_error("CreatePipe");
    close_handle(in_read);
    close_handle(in_write);
    return error;
  }
  SetHandleInformation(in_write, HANDLE_FLAG_INHERIT, 0);
  SetHandleInformation(out_read, HANDLE_FLAG_INHERIT, 0);

  // The child shares our stderr, through an inheritable duplicate
  HANDLE err_dup = nullptr;
  HANDLE err = GetStdHandle(STD_ERROR_HANDLE);
  if (err && err != INVALID_HANDLE_VALUE) {
    if (!DuplicateHandle(GetCurrentProcess(), err, GetCurrentProcess(), &err_dup, 0, TRUE,
        DUPLICATE_SAME_ACCESS))
    {
      err_dup = nullptr;
    }
  }

  // Let the child inherit only these handles, not ones another thread is creating
  HANDLE inherit[3] = {in_read, out_write, err_dup};
  DWORD inherit_count = err_dup ? 3 : 2;
  SIZE_T attr_size = 0;
  InitializeProcThreadAttributeList(nullptr, 1, 0, &attr_size);
  std::vector<char> attr_buffer(attr_size);
  auto attrs = reinterpret_cast<LPPROC_THREAD_ATTRIBUTE_LIST>(attr_buffer.data());
  bool attrs_ok = InitializeProcThreadAttributeList(attrs, 1, 0, &attr_size) &&
    UpdateProcThreadAttribute(attrs, 0, PROC_THREAD_ATTRIBUTE_HANDLE_LIST, inherit,
      inherit_count * sizeof(HANDLE), nullptr, nullptr);

  std::optional<std::string> error;
  PROCESS_INFORMATION pi{};
  if (!attrs_ok) {
    error = last_error("InitializeProcThreadAttributeList");
  } else {
    STARTUPINFOEXA si{};
    si.StartupInfo.cb = sizeof(si);
    si.StartupInfo.dwFlags = STARTF_USESTDHANDLES;
    si.StartupInfo.hStdInput = in_read;
    si.StartupInfo.hStdOutput = out_write;
    si.StartupInfo.hStdError = err_dup;
    si.lpAttributeList = attrs;

    std::string command_line;
    for (const auto & arg : argv) {
      if (!command_line.empty()) {
        command_line.push_back(' ');
      }
      command_line += quote_arg(arg);
    }
    if (!CreateProcessA(nullptr, command_line.data(), nullptr, nullptr, TRUE,
        EXTENDED_STARTUPINFO_PRESENT, nullptr, nullptr, &si.StartupInfo, &pi))
    {
      error = last_error("Starting '" + argv[0] + "'");
    }
    DeleteProcThreadAttributeList(attrs);
  }

  // The child has its own copies now; close ours so EOF is seen when it exits
  close_handle(in_read);
  close_handle(out_write);
  close_handle(err_dup);
  if (error) {
    close_handle(in_write);
    close_handle(out_read);
    return error;
  }
  CloseHandle(pi.hThread);

  std::thread writer([in_write, &input]() {
      std::size_t offset = 0;
      while (offset < input.size()) {
        DWORD chunk = static_cast<DWORD>(std::min<std::size_t>(input.size() - offset, 1 << 16));
        DWORD written = 0;
        if (!WriteFile(in_write, input.data() + offset, chunk, &written, nullptr)) {
          break;  // The child exited or closed its stdin
        }
        offset += written;
      }
      CloseHandle(in_write);
    });

  char buffer[4096];
  DWORD bytes_read = 0;
  while (ReadFile(out_read, buffer, sizeof(buffer), &bytes_read, nullptr) && bytes_read > 0) {
    output.insert(output.end(), buffer, buffer + bytes_read);
  }
  close_handle(out_read);
  writer.join();

  WaitForSingleObject(pi.hProcess, INFINITE);
  DWORD code = 0;
  GetExitCodeProcess(pi.hProcess, &code);
  CloseHandle(pi.hProcess);
  exit_code = static_cast<int>(code);
  return std::nullopt;
}

#else  // POSIX

#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <pthread.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char ** environ;

namespace
{

std::string errno_message(const std::string & what, int err)
{
  return what + " failed: " + std::strerror(err);
}

void close_fd(int & fd)
{
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

// A pipe whose ends are not inherited by child processes unless dup2'd to a standard fd
int cloexec_pipe(int fds[2])
{
  if (::pipe(fds) != 0) {
    return errno;
  }
  for (int i = 0; i < 2; ++i) {
    if (::fcntl(fds[i], F_SETFD, FD_CLOEXEC) != 0) {
      int err = errno;
      ::close(fds[0]);
      ::close(fds[1]);
      return err;
    }
  }
  return 0;
}

}  // namespace

std::optional<std::string> run_process(
  const std::vector<std::string> & argv,
  const std::string & input,
  std::vector<uint8_t> & output,
  int & exit_code)
{
  if (argv.empty()) {
    return std::string("run_process: no program given");
  }

  int in_pipe[2] = {-1, -1};
  int out_pipe[2] = {-1, -1};
  if (int err = cloexec_pipe(in_pipe)) {
    return errno_message("pipe", err);
  }
  if (int err = cloexec_pipe(out_pipe)) {
    close_fd(in_pipe[0]);
    close_fd(in_pipe[1]);
    return errno_message("pipe", err);
  }

  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, in_pipe[0], STDIN_FILENO);
  posix_spawn_file_actions_adddup2(&actions, out_pipe[1], STDOUT_FILENO);

  std::vector<char *> args;
  for (const auto & arg : argv) {
    args.push_back(const_cast<char *>(arg.c_str()));
  }
  args.push_back(nullptr);

  pid_t pid = -1;
  int spawn_err = posix_spawnp(&pid, args[0], &actions, nullptr, args.data(), environ);
  posix_spawn_file_actions_destroy(&actions);

  // The child has its own copies now; close ours so EOF is seen when it exits
  close_fd(in_pipe[0]);
  close_fd(out_pipe[1]);
  if (spawn_err != 0) {
    close_fd(in_pipe[1]);
    close_fd(out_pipe[0]);
    return errno_message("Starting '" + argv[0] + "'", spawn_err);
  }

  std::thread writer([fd = in_pipe[1], &input]() {
      // If the child exits without reading all its input, write() fails with EPIPE.
      // Block SIGPIPE in this thread so that does not kill the process; a SIGPIPE
      // pending on this thread is discarded when it exits.
      sigset_t sigpipe;
      sigemptyset(&sigpipe);
      sigaddset(&sigpipe, SIGPIPE);
      pthread_sigmask(SIG_BLOCK, &sigpipe, nullptr);

      std::size_t offset = 0;
      while (offset < input.size()) {
        ssize_t n = ::write(fd, input.data() + offset, input.size() - offset);
        if (n < 0) {
          if (errno == EINTR) {
            continue;
          }
          break;  // The child exited or closed its stdin
        }
        offset += static_cast<std::size_t>(n);
      }
      ::close(fd);
    });

  char buffer[4096];
  while (true) {
    ssize_t n = ::read(out_pipe[0], buffer, sizeof(buffer));
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n <= 0) {
      break;
    }
    output.insert(output.end(), buffer, buffer + n);
  }
  close_fd(out_pipe[0]);
  writer.join();

  int status = 0;
  while (::waitpid(pid, &status, 0) < 0) {
    if (errno != EINTR) {
      return errno_message("waitpid", errno);
    }
  }
  if (WIFEXITED(status)) {
    exit_code = WEXITSTATUS(status);
  } else if (WIFSIGNALED(status)) {
    exit_code = 128 + WTERMSIG(status);
  } else {
    exit_code = -1;
  }
  return std::nullopt;
}

#endif
