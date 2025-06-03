#include <ifm3d/common/util.h>

void
ifm3d::stdin_echo(bool enable)
{
#ifdef _WIN32
  HANDLE hStdin = GetStdHandle(STD_INPUT_HANDLE);
  DWORD mode;
  GetConsoleMode(hStdin, &mode);

  if (!enable)
    mode &= ~ENABLE_ECHO_INPUT;
  else
    mode |= ENABLE_ECHO_INPUT;

  SetConsoleMode(hStdin, mode);
#else
  struct termios tty;
  tcgetattr(STDIN_FILENO, &tty);
  if (!enable)
    tty.c_lflag &= ~ECHO;
  else
    tty.c_lflag |= ECHO;

  tcsetattr(STDIN_FILENO, TCSANOW, &tty);
#endif
}

std::string
ifm3d::read_password()
{
  std::string password;
  stdin_echo(false);
  std::getline(std::cin, password);
  stdin_echo(true);
  std::cout << std::endl; // Move to the next line after input
  return password;
}
