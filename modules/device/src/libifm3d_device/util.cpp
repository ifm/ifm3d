/*
 * Copyright 2018 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/device/util.h>
#include <sstream>
#include <string>
#include <vector>

std::string&
ifm3d::ltrim(std::string& str, const std::string& chars)
{
  str.erase(0, str.find_first_not_of(chars));
  return str;
}

std::string&
ifm3d::rtrim(std::string& str, const std::string& chars)
{
  str.erase(str.find_last_not_of(chars) + 1);
  return str;
}

std::string&
ifm3d::trim(std::string& str, const std::string& chars)
{
  return ifm3d::ltrim(ifm3d::rtrim(str, chars), chars);
}

std::vector<std::string>
ifm3d::split(const std::string& in, char delim)
{
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(in);
  while (std::getline(tokenStream, token, delim))
    {
      tokens.push_back(token);
    }
  return tokens;
}

bool
ifm3d::IsStdinAvailable(int timeoutSeconds)
{
#ifdef _WIN32
  if (!(_isatty(_fileno(stdin))))
    {
      HANDLE hInput = GetStdHandle(STD_INPUT_HANDLE);

      if (hInput == INVALID_HANDLE_VALUE)
        {
          std::cerr << "Error getting standard input handle." << std::endl;
          return false;
        }

      DWORD waitResult = WaitForSingleObject(hInput, timeoutSeconds * 1000);

      if (waitResult == WAIT_OBJECT_0)
        {
          return true;
        }
      else
        {
          return false;
        }
    }
  else
    {
      return false;
    }

#elif __unix__
  if (!(isatty(fileno(stdin))))
    {
      fd_set set;

      FD_ZERO(&set);
      FD_SET(STDIN_FILENO, &set);

      struct timeval timeout;

      timeout.tv_sec = timeoutSeconds;
      timeout.tv_usec = 0;

      int result = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);

      if (result > 0)
        {
          return true;
        }
      else
        {
          return false;
        }
    }
  else
    {
      return false;
    }

#else
#  error Unsupported platform! Code may not behave as expected.
#endif
}