/*
 * Copyright 2018 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ifm3d/common/err.h"
#include <cstdint>
#if defined(__GNUC__) && !defined(__clang__) && !defined(_MSC_VER)
#  include <bits/types/struct_timeval.h>
#elif defined(__clang__) && !defined(_MSC_VER)
#  if __has_include(<bits/types/struct_timeval.h>)
#    include <bits/types/struct_timeval.h>
#  endif
#endif
#include "fmt/core.h"
#include <ifm3d/device/util.h>
#include <sstream>
#include <cstdio>
#include <string>
#include <vector>
#include <httplib.h>
#include <array>
#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#  include <io.h>
#elif __unix__
#  include <sys/select.h>
#  include <unistd.h>
#endif

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
  std::istringstream token_stream(in);
  while (std::getline(token_stream, token, delim))
    {
      tokens.push_back(token);
    }
  return tokens;
}

// Based on https://stackoverflow.com/a/37109258
std::string
ifm3d::base64_encode(const std::vector<std::uint8_t>& data)
{
  static const char* B64CHARS =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  size_t const len = data.size();
  std::string result((len + 2) / 3 * 4, '=');
  const uint8_t* p = data.data();
  char* str = result.data();
  size_t j = 0;
  size_t pad = len % 3;
  const size_t last = len - pad;

  for (size_t i = 0; i < last; i += 3)
    {
      int const n = int(p[i]) << 16 | int(p[i + 1]) << 8 | p[i + 2];
      str[j++] = B64CHARS[n >> 18];
      str[j++] = B64CHARS[n >> 12 & 0x3F];
      str[j++] = B64CHARS[n >> 6 & 0x3F];
      str[j++] = B64CHARS[n & 0x3F];
    }
  if (pad) /// Set padding
    {
      int const n = --pad ? int(p[last]) << 8 | p[last + 1] : p[last];
      str[j++] = B64CHARS[pad ? n >> 10 & 0x3F : n >> 2];
      str[j++] = B64CHARS[pad ? n >> 4 & 0x03F : n << 4 & 0x3F];
      str[j++] = pad ? B64CHARS[n << 2 & 0x3F] : '=';
    }
  return result;
}

// Based on https://stackoverflow.com/a/37109258
std::vector<std::uint8_t>
ifm3d::base64_decode(const std::string& base64)
{
  static constexpr std::array<int, 256> B64INDEX = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  62, 63, 62, 62, 63, 52, 53, 54, 55, 56, 57,
    58, 59, 60, 61, 0,  0,  0,  0,  0,  0,  0,  0,  1,  2,  3,  4,  5,  6,
    7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
    25, 0,  0,  0,  0,  63, 0,  26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
    37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51};

  std::size_t const len = base64.length();
  if (len == 0)
    {
      return {};
    }

  const auto* p = reinterpret_cast<const uint8_t*>(base64.c_str());
  size_t j = 0;
  size_t const pad1 = len % 4 || p[len - 1] == '=';
  size_t const pad2 = pad1 && (len % 4 > 2 || p[len - 2] != '=');
  const size_t last = (len - pad1) / 4 << 2;
  std::vector<std::uint8_t> result((last / 4 * 3) + pad1 + pad2, '\0');
  auto* str = (uint8_t*)result.data();

  for (size_t i = 0; i < last; i += 4)
    {
      int const n = B64INDEX[p[i]] << 18 | B64INDEX[p[i + 1]] << 12 |
                    B64INDEX[p[i + 2]] << 6 | B64INDEX[p[i + 3]];
      str[j++] = n >> 16;
      str[j++] = n >> 8 & 0xFF;
      str[j++] = n & 0xFF;
    }
  if (pad1)
    {
      int n = B64INDEX[p[last]] << 18 | B64INDEX[p[last + 1]] << 12;
      str[j++] = n >> 16;
      if (pad2)
        {
          n |= B64INDEX[p[last + 2]] << 6;
          str[j++] = n >> 8 & 0xFF;
        }
    }

  return result;
}

bool
ifm3d::IsStdinAvailable(int timeout_seconds)
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

      DWORD waitResult = WaitForSingleObject(hInput, timeout_seconds * 1000);

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
  if (!(isatty(STDIN_FILENO)))
    {
      fd_set set;

      FD_ZERO(&set);
      FD_SET(STDIN_FILENO, &set);

      struct timeval timeout
      {
      };

      timeout.tv_sec = timeout_seconds;
      timeout.tv_usec = 0;

      int const result =
        select(STDIN_FILENO + 1, &set, nullptr, nullptr, &timeout);

      return result > 0;
    }

  return false;

#else
#  error Unsupported platform! Code may not behave as expected.
#endif
}

void
ifm3d::check_http_result(httplib::Result const& res)
{
  if (res.error() != httplib::Error::Success)
    {
      if (res.error() == httplib::Error::ConnectionTimeout)
        {
          throw ifm3d::Error(IFM3D_CURL_TIMEOUT, "Connection timeout.");
        }

      throw ifm3d::Error(IFM3D_CURL_ERROR,
                         fmt::format("{}", httplib::to_string(res.error())));
    }

  if (res->status != 200)
    {

      if (res->status == 407)
        {
          throw ifm3d::Error(IFM3D_PROXY_AUTH_REQUIRED);
        }

      throw ifm3d::Error(IFM3D_XMLRPC_FAILURE,
                         fmt::format("HTTP Status: {}", res->status));
    }
}