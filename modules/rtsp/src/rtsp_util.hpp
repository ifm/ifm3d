/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_RTSP_UTIL_HPP
#define IFM3D_RTSP_RTSP_UTIL_HPP

/** @file
 * @brief Small string, base64 and numeric parsing helpers used across the
 * RTSP module.
 */

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cstdint>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace ifm3d::rtsp
{
  /**
   * Parse a port string into a uint16_t. Returns 0 when the
   * value is malformed or outside the valid 0-65535 range.
   */
  inline std::uint16_t
  parse_port(std::string_view s)
  {
    unsigned int value = 0;
    const auto [ptr, ec] =
      std::from_chars(s.data(), s.data() + s.size(), value);
    if (ec != std::errc{} || ptr != s.data() + s.size() || value > 0xFFFFU)
      {
        return 0;
      }
    return static_cast<std::uint16_t>(value);
  }

  inline std::vector<std::uint8_t>
  base64_decode(std::string_view in)
  {
    static constexpr std::string_view K_CHARS =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    std::vector<std::uint8_t> out;
    out.reserve(in.size() * 3 / 4);
    std::uint32_t accum = 0;
    int bits = 0;
    for (char c : in)
      {
        if (c == '=')
          {
            break;
          }
        const auto pos = K_CHARS.find(c);
        if (pos == std::string_view::npos)
          {
            continue;
          }
        accum = (accum << 6) | static_cast<std::uint32_t>(pos);
        bits += 6;
        if (bits >= 8)
          {
            bits -= 8;
            out.push_back(static_cast<std::uint8_t>((accum >> bits) & 0xFF));
          }
      }
    return out;
  }

  inline std::string
  to_lower(std::string s)
  {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return s;
  }

  inline std::string
  trim(const std::string& s)
  {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos)
      {
        return {};
      }
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
  }

  inline std::string
  replace_all(std::string s, const std::string& from, const std::string& to)
  {
    std::size_t pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos)
      {
        s.replace(pos, from.size(), to);
        pos += to.size();
      }
    return s;
  }

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_RTSP_UTIL_HPP
