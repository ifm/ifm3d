/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sei_parser.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace ifm3d::rtsp
{
  namespace
  {
    constexpr int SEI_TYPE_UNREGISTERED_USER_DATA = 5;
    constexpr int UUID_SIZE = 16;

    // SEI payloadType / payloadSize are each encoded as a run of 0xFF bytes
    // followed by one byte < 0xFF; the value is the sum of all bytes.
    int
    read_sei_field(const std::uint8_t*& p, const std::uint8_t* end)
    {
      int val = 0;
      while (p < end)
        {
          const std::uint8_t b = *p++;
          val += b;
          if (b != 0xFF)
            {
              break;
            }
        }
      return val;
    }

    std::uint32_t
    read_u32_le(const std::uint8_t* p)
    {
      return static_cast<std::uint32_t>(p[0]) |
             (static_cast<std::uint32_t>(p[1]) << 8) |
             (static_cast<std::uint32_t>(p[2]) << 16) |
             (static_cast<std::uint32_t>(p[3]) << 24);
    }

    std::uint64_t
    read_u64_le(const std::uint8_t* p)
    {
      return static_cast<std::uint64_t>(read_u32_le(p)) |
             (static_cast<std::uint64_t>(read_u32_le(p + 4)) << 32);
    }

  } // namespace

  std::vector<std::uint8_t>
  remove_emulation_prevention(const std::uint8_t* data, int len)
  {
    std::vector<std::uint8_t> out;
    out.reserve(static_cast<std::size_t>(len));
    for (int i = 0; i < len; ++i)
      {
        if (i + 2 < len && data[i] == 0x00 && data[i + 1] == 0x00 &&
            data[i + 2] == 0x03)
          {
            out.push_back(0x00);
            out.push_back(0x00);
            i += 2; // skip the 0x03
          }
        else
          {
            out.push_back(data[i]);
          }
      }
    return out;
  }

  void
  parse_sei_nal(const std::uint8_t* payload,
                int payload_len,
                const std::function<void(const std::array<std::uint8_t, 16>&,
                                         const std::vector<std::uint8_t>&)>&
                  on_unregistered)
  {
    if (!payload || payload_len <= 0)
      {
        return;
      }

    const std::vector<std::uint8_t> rbsp =
      remove_emulation_prevention(payload, payload_len);

    const std::uint8_t* p = rbsp.data();
    const std::uint8_t* end = rbsp.data() + rbsp.size();

    while (p < end)
      {
        if (*p == 0x80) // RBSP stop bit
          {
            break;
          }

        const int sei_type = read_sei_field(p, end);
        if (p >= end)
          {
            break;
          }
        const int sei_size = read_sei_field(p, end);
        if (p >= end || sei_size <= 0)
          {
            break;
          }

        const int available = static_cast<int>(end - p);
        const int consume = std::min(sei_size, available);

        if (sei_type == SEI_TYPE_UNREGISTERED_USER_DATA &&
            consume >= UUID_SIZE && on_unregistered)
          {
            std::array<std::uint8_t, 16> uuid{};
            std::copy(p, p + UUID_SIZE, uuid.begin());

            std::vector<std::uint8_t> const data(p + UUID_SIZE, p + consume);
            on_unregistered(uuid, data);
          }

        p += consume;
      }
  }

  std::optional<RgbInfoPayload>
  parse_rgb_info(const std::array<std::uint8_t, 16>& uuid,
                 const std::vector<std::uint8_t>& data)
  {
    if (uuid != RGB_INFO_UUID || data.size() < RGB_INFO_WIRE_SIZE)
      {
        return std::nullopt;
      }

    return RgbInfoPayload{
      std::vector<std::uint8_t>(data.begin(),
                                data.begin() + RGB_INFO_WIRE_SIZE),
      read_u32_le(data.data() + 4),
      read_u64_le(data.data() + 8)};
  }

} // namespace ifm3d::rtsp
