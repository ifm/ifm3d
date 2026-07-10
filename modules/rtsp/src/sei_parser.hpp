/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_SEI_PARSER_HPP
#define IFM3D_RTSP_SEI_PARSER_HPP

/** @file
 * @brief H.264 SEI parsing helpers (ITU-T H.264 §D.1).
 */

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include <ifm3d/rtsp/frame_metadata.h>

namespace ifm3d::rtsp
{
  /**
   * Strip RBSP emulation-prevention bytes (ITU-T H.264 §7.4.1):
   * every `00 00 03` sequence has the `03` removed.
   */
  std::vector<std::uint8_t> remove_emulation_prevention(
    const std::uint8_t* data,
    int len);

  /**
   * Parse an SEI NAL payload (everything after the NAL header byte) and
   * invoke @p on_unregistered for every unregistered-user-data message
   * (payloadType 5) found.
   *
   * @param payload    Pointer to the SEI RBSP (after the NAL header byte).
   * @param payload_len Length of @p payload.
   * @param on_unregistered Callback receiving the 16-byte UUID and the
   *                        user data bytes that follow it.
   */
  void parse_sei_nal(
    const std::uint8_t* payload,
    int payload_len,
    const std::function<void(const std::array<std::uint8_t, 16>&,
                             const std::vector<std::uint8_t>&)>&
      on_unregistered);

  /**
   * Decode an O3C `RGB_INFO` SEI payload (308-byte little-endian layout).
   *
   * @param uuid The SEI message UUID.
   * @param data The user-data bytes following the UUID.
   * @return the parsed structure, or std::nullopt if the UUID does not match
   *         RGB_INFO or the payload is too short.
   */
  std::optional<RgbInfo> parse_rgb_info(
    const std::array<std::uint8_t, 16>& uuid,
    const std::vector<std::uint8_t>& data);

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_SEI_PARSER_HPP
