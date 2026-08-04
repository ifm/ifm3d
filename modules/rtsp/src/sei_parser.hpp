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
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace ifm3d::rtsp
{
  inline constexpr std::size_t RGB_INFO_WIRE_SIZE = 308;
  inline constexpr std::array<std::uint8_t, 16> RGB_INFO_UUID = {0xe6,
                                                                 0xcc,
                                                                 0xb4,
                                                                 0xb0,
                                                                 0x00,
                                                                 0x71,
                                                                 0x58,
                                                                 0x0e,
                                                                 0x82,
                                                                 0x8d,
                                                                 0x92,
                                                                 0x8c,
                                                                 0x60,
                                                                 0x07,
                                                                 0x81,
                                                                 0x43};

  struct RgbInfoPayload
  {
    std::vector<std::uint8_t> data;
    std::uint32_t frame_counter = 0;
    std::uint64_t timestamp_ns = 0;
  };

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
   * Validate an O3C `RGB_INFO` SEI payload and extract the fields used by
   * `Frame`.
   *
   * @param uuid The SEI message UUID.
   * @param data The user-data bytes following the UUID.
   * @return the raw RGB_INFO data and its frame identity, or std::nullopt if
   *         the UUID does not match RGB_INFO or the payload is too short.
   */
  std::optional<RgbInfoPayload> parse_rgb_info(
    const std::array<std::uint8_t, 16>& uuid,
    const std::vector<std::uint8_t>& data);

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_SEI_PARSER_HPP
