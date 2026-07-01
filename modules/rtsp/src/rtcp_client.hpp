/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_RTCP_CLIENT_HPP
#define IFM3D_RTSP_RTCP_CLIENT_HPP

/** @file
 * @brief Minimal RTCP receiver (RFC 3550).
 */

#include <cstdint>
#include <functional>
#include <vector>

#include "rtp_connection.hpp"

namespace ifm3d::rtsp
{
  /**
   * @brief Receives RTCP packets on its transport.
   *
   * The current implementation only drains the channel; parsing of
   * SR/RR/BYE reports is not yet implemented.
   */
  class RtcpClient
  {
  public:
    RtcpClient() = default;

    void InitConnection(RtpConnectionSP connection);

    std::function<void(int)> on_error;

  private:
    void process_packet(const std::vector<std::uint8_t>& data);

    RtpConnectionSP _connection;
  };

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_RTCP_CLIENT_HPP
