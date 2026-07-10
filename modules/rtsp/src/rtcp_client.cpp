/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtcp_client.hpp"

#include <cstdint>
#include <utility>
#include <vector>

#include "rtp_connection.hpp"

namespace ifm3d::rtsp
{
  void
  RtcpClient::InitConnection(RtpConnectionSP connection)
  {
    if (_connection)
      {
        _connection->on_data_received = nullptr;
      }

    _connection = std::move(connection);
    _connection->on_data_received =
      [this](const std::vector<std::uint8_t>& data) { process_packet(data); };
  }

  void
  RtcpClient::process_packet(const std::vector<std::uint8_t>& /*data*/)
  {
    // RTCP SR/RR/BYE parsing is not yet implemented; packets are drained.
  }

} // namespace ifm3d::rtsp
