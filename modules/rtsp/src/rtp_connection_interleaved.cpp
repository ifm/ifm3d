/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtp_connection_interleaved.hpp"

#include <cstdint>
#include <vector>

#include "byte_span.hpp"
#include "rtsp_session.hpp"

namespace ifm3d::rtsp
{
  RtpConnectionInterleaved::RtpConnectionInterleaved(RtspSession& session,
                                                     int channel)
    : _session(&session),
      _channel(channel)
  {
    _session->AddChannelListener(
      channel,
      [this](std::uint8_t ch, const std::vector<std::uint8_t>& data) {
        OnChannelDataReceived(ch, data);
      });
  }

  void
  RtpConnectionInterleaved::Send(ByteSpan data)
  {
    _session->WriteChannelData(static_cast<std::uint8_t>(_channel), data);
  }

  void
  RtpConnectionInterleaved::OnChannelDataReceived(
    std::uint8_t channel,
    const std::vector<std::uint8_t>& data)
  {
    if (channel == static_cast<std::uint8_t>(_channel) && on_data_received)
      {
        on_data_received(data);
      }
  }

} // namespace ifm3d::rtsp
