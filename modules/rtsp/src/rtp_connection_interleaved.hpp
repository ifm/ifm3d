/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_RTP_CONNECTION_INTERLEAVED_HPP
#define IFM3D_RTSP_RTP_CONNECTION_INTERLEAVED_HPP

/** @file
 * @brief RTP/RTCP transport multiplexed over an interleaved RTSP/TCP
 * connection (RFC 2326 §10.12).
 */

#include <cstdint>
#include <vector>

#include "rtp_connection.hpp"

namespace ifm3d::rtsp
{
  class RtspSession;

  /**
   * @brief RTP/RTCP transport that rides on the interleaved RTSP control
   * connection.
   */
  class RtpConnectionInterleaved : public RtpConnection
  {
  public:
    /**
     * @param session  Owning RTSP session (must outlive this object).
     * @param channel  Interleaved channel (0 = RTP, 1 = RTCP typically).
     */
    RtpConnectionInterleaved(RtspSession& session, int channel);

    void Send(ByteSpan data) override;

    /** Called by the session when channel data arrives on the TCP stream. */
    void OnChannelDataReceived(std::uint8_t channel,
                               const std::vector<std::uint8_t>& data);

  private:
    RtspSession* _session;
    int _channel;
  };

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_RTP_CONNECTION_INTERLEAVED_HPP
