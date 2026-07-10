/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_RTP_CONNECTION_HPP
#define IFM3D_RTSP_RTP_CONNECTION_HPP

/** @file
 * @brief Abstract RTP/RTCP transport interface.
 *
 * Hides whether the underlying transport is UDP or interleaved TCP (RTSP).
 */

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "byte_span.hpp"

namespace ifm3d::rtsp
{
  /**
   * @brief Transport-agnostic RTP/RTCP channel.
   *
   * Concrete subclasses move bytes over UDP or the interleaved RTSP TCP
   * connection and notify the owner through `on_data_received` whenever a
   * complete datagram / interleaved packet arrives.
   */
  class RtpConnection
  {
  public:
    virtual ~RtpConnection() = default;

    virtual void Send(ByteSpan data) = 0;

    /** Called whenever a complete datagram / interleaved packet arrives. */
    std::function<void(std::vector<uint8_t>)> on_data_received;
  };

  using RtpConnectionSP = std::shared_ptr<RtpConnection>;

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_RTP_CONNECTION_HPP
