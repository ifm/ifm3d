/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_RTP_CLIENT_HPP
#define IFM3D_RTSP_RTP_CLIENT_HPP

/** @file
 * @brief RTP packet demultiplexer and payload dispatcher (RFC 3550).
 */

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include "bit_reader_writer.hpp"
#include "package_decoder.hpp"
#include "rtp_connection.hpp"

namespace ifm3d::rtsp
{
  /** RTP fixed-header field descriptors (RFC 3550). */
  namespace rtp_header
  {
    using v = BitField<0, 2>;
    using p = BitField<2, 1>;
    using x = BitField<3, 1>;
    using cc = BitField<4, 4>;
    using m = BitField<8, 1>;
    using pt = BitField<9, 7>;
    using seq_num = BitField<16, 16>;
    using timestamp = BitField<32, 32>;
    using ssrc = BitField<64, 32>;
  } // namespace rtp_header

  /**
   * @brief Parses incoming RTP packets, detects sequence gaps, and
   * dispatches payloads to the registered `PackageDecoder`.
   */
  class RtpClient
  {
  public:
    RtpClient() = default;

    /** Attach (or replace) the transport connection. */
    void InitConnection(RtpConnectionSP connection);

    /** Register a depacketizer for a specific RTP payload type. */
    void RegisterDecoder(int payload_type, PackageDecoderSP decoder);

    /** Fires on transport or protocol errors. */
    std::function<void(int)> on_error;

  private:
    void process_packet(std::vector<std::uint8_t> data);

    std::unordered_map<int, PackageDecoderSP> _decoders;
    RtpConnectionSP _connection;

    int _last_seq_num = 0;
    bool _initialized = false;
    bool _frame_valid = false;
    std::uint32_t _last_rtp_timestamp = 0;
  };

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_RTP_CLIENT_HPP
