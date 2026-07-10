/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtp_client.hpp"

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include <ifm3d/common/logging/log.h>

#include "bit_reader_writer.hpp"
#include "package_decoder.hpp"
#include "rtp_connection.hpp"

namespace ifm3d::rtsp
{
  void
  RtpClient::InitConnection(RtpConnectionSP connection)
  {
    if (_connection)
      {
        _connection->on_data_received = nullptr;
      }

    _connection = std::move(connection);
    _connection->on_data_received = [this](std::vector<std::uint8_t> data) {
      process_packet(std::move(data));
    };
  }

  void
  RtpClient::RegisterDecoder(int payload_type, PackageDecoderSP decoder)
  {
    _decoders[payload_type] = std::move(decoder);
  }

  void
  RtpClient::process_packet(std::vector<std::uint8_t> data)
  {
    if (data.size() < 12)
      {
        LOG_WARNING("RtpClient: packet too short, discarding");
        return;
      }

    const BitReader reader(data.data());

    const std::uint16_t cur_seq_num = reader.Read<rtp_header::seq_num>();

    if (!_initialized)
      {
        _initialized = true;
        _last_seq_num =
          static_cast<int>(static_cast<std::uint16_t>(cur_seq_num - 1U));
      }

    const auto seq_delta =
      static_cast<std::uint16_t>(cur_seq_num - _last_seq_num);

    if (seq_delta == 0)
      {
        // Duplicate packet.
        return;
      }

    if (reader.Read<rtp_header::p>())
      {
        LOG_WARNING("RtpClient: RTP padding not supported, discarding");
        return;
      }

    std::size_t header_size =
      12U + (static_cast<std::size_t>(reader.Read<rtp_header::cc>()) * 4U);

    if (reader.Read<rtp_header::x>())
      {
        // RTP header extension (RFC 3550 5.3.1): a 4-byte extension header
        // (2 bytes profile-defined + a 16-bit length giving the number of
        // 32-bit words that follow), then length * 4 bytes of extension data.
        const std::size_t ext_header_offset = header_size;
        if (data.size() < ext_header_offset + 4U)
          {
            LOG_WARNING("RtpClient: truncated RTP extension header, "
                        "discarding");
            return;
          }
        const std::uint16_t ext_words =
          big_endian::read_u_int16(data.data() + ext_header_offset + 2U);
        header_size += 4U + (static_cast<std::size_t>(ext_words) * 4U);
      }

    if (seq_delta != 1)
      {
        _frame_valid = false;
      }
    _last_seq_num = cur_seq_num;

    const std::uint8_t payload_type = reader.Read<rtp_header::pt>();

    auto it = _decoders.find(payload_type);
    if (it == _decoders.end())
      {
        LOG_VERBOSE("RtpClient: no decoder for payload type {}", payload_type);
        return;
      }

    PackageDecoderSP const& decoder = it->second;

    if (_frame_valid)
      {
        _last_rtp_timestamp = reader.Read<rtp_header::timestamp>();

        if (header_size >= data.size())
          {
            LOG_WARNING("RtpClient: header size exceeds packet length");
            return;
          }

        std::vector<std::uint8_t> const payload(
          data.begin() + static_cast<std::ptrdiff_t>(header_size),
          data.end());

        decoder->DecodePackage(payload, _last_rtp_timestamp, cur_seq_num);
      }

    if (reader.Read<rtp_header::m>())
      {
        if (_frame_valid)
          {
            decoder->FinishFrame();
          }
        else
          {
            decoder->CancelFrame();
          }
        _frame_valid = true;
      }
  }

} // namespace ifm3d::rtsp
