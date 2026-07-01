/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "h264_depacketizer.hpp"

#include <array>
#include <cstdint>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <ifm3d/common/logging/log.h>
#include <ifm3d/rtsp/nal_unit.h>

#include "bit_reader_writer.hpp"
#include "rtsp_util.hpp"
#include "sei_parser.hpp"

namespace ifm3d::rtsp
{
  namespace
  {
    constexpr std::array<std::uint8_t, 4> START_CODE = {0x00,
                                                        0x00,
                                                        0x00,
                                                        0x01};

    std::uint64_t
    rtp_to_micros(std::uint32_t rtp_timestamp)
    {
      // H.264 RTP clock rate is 90 kHz.
      return static_cast<std::uint64_t>(rtp_timestamp) * 1000000ULL / 90000ULL;
    }
  } // namespace

  void
  H264Depacketizer::DecodePackage(const std::vector<std::uint8_t>& package,
                                  std::uint32_t rtp_timestamp,
                                  std::uint16_t sequence_number)
  {
    if (package.empty())
      {
        return;
      }

    _current_rtp_timestamp = rtp_timestamp;

    const BitReader nal(package.data());
    bool frame_end = false;

    _current_packet_type = nal.Read<nal_header::type>();

    if (_current_packet_type == NAL_STAP_A)
      {
        _current_frame.assign(package.begin() + 1, package.end());
        _current_read_index = 0;
        _current_first_sequence = sequence_number;
        _current_last_sequence = sequence_number;
        frame_end = true;
      }
    else if (_current_packet_type == NAL_STAP_B ||
             _current_packet_type == NAL_MTAP_16 ||
             _current_packet_type == NAL_MTAP_24 ||
             _current_packet_type == NAL_FU_B)
      {
        LOG_WARNING("H264Depacketizer: unsupported packetization type {}",
                    _current_packet_type);
        return;
      }
    else if (_current_packet_type == NAL_FU_A)
      {
        if (package.size() < 2)
          {
            return;
          }
        const BitReader fu(package.data() + 1);

        if (fu.Read<fu_header::s>())
          {
            // First fragment: reconstruct the original NAL header byte.
            std::uint8_t hdr = 0;
            BitWriter w(&hdr);
            w.Write<nal_header::f>(nal.Read<nal_header::f>());
            w.Write<nal_header::nri>(nal.Read<nal_header::nri>());
            w.Write<nal_header::type>(fu.Read<fu_header::type>());
            _current_frame.clear();
            _current_frame.push_back(hdr);
            _current_read_index = 0;
            _fu_first_sequence = sequence_number;
          }
        else if (_current_frame.empty())
          {
            LOG_DEBUG(
              "H264Depacketizer: FU-A continuation without start, dropping");
            return;
          }

        _current_frame.insert(_current_frame.end(),
                              package.begin() + 2,
                              package.end());
        frame_end = fu.Read<fu_header::e>();
        _current_first_sequence = _fu_first_sequence;
        _current_last_sequence = sequence_number;
      }
    else
      {
        // Single NAL unit packet (types 1-23).
        _current_frame = package;
        _current_read_index = 0;
        _current_first_sequence = sequence_number;
        _current_last_sequence = sequence_number;
        frame_end = true;
      }

    if (!frame_end)
      {
        return;
      }

    int nal_size = 0;
    while (const std::uint8_t* nal_unit = next_nal_unit(nal_size))
      {
        emit_nal(nal_unit, nal_size);
      }

    _current_frame.clear();
    _current_read_index = 0;
  }

  void
  H264Depacketizer::emit_nal(const std::uint8_t* nal, int size)
  {
    if (size <= 0)
      {
        return;
      }

    const std::uint8_t header = nal[0];
    const std::uint8_t nal_ref_idc = (header >> 5) & 0x03;
    const std::uint8_t nal_unit_type = header & 0x1F;

    NalUnit unit;
    unit.data.assign(nal, nal + size);
    unit.nal_ref_idc = nal_ref_idc;
    unit.nal_unit_type = nal_unit_type;
    unit.pts_us = rtp_to_micros(_current_rtp_timestamp);
    unit.is_idr = (nal_unit_type == NalUnit::IDR_SLICE);
    unit.first_sequence_number = _current_first_sequence;
    unit.last_sequence_number = _current_last_sequence;

    if (on_nal_unit)
      {
        on_nal_unit(unit);
      }

    // SEI (type 6): parse for unregistered user data before forwarding.
    if (nal_unit_type == NalUnit::SEI && on_sei_unregistered_user_data)
      {
        parse_sei_nal(nal + 1, size - 1, on_sei_unregistered_user_data);
      }

    // Prepend out-of-band SPS/PPS ahead of the first emitted access unit.
    if (!_sprop_prefix.empty() && _access_unit.empty() &&
        !_access_unit_has_sprop)
      {
        _access_unit.insert(_access_unit.end(),
                            _sprop_prefix.begin(),
                            _sprop_prefix.end());
        _access_unit_has_sprop = true;
      }

    _access_unit.insert(_access_unit.end(),
                        std::begin(START_CODE),
                        std::end(START_CODE));
    _access_unit.insert(_access_unit.end(), nal, nal + size);

    // Flush a complete access unit after a VCL slice NAL (IDR or non-IDR).
    if (nal_unit_type == NalUnit::IDR_SLICE ||
        nal_unit_type == NalUnit::NON_IDR_SLICE)
      {
        if (on_access_unit)
          {
            on_access_unit(_access_unit,
                           rtp_to_micros(_current_rtp_timestamp));
          }
        _access_unit.clear();
      }
  }

  const std::uint8_t*
  H264Depacketizer::next_nal_unit(int& size)
  {
    const std::uint8_t* ptr = _current_frame.data() + _current_read_index;
    const int remaining =
      static_cast<int>(_current_frame.size()) - _current_read_index;
    int off = 0;

    if (_current_packet_type == NAL_STAP_A)
      {
        if (remaining < 2)
          {
            size = 0;
            return nullptr;
          }
        size = (ptr[0] << 8) | ptr[1];
        off = 2;
      }
    else
      {
        size = remaining;
      }

    if (size <= 0 || off + size > remaining)
      {
        size = 0;
        return nullptr;
      }

    _current_read_index += off + size;
    return ptr + off;
  }

  void
  H264Depacketizer::FinishFrame()
  {
    // Access-unit boundaries are determined within DecodePackage by the
    // FU-A end bit and VCL slice detection.
  }

  void
  H264Depacketizer::CancelFrame()
  {
    _current_frame.clear();
    _current_read_index = 0;
    _access_unit.clear();
  }

  void
  H264Depacketizer::SeedFromSprop(const std::string& sprop_parameter_sets)
  {
    std::istringstream ss(sprop_parameter_sets);
    std::string token;
    int count = 0;
    while (std::getline(ss, token, ','))
      {
        token = trim(token);
        if (token.empty())
          {
            continue;
          }

        const std::vector<std::uint8_t> nal = base64_decode(token);
        if (nal.empty())
          {
            continue;
          }

        _sprop_prefix.insert(_sprop_prefix.end(),
                             std::begin(START_CODE),
                             std::end(START_CODE));
        _sprop_prefix.insert(_sprop_prefix.end(), nal.begin(), nal.end());

        // Also surface the parameter set as a NalUnit for inspection.
        const std::uint8_t header = nal[0];
        NalUnit unit;
        unit.data = nal;
        unit.nal_ref_idc = (header >> 5) & 0x03;
        unit.nal_unit_type = header & 0x1F;
        if (on_nal_unit)
          {
            on_nal_unit(unit);
          }

        ++count;
      }

    if (count > 0)
      {
        LOG_INFO("H264Depacketizer: seeded {} parameter set(s) from SDP",
                 count);
      }
  }

} // namespace ifm3d::rtsp
