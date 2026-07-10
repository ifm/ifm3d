/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_NAL_UNIT_H
#define IFM3D_RTSP_NAL_UNIT_H

#include <cstdint>
#include <vector>

#include <ifm3d/rtsp/module_rtsp.h>

namespace ifm3d
{
  /** @ingroup RTSP
   *
   * @brief A single, fully reassembled H.264 NAL unit.
   *
   * A `NalUnit` is emitted for every complete NAL unit produced by the RTP
   * depacketizer, after FU-A fragments have been reassembled and STAP-A
   * aggregations have been expanded into individual units. The payload is the
   * raw NAL byte stream **without** an Annex-B start code; the first byte is
   * the NAL header (`forbidden_zero_bit`, `nal_ref_idc`, `nal_unit_type`).
   *
   * NAL units are delivered to the `RtspClient::OnNalUnit` callback before the
   * decoder host is invoked, allowing callers to inspect the raw bitstream
   * even when decoding is disabled.
   */
  struct IFM3D_EXPORT NalUnit
  {
    /** H.264 NAL unit type values (ITU-T H.264 Table 7-1, RFC 6184 §1.3). */
    enum Type : std::uint8_t
    {
      NON_IDR_SLICE = 1,
      IDR_SLICE = 5,
      SEI = 6,
      SPS = 7,
      PPS = 8,
      ACCESS_UNIT_DELIMITER = 9,
    };

    /**
     * Raw NAL payload, NAL-header byte first, without an Annex-B start code
     * and with emulation-prevention bytes still present.
     */
    std::vector<std::uint8_t> data;

    /** `nal_ref_idc` field from the NAL header (0-3). */
    std::uint8_t nal_ref_idc = 0;

    /** `nal_unit_type` field from the NAL header (1-31). */
    std::uint8_t nal_unit_type = 0;

    /** Presentation timestamp derived from the RTP timestamp, in microseconds.
     */
    std::uint64_t pts_us = 0;

    /** True when this NAL unit is an IDR slice (`nal_unit_type == 5`). */
    bool is_idr = false;

    /** RTP sequence number of the first packet this NAL originated from. */
    std::uint16_t first_sequence_number = 0;

    /** RTP sequence number of the last packet this NAL originated from. */
    std::uint16_t last_sequence_number = 0;

    /** @return the number of bytes in the NAL payload. */
    [[nodiscard]] std::size_t
    Size() const
    {
      return data.size();
    }
  };

} // namespace ifm3d

#endif // IFM3D_RTSP_NAL_UNIT_H
