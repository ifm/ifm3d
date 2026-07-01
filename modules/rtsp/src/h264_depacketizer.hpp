/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_H264_DEPACKETIZER_HPP
#define IFM3D_RTSP_H264_DEPACKETIZER_HPP

/** @file
 * @brief RTP H.264 depacketizer (RFC 6184).
 *
 * Reassembles Single NAL, STAP-A and FU-A packets into NAL units and
 * Annex-B access units. No decoding is performed here; decoding is delegated
 * to a compiled-in video decoder via the DecoderHost.
 */

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "bit_reader_writer.hpp"
#include "package_decoder.hpp"

namespace ifm3d::rtsp
{
  /** H.264 RTP NAL unit types (RFC 6184 §5.2). */
  enum NalPacketType : std::uint8_t
  {
    // 1-23 = single NAL unit packets
    NAL_STAP_A = 24,
    NAL_STAP_B = 25,
    NAL_MTAP_16 = 26,
    NAL_MTAP_24 = 27,
    NAL_FU_A = 28,
    NAL_FU_B = 29,
  };

  namespace nal_header
  {
    using f = BitField<0, 1>;   // forbidden_zero_bit
    using nri = BitField<1, 2>; // nal_ref_idc
    using type = BitField<3, 5>;
  } // namespace nal_header

  namespace fu_header
  {
    using s = BitField<0, 1>;    // start
    using e = BitField<1, 1>;    // end
    using r = BitField<2, 1>;    // reserved
    using type = BitField<3, 5>; // NAL unit type
  }                              // namespace fu_header

  /**
   * @brief Depacketizes RTP H.264 payloads into NAL units and Annex-B
   * access units.
   */
  class H264Depacketizer : public PackageDecoder
  {
  public:
    H264Depacketizer() = default;

    void DecodePackage(const std::vector<std::uint8_t>& package,
                       std::uint32_t rtp_timestamp,
                       std::uint16_t sequence_number) override;
    void FinishFrame() override;
    void CancelFrame() override;

    /**
     * Pre-seed the depacketizer with out-of-band SPS/PPS from the SDP
     * `a=fmtp sprop-parameter-sets` value (comma-separated base64 NALs).
     * The parameter sets are prepended to the first access unit.
     */
    void SeedFromSprop(const std::string& sprop_parameter_sets);

    /** Fired for every SEI unregistered-user-data message (payloadType 5). */
    std::function<void(const std::array<std::uint8_t, 16>& uuid,
                       const std::vector<std::uint8_t>& data)>
      on_sei_unregistered_user_data;

  private:
    const std::uint8_t* next_nal_unit(int& size);
    void emit_nal(const std::uint8_t* nal, int size);

    std::vector<std::uint8_t> _current_frame;
    int _current_read_index = 0;
    int _current_packet_type = 0;

    std::vector<std::uint8_t> _access_unit;
    bool _access_unit_has_sprop = false;
    std::vector<std::uint8_t> _sprop_prefix;

    std::uint32_t _current_rtp_timestamp = 0;
    std::uint16_t _fu_first_sequence = 0;
    std::uint16_t _current_first_sequence = 0;
    std::uint16_t _current_last_sequence = 0;
  };

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_H264_DEPACKETIZER_HPP
