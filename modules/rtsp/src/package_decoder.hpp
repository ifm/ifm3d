/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_PACKAGE_DECODER_HPP
#define IFM3D_RTSP_PACKAGE_DECODER_HPP

/** @file
 * @brief Abstract base class for RTP payload depacketizers.
 */

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include <ifm3d/rtsp/nal_unit.h>

namespace ifm3d::rtsp
{
  /**
   * @brief Reassembles RTP payloads into codec access units.
   *
   * The RTP client feeds payloads to a `PackageDecoder` registered for a
   * given RTP payload type. The decoder emits reassembled NAL units through
   * `on_nal_unit` and complete Annex-B access units through
   * `on_access_unit`.
   */
  class PackageDecoder
  {
  public:
    PackageDecoder() = default;
    virtual ~PackageDecoder() = default;

    PackageDecoder(const PackageDecoder&) = delete;
    PackageDecoder& operator=(const PackageDecoder&) = delete;
    PackageDecoder(PackageDecoder&&) = delete;
    PackageDecoder& operator=(PackageDecoder&&) = delete;

    /** Called for each RTP payload belonging to the current frame. */
    virtual void DecodePackage(const std::vector<std::uint8_t>& package,
                               std::uint32_t rtp_timestamp,
                               std::uint16_t sequence_number) = 0;

    /** Called when the RTP marker bit signals end of frame. */
    virtual void FinishFrame() = 0;

    /** Called when a sequence gap invalidates the current frame. */
    virtual void CancelFrame() = 0;

    /** Fired for every fully reassembled NAL unit. */
    std::function<void(const ifm3d::NalUnit&)> on_nal_unit;

    /** Fired with a complete Annex-B access unit and its presentation
     *  timestamp in microseconds. */
    std::function<void(const std::vector<std::uint8_t>&, std::uint64_t)>
      on_access_unit;
  };

  using PackageDecoderSP = std::shared_ptr<PackageDecoder>;

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_PACKAGE_DECODER_HPP
