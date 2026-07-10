/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_SDP_PARSER_HPP
#define IFM3D_RTSP_SDP_PARSER_HPP

/** @file
 * @brief Minimal SDP (RFC 4566) parsing for locating an H.264 video track.
 */

#include <string>

namespace ifm3d::rtsp
{
  /** Information extracted from an SDP description for an H.264 track. */
  struct SdpInfo
  {
    /** SETUP/track control URL (resolved against the base URI). */
    std::string track_url;

    /** Dynamic RTP payload type of the H.264 track (RFC 6184), e.g. 96. */
    int payload_type = 96;

    /** `sprop-parameter-sets` from the `a=fmtp` line, if present. */
    std::string sprop_parameter_sets;

    /** True when an `a=rtpmap ... H264/90000` track was found. */
    bool has_h264 = false;
  };

  /**
   * Parse an SDP body and resolve the first H.264 video track.
   *
   * @param sdp      Raw SDP description text.
   * @param base_uri RTSP base URI used to resolve relative `a=control`.
   * @return parsed track information.
   */
  SdpInfo parse_sdp(const std::string& sdp, const std::string& base_uri);

  /**
   * Resolve the first media track's control URL against @p base_uri,
   * following RFC 2326 §C.1.1 (absolute, relative, or aggregate `*`).
   */
  std::string resolve_track_url(const std::string& sdp,
                                const std::string& base_uri);

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_SDP_PARSER_HPP
