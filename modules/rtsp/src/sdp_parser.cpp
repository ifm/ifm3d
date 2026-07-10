/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sdp_parser.hpp"

#include <cstddef>
#include <sstream>
#include <string>

#include <ifm3d/common/logging/log.h>

#include "rtsp_util.hpp"

namespace ifm3d::rtsp
{
  namespace
  {
    std::string
    strip_cr(std::string line)
    {
      if (!line.empty() && line.back() == '\r')
        {
          line.pop_back();
        }
      return line;
    }
  } // namespace

  std::string
  resolve_track_url(const std::string& sdp, const std::string& base_uri)
  {
    std::string track_control;
    bool in_media = false;

    std::istringstream ss(sdp);
    std::string line;
    while (std::getline(ss, line))
      {
        line = strip_cr(line);

        if (line.rfind("m=", 0) == 0)
          {
            in_media = true;
          }
        if (in_media && line.rfind("a=control:", 0) == 0)
          {
            track_control = line.substr(10);
            break;
          }
      }

    if (track_control.empty())
      {
        LOG_WARNING("SdpParser: no a=control in SDP, using base URI");
        return base_uri + "/";
      }

    if (track_control.rfind("rtsp://", 0) == 0 ||
        track_control.rfind("rtsps://", 0) == 0)
      {
        return track_control;
      }

    if (track_control == "*")
      {
        return base_uri;
      }

    const std::string sep = (base_uri.back() == '/') ? "" : "/";
    return base_uri + sep + track_control;
  }

  SdpInfo
  parse_sdp(const std::string& sdp, const std::string& base_uri)
  {
    SdpInfo info;
    info.track_url = resolve_track_url(sdp, base_uri);

    std::istringstream ss(sdp);
    std::string line;
    bool in_video = false;

    while (std::getline(ss, line))
      {
        line = strip_cr(line);

        if (line.rfind("m=", 0) == 0)
          {
            in_video = line.rfind("m=video", 0) == 0;
            continue;
          }

        if (!in_video)
          {
            continue;
          }

        // a=rtpmap:<pt> H264/90000
        if (line.rfind("a=rtpmap:", 0) == 0 &&
            to_lower(line).find("h264/90000") != std::string::npos)
          {
            info.has_h264 = true;
            const std::string rest = line.substr(9);
            try
              {
                info.payload_type = std::stoi(rest);
              }
            catch (...)
              {
                // IGNORE: malformed payload type left unset
              }
          }

        // a=fmtp:<pt> ...;sprop-parameter-sets=<base64,base64>;...
        if (line.rfind("a=fmtp:", 0) == 0)
          {
            const std::string key = "sprop-parameter-sets=";
            const std::size_t pos = line.find(key);
            if (pos != std::string::npos)
              {
                std::size_t const start = pos + key.size();
                std::size_t end = line.find(';', start);
                if (end == std::string::npos)
                  {
                    end = line.size();
                  }
                info.sprop_parameter_sets =
                  trim(line.substr(start, end - start));
              }
          }
      }

    return info;
  }

} // namespace ifm3d::rtsp
