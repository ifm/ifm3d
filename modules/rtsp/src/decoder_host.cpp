/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "decoder_host.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <ifm3d/common/err.h>
#include <ifm3d/common/json_impl.hpp>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/device.h>
#include <ifm3d/rtsp/decoder_manager.h>
#include <ifm3d/rtsp/frame_metadata.h>
#include <ifm3d/rtsp/rtsp_client.h>
#include <ifm3d/rtsp/video_decoder.h>

namespace ifm3d::rtsp
{
  namespace
  {
    std::uint8_t
    clamp8(int v)
    {
      return static_cast<std::uint8_t>(std::clamp(v, 0, 255));
    }
  } // namespace

  ifm3d::json
  rgb_info_to_json(const RgbInfo& info)
  {
    ifm3d::json j;
    j[frame_metadata::VERSION] = info.version;
    j[frame_metadata::FRAME_COUNTER] = info.frame_counter;
    j[frame_metadata::TIMESTAMP_NS] = info.timestamp_ns;
    j[frame_metadata::EXPOSURE_TIME] = info.exposure_time;
    j[frame_metadata::EXTRINSIC_OPTIC_TO_USER] = info.extrinsic_optic_to_user;
    j[frame_metadata::INTRINSIC_MODEL_ID] = info.intrinsic.model_id;
    j[frame_metadata::INTRINSIC_PARAMETERS] = info.intrinsic.model_parameters;
    j[frame_metadata::INVERSE_INTRINSIC_MODEL_ID] =
      info.inverse_intrinsic.model_id;
    j[frame_metadata::INVERSE_INTRINSIC_PARAMETERS] =
      info.inverse_intrinsic.model_parameters;
    return j;
  }

  DecoderHost::DecoderHost(std::optional<std::string> decoder,
                           ifm3d::RtspClient::OutputFormat output_format)
    : _decoder_manager(std::make_unique<DecoderManager>(std::move(decoder))),
      _output_format(output_format)
  {}

  DecoderHost::~DecoderHost() = default;

  bool
  DecoderHost::Init()
  {
    _decoder_manager->LoadDecoders();
    _decoder = _decoder_manager->CreateDecoder(VIDEO_CODEC_H264);
    if (_decoder == nullptr)
      {
        LOG_WARNING("DecoderHost: no decoder available for H.264; running in "
                    "NAL-only mode");
        return false;
      }

    LOG_INFO("DecoderHost: H.264 decoder ready");
    return true;
  }

  void
  DecoderHost::SubmitAccessUnit(const std::vector<std::uint8_t>& access_unit,
                                std::uint64_t pts_us,
                                const std::optional<ifm3d::json>& metadata)
  {
    (void)pts_us;
    if (_decoder == nullptr || access_unit.empty())
      {
        return;
      }

    const int rc = _decoder->SendPacket(access_unit.data(),
                                        static_cast<int>(access_unit.size()));
    if (rc < 0)
      {
        const std::string detail = _decoder->LastError();
        LOG_WARNING("DecoderHost: send_packet failed ({}): {}",
                    rc,
                    detail.empty() ? "unknown error" : detail);
        if (on_error)
          {
            on_error(IFM3D_DECODE_ERROR,
                     detail.empty() ? "send_packet failed" : detail);
          }
        return;
      }

    VideoFrame frame{};
    int recv = 0;
    while ((recv = _decoder->ReceiveFrame(frame)) == 1)
      {
        if (auto buffer = convert_frame(frame, metadata))
          {
            if (on_frame)
              {
                on_frame(std::move(*buffer));
              }
          }
      }

    if (recv < 0)
      {
        const std::string detail = _decoder->LastError();
        LOG_WARNING("DecoderHost: receive_frame failed ({}): {}",
                    recv,
                    detail.empty() ? "unknown error" : detail);
        if (on_error)
          {
            on_error(IFM3D_DECODE_ERROR,
                     detail.empty() ? "receive_frame failed" : detail);
          }
      }
  }

  std::optional<ifm3d::Buffer>
  DecoderHost::convert_frame(const VideoFrame& frame,
                             const std::optional<ifm3d::json>& metadata) const
  {
    if (frame.format != VIDEO_FORMAT_YUV420P || frame.width <= 0 ||
        frame.height <= 0 || frame.planes[0] == nullptr ||
        frame.planes[1] == nullptr || frame.planes[2] == nullptr)
      {
        LOG_WARNING("DecoderHost: unsupported or invalid decoded frame");
        return std::nullopt;
      }

    return _output_format == ifm3d::RtspClient::OutputFormat::RGB ?
             frame_to_rgb(frame, metadata) :
             frame_to_i420(frame, metadata);
  }

  std::optional<ifm3d::Buffer>
  DecoderHost::frame_to_rgb(const VideoFrame& frame,
                            const std::optional<ifm3d::json>& metadata) const
  {
    const int width = frame.width;
    const int height = frame.height;
    const int stride_y = frame.linesize[0];
    const int stride_uv = frame.linesize[1];

    ifm3d::Buffer buffer(static_cast<std::uint32_t>(width),
                         static_cast<std::uint32_t>(height),
                         3,
                         ifm3d::PixelFormat::FORMAT_8U,
                         metadata);

    const std::uint8_t* y = frame.planes[0];
    const std::uint8_t* u = frame.planes[1];
    const std::uint8_t* v = frame.planes[2];

    for (int row = 0; row < height; ++row)
      {
        auto* dst = buffer.Ptr<std::uint8_t>(static_cast<std::uint32_t>(row));
        for (int col = 0; col < width; ++col)
          {
            const int y_val = y[(row * stride_y) + col];
            const int u_val = u[((row / 2) * stride_uv) + (col / 2)] - 128;
            const int v_val = v[((row / 2) * stride_uv) + (col / 2)] - 128;

            const int r = y_val + ((91881 * v_val) >> 16);
            const int g =
              y_val - ((22554 * u_val) >> 16) - ((46802 * v_val) >> 16);
            const int b = y_val + ((116130 * u_val) >> 16);

            const std::size_t off = static_cast<std::size_t>(col) * 3;
            dst[off + 0] = clamp8(r);
            dst[off + 1] = clamp8(g);
            dst[off + 2] = clamp8(b);
          }
      }

    return buffer;
  }

  std::optional<ifm3d::Buffer>
  DecoderHost::frame_to_i420(const VideoFrame& frame,
                             const std::optional<ifm3d::json>& metadata) const
  {
    // Deliver the raw YUV420P data in planar I420 layout: a single-channel
    // 8-bit buffer of width x (height * 3 / 2), with the full-resolution Y
    // plane followed by the half-resolution U and V planes. This is the
    // layout consumed by e.g. cv2.cvtColor(..., COLOR_YUV2BGR_I420).
    const int width = frame.width;
    const int height = frame.height;
    const int chroma_width = (width + 1) / 2;
    const int chroma_height = (height + 1) / 2;

    // The Y plane is (width x height); each chroma plane is
    // (chroma_width x chroma_height). Packed contiguously into a width-wide
    // buffer the chroma planes occupy
    // (2 * chroma_width * chroma_height) / width rows.
    const std::size_t total_bytes =
      (static_cast<std::size_t>(width) * height) +
      (2U * static_cast<std::size_t>(chroma_width) * chroma_height);
    const auto buffer_height = static_cast<std::uint32_t>(
      (total_bytes + width - 1) / static_cast<std::size_t>(width));

    ifm3d::Buffer buffer(static_cast<std::uint32_t>(width),
                         buffer_height,
                         1,
                         ifm3d::PixelFormat::FORMAT_8U,
                         metadata);

    auto* dst = buffer.Ptr<std::uint8_t>(0);
    std::size_t offset = 0;

    const auto copy_plane = [&](const std::uint8_t* src,
                                int stride,
                                int plane_width,
                                int plane_height) {
      for (int row = 0; row < plane_height; ++row)
        {
          std::copy_n(src + (static_cast<std::size_t>(row) * stride),
                      plane_width,
                      dst + offset);
          offset += static_cast<std::size_t>(plane_width);
        }
    };

    copy_plane(frame.planes[0], frame.linesize[0], width, height);
    copy_plane(frame.planes[1],
               frame.linesize[1],
               chroma_width,
               chroma_height);
    copy_plane(frame.planes[2],
               frame.linesize[2],
               chroma_width,
               chroma_height);

    return buffer;
  }

} // namespace ifm3d::rtsp
