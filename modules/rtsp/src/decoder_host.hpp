/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_DECODER_HOST_HPP
#define IFM3D_RTSP_DECODER_HOST_HPP

/** @file
 * @brief Bridges Annex-B H.264 access units to a compiled-in video decoder
 * and converts decoded frames into ifm3d::Buffer instances.
 */

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <ifm3d/common/json.hpp>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/rtsp/decoder_manager.h>
#include <ifm3d/rtsp/frame_metadata.h>
#include <ifm3d/rtsp/rtsp_client.h>
#include <ifm3d/rtsp/video_decoder.h>

namespace ifm3d::rtsp
{
  /** Convert a parsed RGB_INFO SEI structure into Buffer JSON metadata. */
  ifm3d::json rgb_info_to_json(const RgbInfo& info);

  /**
   * @brief Owns a DecoderManager and a single H.264 VideoDecoder, turning
   * Annex-B access units into decoded ifm3d::Buffer frames.
   */
  class DecoderHost
  {
  public:
    DecoderHost(std::optional<std::string> decoder,
                ifm3d::RtspClient::OutputFormat output_format =
                  ifm3d::RtspClient::OutputFormat::RGB);
    ~DecoderHost();

    DecoderHost(const DecoderHost&) = delete;
    DecoderHost& operator=(const DecoderHost&) = delete;
    DecoderHost(DecoderHost&&) = delete;
    DecoderHost& operator=(DecoderHost&&) = delete;

    /**
     * Select and create a decoder for H.264.
     * @return true if a decoder is available, false otherwise.
     */
    bool Init();

    /** @return true if a decoder was successfully created. */
    [[nodiscard]] bool
    HasDecoder() const
    {
      return _decoder != nullptr;
    }

    /**
     * Submit one Annex-B access unit for decoding. Any frames produced are
     * delivered through `on_frame`, carrying @p metadata (if any).
     */
    void SubmitAccessUnit(const std::vector<std::uint8_t>& access_unit,
                          std::uint64_t pts_us,
                          const std::optional<ifm3d::json>& metadata);

    /** Fired for each decoded frame. */
    std::function<void(ifm3d::Buffer)> on_frame;

    /** Fired on decoder errors (with an ifm3d error code). */
    std::function<void(int, const std::string&)> on_error;

  private:
    [[nodiscard]] std::optional<ifm3d::Buffer> convert_frame(
      const VideoFrame& frame,
      const std::optional<ifm3d::json>& metadata) const;

    [[nodiscard]] std::optional<ifm3d::Buffer> frame_to_rgb(
      const VideoFrame& frame,
      const std::optional<ifm3d::json>& metadata) const;

    [[nodiscard]] std::optional<ifm3d::Buffer> frame_to_i420(
      const VideoFrame& frame,
      const std::optional<ifm3d::json>& metadata) const;

    std::unique_ptr<DecoderManager> _decoder_manager;
    std::unique_ptr<VideoDecoder> _decoder;
    ifm3d::RtspClient::OutputFormat _output_format =
      ifm3d::RtspClient::OutputFormat::RGB;
  };

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_DECODER_HOST_HPP
