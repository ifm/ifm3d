/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_DECODER_HOST_HPP
#define IFM3D_RTSP_DECODER_HOST_HPP

/** @file
 * @brief Bridges Annex-B H.264 access units to a compiled-in video decoder
 * and converts decoded frames into requested ifm3d buffers.
 */

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <ifm3d/fg/frame.h>
#include <ifm3d/rtsp/decoder_manager.h>
#include <ifm3d/rtsp/video_decoder.h>

namespace ifm3d::rtsp
{
  using BufferMap = std::map<ifm3d::buffer_id, ifm3d::Buffer>;

  /**
   * @brief Owns a DecoderManager and a single H.264 VideoDecoder, turning
   * Annex-B access units into decoded ifm3d::Buffer frames.
   */
  class DecoderHost
  {
  public:
    DecoderHost(std::optional<std::string> decoder,
                bool request_rgb,
                bool request_yuv);
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
    ProducesFrames() const
    {
      return _produces_frames;
    }

    /**
     * Submit one Annex-B access unit for decoding.
     *
     * @p pts is an opaque identifier for this access unit; it is handed to
     * the decoder and comes back on `on_frame` for the frame that access unit
     * produced. That frame is not necessarily produced by this call: decoders
     * are allowed to lag, and libavcodec does exactly that for streams whose
     * SPS advertises `max_num_reorder_frames > 0`. Callers must therefore key
     * their per-access-unit state off @p pts rather than off call ordering.
     */
    void SubmitAccessUnit(const std::vector<std::uint8_t>& access_unit,
                          std::uint64_t pts);

    /** Fired for each decoded frame, with the pts of its access unit. */
    std::function<void(BufferMap, std::uint64_t)> on_frame;

    /** Fired on decoder errors (with an ifm3d error code). */
    std::function<void(int, const std::string&)> on_error;

  private:
    [[nodiscard]] std::optional<ifm3d::Buffer> frame_to_rgb(
      const VideoFrame& frame) const;

    [[nodiscard]] std::optional<ifm3d::Buffer> frame_to_i420(
      const VideoFrame& frame) const;

    std::unique_ptr<DecoderManager> _decoder_manager;
    std::unique_ptr<VideoDecoder> _decoder;
    bool _request_rgb;
    bool _request_yuv;
    bool _produces_frames = false;
  };

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_DECODER_HOST_HPP
