/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_FFMPEG_DECODER_HPP
#define IFM3D_RTSP_FFMPEG_DECODER_HPP

/** @file
 * @brief FFmpeg-based H.264 decoder.
 *
 * Compiled into ifm3d_rtsp when BUILD_MODULE_RTSP_FFMPEG is enabled.
 * libavcodec/libavutil are resolved at runtime (see ffmpeg_lib.hpp), so this
 * decoder does not add a link-time dependency and reports itself unavailable
 * when no compatible FFmpeg is installed.
 */

#include <memory>

#include <ifm3d/rtsp/video_decoder.h>

namespace ifm3d::rtsp
{
  /** Name used to request the ffmpeg decoder explicitly. */
  inline constexpr const char* FFMPEG_DECODER_NAME = "ffmpeg";

  /** @return a new instance of the ffmpeg decoder factory. */
  std::unique_ptr<VideoDecoderFactory> make_ffmpeg_decoder_factory();

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_FFMPEG_DECODER_HPP
