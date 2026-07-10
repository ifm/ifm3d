/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_NULL_DECODER_HPP
#define IFM3D_RTSP_NULL_DECODER_HPP

/** @file
 * @brief The "null" decoder.
 *
 * The null decoder implements the VideoDecoderFactory/VideoDecoder interface
 * but discards every packet and never produces a decoded frame. It exists as
 * an always-available fallback so the RTSP pipeline can run in NAL-only mode
 * through the regular decoder path.
 *
 * It is only selected when no other decoder supports the requested codec or
 * when it has been requested explicitly via its name.
 */

#include <memory>

#include <ifm3d/rtsp/video_decoder.h>

namespace ifm3d::rtsp
{
  /** Name used to request the null decoder explicitly. */
  inline constexpr const char* NULL_DECODER_NAME = "null";

  /** @return a new instance of the null decoder factory. */
  std::unique_ptr<VideoDecoderFactory> make_null_decoder_factory();

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_NULL_DECODER_HPP
