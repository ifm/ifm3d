/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_FFMPEG_LIB_HPP
#define IFM3D_RTSP_FFMPEG_LIB_HPP

/** @file
 * @brief Runtime loader for libavcodec/libavutil.
 *
 * The ffmpeg decoder is compiled into ifm3d_rtsp but does NOT link against
 * libavcodec/libavutil. Instead the required functions are resolved at runtime
 * via dlopen/dlsym (LoadLibrary/GetProcAddress on Windows), trying a range of
 * library SONAMEs. This lets a single ifm3d binary run against whichever
 * FFmpeg major version happens to be installed (e.g. Ubuntu 22.04 ships
 * libavcodec.so.58, Ubuntu 24.04 ships libavcodec.so.60), and to gracefully
 * report "unavailable" (falling back to the null decoder) when no compatible
 * FFmpeg is present.
 *
 * ABI note: only the long-stable front fields of AVFrame (data, linesize,
 * width, height, format) and AVPacket (data, size) are accessed, and all
 * AV* structs are allocated by the loaded library itself (av_*_alloc), so no
 * struct-size or field-offset assumptions are made beyond those stable fields.
 */

#include <string>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
}

namespace ifm3d::rtsp
{
  /**
   * Resolved libavcodec/libavutil entry points. Function pointer types are
   * taken directly from the FFmpeg headers so the signatures always match.
   */
  struct FfmpegApi
  {
    /* libavcodec */
    decltype(&::avcodec_find_decoder) avcodec_find_decoder = nullptr;
    decltype(&::avcodec_alloc_context3) avcodec_alloc_context3 = nullptr;
    decltype(&::avcodec_open2) avcodec_open2 = nullptr;
    decltype(&::avcodec_free_context) avcodec_free_context = nullptr;
    decltype(&::avcodec_send_packet) avcodec_send_packet = nullptr;
    decltype(&::avcodec_receive_frame) avcodec_receive_frame = nullptr;
    decltype(&::av_packet_alloc) av_packet_alloc = nullptr;
    decltype(&::av_packet_free) av_packet_free = nullptr;
    decltype(&::av_packet_unref) av_packet_unref = nullptr;
    decltype(&::av_new_packet) av_new_packet = nullptr;

    /* libavutil */
    decltype(&::av_frame_alloc) av_frame_alloc = nullptr;
    decltype(&::av_frame_free) av_frame_free = nullptr;
    decltype(&::av_strerror) av_strerror = nullptr;

    /* True when every required function was resolved. */
    bool available = false;
    /* Human-readable reason when available is false; empty otherwise. */
    std::string error;
  };

  /**
   * Lazily load libavcodec/libavutil and resolve the required entry points.
   * The result is cached; the returned reference is valid for the program's
   * lifetime and the libraries are never unloaded.
   */
  const FfmpegApi& ffmpeg_api();

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_FFMPEG_LIB_HPP
