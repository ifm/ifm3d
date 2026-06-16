/*
 * Copyright (C) 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 *
 * ifm3d video plugin C-ABI  —  ABI version 1
 *
 * This header is intentionally pure C and self-contained so that plugins
 * written in C, C++, or any language with a C FFI can use it without
 * pulling in any ifm3d or C++ headers.
 *
 * Thread-safety contract
 * ----------------------
 * All C-ABI calls (send_packet, receive_frame, flush, …) for a given
 * VideoDecoder* instance MUST be made from a single thread at a time.
 * Plugins are not required to be thread-safe across instances.
 *
 * Buffer ownership
 * ----------------
 * - Buffers passed to send_packet are owned by the caller and are only
 *   valid for the duration of that call; plugins must copy if they retain.
 * - VideoFrame pointers returned by receive_frame are owned by the decoder
 *   and remain valid until the next call to receive_frame, flush, or
 *   destroy_decoder on the same instance.
 */

#ifndef IFM3D_RTSP_PLUGIN_H
#define IFM3D_RTSP_PLUGIN_H

#include <stdint.h>

#define VIDEO_PLUGIN_ABI_VERSION 1

#ifdef _WIN32
#  define PLUGIN_EXPORT __declspec(dllexport)
#else
#  define PLUGIN_EXPORT __attribute__((visibility("default")))
#endif

/* Opaque decoder handle — defined by each plugin implementation. */
typedef struct VideoDecoder VideoDecoder;

/* Supported video codecs.
 * New values may be appended in future ABI versions without breaking v1
 * plugins; unknown values must be rejected by create_decoder. */
typedef enum
{
  VIDEO_CODEC_H264 = 1
} VideoCodec;

/* Pixel formats for decoded frames.
 * New values may be appended in future ABI versions. */
typedef enum
{
  VIDEO_FORMAT_YUV420P = 1
} VideoFormat;

/* Decoded frame view.
 * planes/linesize follow the same convention as libavcodec AVFrame:
 * planes[0] = Y, planes[1] = U, planes[2] = V for YUV420P.
 * The frame is owned by the decoder — see buffer ownership rules above. */
typedef struct
{
  uint8_t* planes[4];
  int linesize[4];
  int width;
  int height;
  int format; /* VideoFormat value, e.g. VIDEO_FORMAT_YUV420P */
} VideoFrame;

/* Function table exported by every plugin.
 * Fields marked "optional" may be NULL; the host must NULL-check before
 * calling.  All other function pointers must be non-NULL. */
typedef struct
{
  uint32_t abi_version; /* Must equal VIDEO_PLUGIN_ABI_VERSION */

  /* Lifecycle */
  VideoDecoder* (*create_decoder)(VideoCodec codec);
  void (*destroy_decoder)(VideoDecoder* dec);

  /* Push one unit of compressed data into the decoder.
   * Returns 0 on success, negative on error. */
  int (*send_packet)(VideoDecoder* dec, const uint8_t* data, int size);

  /* Pull a decoded frame.
   * Returns: 1 = frame ready in *out, 0 = need more input, negative = error.
   */
  int (*receive_frame)(VideoDecoder* dec, VideoFrame* out);

  /* Optional: flush buffered frames at end-of-stream. */
  int (*flush)(VideoDecoder* dec); /* optional */

  /* Optional: return a human-readable description of the last error. */
  const char* (*last_error)(VideoDecoder* dec); /* optional */

} VideoPluginAPI;

/* Every plugin shared library must export exactly this symbol.
 * host_abi: the VIDEO_PLUGIN_ABI_VERSION the host was compiled against.
 * out_api:  set to a pointer to the plugin's static VideoPluginAPI table.
 * Returns 0 on success, non-zero on failure. */
typedef int (*video_plugin_init_fn)(uint32_t host_abi,
                                    const VideoPluginAPI** out_api);

#endif /* IFM3D_RTSP_PLUGIN_H */
