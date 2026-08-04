/*
 * Copyright (C) 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 *
 * ifm3d RTSP video decoder interface.
 *
 * Decoder implementations are compiled directly into the RTSP
 * module. Each implements the abstract VideoDecoder / VideoDecoderFactory C++
 * interfaces below; DecoderManager lists the available implementations and
 * selects one for a given codec. An implementation is free to load its own
 * heavy runtime dependencies lazily (e.g. the ffmpeg decoder resolves
 * libavcodec/libavutil at runtime) and to report, through IsAvailable(),
 * whether it can actually be used on the current system.
 *
 * Thread-safety contract
 * ----------------------
 * All calls (SendPacket, ReceiveFrame, Flush, ...) for a given VideoDecoder
 * instance MUST be made from a single thread at a time. Decoders are not
 * required to be thread-safe across instances.
 *
 * Buffer ownership
 * ----------------
 * - Buffers passed to SendPacket are owned by the caller and are only valid
 *   for the duration of that call; decoders must copy if they retain.
 * - VideoFrame plane pointers returned by ReceiveFrame are owned by the
 *   decoder and remain valid until the next call to ReceiveFrame, Flush, or
 *   the decoder's destruction.
 */

#ifndef IFM3D_RTSP_VIDEO_DECODER_H
#define IFM3D_RTSP_VIDEO_DECODER_H

#include <cstdint>
#include <memory>
#include <string>

namespace ifm3d::rtsp
{
  /** Supported video codecs. */
  enum VideoCodec
  {
    VIDEO_CODEC_H264 = 1
  };

  /** Pixel formats for decoded frames. */
  enum VideoFormat
  {
    VIDEO_FORMAT_YUV420P = 1
  };

  /**
   * Decoded frame view.
   *
   * planes/linesize follow the same convention as libavcodec's AVFrame:
   * planes[0] = Y, planes[1] = U, planes[2] = V for YUV420P. The frame is
   * owned by the decoder -- see the buffer ownership rules above.
   */
  struct VideoFrame
  {
    std::uint8_t* planes[4]{};
    int linesize[4]{};
    int width{0};
    int height{0};
    int format{0};        /* VideoFormat value, e.g. VIDEO_FORMAT_YUV420P */
    std::uint64_t pts{0}; /* echoed from the SendPacket call that built this
                             frame -- see the frame identity contract above */
  };

  /**
   * Abstract H.264 (etc.) video decoder.
   *
   * Instances are created through VideoDecoderFactory::CreateDecoder and owned
   * by the caller (via std::unique_ptr).
   */
  class VideoDecoder
  {
  public:
    virtual ~VideoDecoder() = default;

    /**
     * Push one unit of compressed data into the decoder.
     *
     * @param data  compressed payload, valid for the duration of the call.
     * @param size  size of @p data in bytes.
     * @param pts   an opaque, caller-chosen identifier for this packet. The
     *              decoder does not interpret it; it only stores it and
     *              returns it on the VideoFrame decoded from this packet.
     *              Callers should keep it unique across the packets that may
     *              be in flight at once. Implementations that have no natural
     *              place to put it must carry it themselves.
     *
     * @return 0 on success, negative on error.
     */
    virtual int SendPacket(const std::uint8_t* data,
                           int size,
                           std::uint64_t pts) = 0;

    /**
     * Pull a decoded frame. Call repeatedly until it reports that no further
     * frame is available.
     *
     * `out.pts` identifies the SendPacket call this frame was decoded from.
     * It is not necessarily the most recent one: an H.264 stream whose SPS
     * advertises `max_num_reorder_frames > 0` makes libavcodec hold frames
     * back, so output routinely lags input by a fixed number of packets.
     *
     * @return 1 = frame ready in @p out, 0 = no further frame can be built
     * from the data submitted so far, negative = error.
     */
    virtual int ReceiveFrame(VideoFrame& out) = 0;

    /** Optional: flush buffered frames at end-of-stream. */
    virtual int
    Flush()
    {
      return 0;
    }

    /** Optional: a human-readable description of the last error. */
    [[nodiscard]] virtual std::string
    LastError() const
    {
      return {};
    }

  protected:
    VideoDecoder() = default;
    VideoDecoder(const VideoDecoder&) = default;
    VideoDecoder& operator=(const VideoDecoder&) = default;
    VideoDecoder(VideoDecoder&&) = default;
    VideoDecoder& operator=(VideoDecoder&&) = default;
  };

  /**
   * Abstract decoder implementation (factory).
   *
   * A VideoDecoderFactory is a factory that advertises which codecs it can
   * decode, whether it is usable on the current system, and creates decoder
   * instances.
   */
  class VideoDecoderFactory
  {
  public:
    virtual ~VideoDecoderFactory() = default;

    /** Stable, human-readable name (e.g. "ffmpeg", "null"). */
    [[nodiscard]] virtual std::string Name() const = 0;

    /**
     * Whether this implementation can actually be used on the current system.
     * Implementations that depend on optional runtime libraries (e.g. the
     * ffmpeg decoder needs libavcodec) return false when those are missing.
     */
    [[nodiscard]] virtual bool IsAvailable() const = 0;

    /** Whether this implementation can decode @p codec. */
    [[nodiscard]] virtual bool SupportsCodec(VideoCodec codec) const = 0;

    /**
     * Create a decoder for @p codec, or nullptr if unsupported/unavailable.
     */
    virtual std::unique_ptr<VideoDecoder> CreateDecoder(VideoCodec codec) = 0;

    /**
     * Optional human-readable reason why IsAvailable() is false; empty when
     * the decoder is available.
     */
    [[nodiscard]] virtual std::string
    AvailabilityError() const
    {
      return {};
    }

  protected:
    VideoDecoderFactory() = default;
    VideoDecoderFactory(const VideoDecoderFactory&) = default;
    VideoDecoderFactory& operator=(const VideoDecoderFactory&) = default;
    VideoDecoderFactory(VideoDecoderFactory&&) = default;
    VideoDecoderFactory& operator=(VideoDecoderFactory&&) = default;
  };

} // namespace ifm3d::rtsp

#endif /* IFM3D_RTSP_VIDEO_DECODER_H */
