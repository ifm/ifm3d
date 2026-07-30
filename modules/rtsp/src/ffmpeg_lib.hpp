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
 * libavcodec/libavutil, nor does it include their headers. The required
 * functions are resolved at runtime via dlopen/dlsym (LoadLibrary/
 * GetProcAddress on Windows), trying a range of library SONAMEs. This lets a
 * single ifm3d binary run against whichever FFmpeg major version happens to be
 * installed (e.g. Ubuntu 22.04 ships libavcodec.so.58, Ubuntu 24.04 ships
 * libavcodec.so.60), and to gracefully report "unavailable" (falling back to
 * the null decoder) when no compatible FFmpeg is present. It also means FFmpeg
 * is not a build-time dependency at all.
 *
 * Because the headers are not included, the small part of the FFmpeg ABI we
 * touch is declared here instead:
 *
 * - All AV* objects are opaque. AVCodecContext and AVCodec are never
 *   dereferenced, only allocated by the library and passed back to it, so
 *   their layout is irrelevant. Decoder options go through av_dict_set rather
 *   than AVCodecContext fields for exactly this reason.
 * - Only the front of AVFrame and AVPacket is read, via FramePrefix and
 *   PacketPrefix below. Those fields have not moved across libavcodec 58..63 /
 *   libavutil 56..61.
 * - AVFrame::pts sits behind the deprecated key_frame int, the one field ahead
 *   of it that FFmpeg has ever removed (in libavutil 60 / FFmpeg 8). Rather
 *   than assume which layout the loaded library uses, its offset is measured
 *   at load time; see probe_frame_layout() in ffmpeg_lib.cpp. If it cannot be
 *   established the loader reports the API as unavailable and the null decoder
 *   takes over, so a layout change can never cause a silent mispairing.
 *
 * test/ifm3d-ffmpeg-abi.cpp exercises the probe. Cross-checking these
 * declarations against the real FFmpeg headers needs those headers, so it
 * lives in the ffmpeg build repository rather than here.
 */

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

namespace ifm3d::rtsp
{
  /**
   * Mirror declarations of the parts of the libavcodec/libavutil ABI used by
   * the ffmpeg decoder. Deliberately kept minimal: every addition here is a
   * new compatibility assumption.
   */
  namespace ff
  {
    /* Opaque handles. Allocated and freed by the loaded library; never
     * dereferenced by us, so no layout is assumed. */
    struct AVCodec;
    struct AVCodecContext;
    struct AVDictionary;
    struct AVFrame;
    struct AVPacket;

    extern "C" {
    /* AVCodecID is spelled `int` here rather than mirroring a 300-entry
     * enum; it is passed in a register either way. Its return is const even
     * though FFmpeg 4.x declares it non-const, which is likewise ABI
     * neutral. */
    using FnAvcodecFindDecoder = const AVCodec* (*)(int id);
    using FnAvcodecAllocContext3 = AVCodecContext* (*)(const AVCodec*);
    using FnAvcodecOpen2 = int (*)(AVCodecContext*,
                                   const AVCodec*,
                                   AVDictionary**);
    using FnAvcodecFreeContext = void (*)(AVCodecContext**);
    using FnAvcodecSendPacket = int (*)(AVCodecContext*, const AVPacket*);
    using FnAvcodecReceiveFrame = int (*)(AVCodecContext*, AVFrame*);
    using FnAvPacketAlloc = AVPacket* (*)();
    using FnAvPacketFree = void (*)(AVPacket**);
    using FnAvPacketUnref = void (*)(AVPacket*);
    using FnAvNewPacket = int (*)(AVPacket*, int size);
    using FnAvFrameAlloc = AVFrame* (*)();
    using FnAvFrameFree = void (*)(AVFrame**);
    using FnAvutilVersion = unsigned (*)();
    using FnAvStrerror = int (*)(int errnum, char* errbuf, std::size_t size);
    using FnAvDictSet = int (*)(AVDictionary**,
                                const char* key,
                                const char* value,
                                int flags);
    using FnAvDictFree = void (*)(AVDictionary**);
    }

    /** AV_NUM_DATA_POINTERS; 8 since before libavutil 56. */
    inline constexpr int NUM_DATA_POINTERS = 8;

    /**
     * The front of AVFrame, up to but excluding the deprecated key_frame int.
     * Unchanged across libavutil 56..61.
     */
    struct FramePrefix
    {
      std::uint8_t* data[NUM_DATA_POINTERS];
      int linesize[NUM_DATA_POINTERS];
      std::uint8_t** extended_data;
      int width;
      int height;
      int nb_samples;
      int format;
    };

    /**
     * The front of AVPacket. Unchanged across libavcodec 58..63.
     * `buf` is an AVBufferRef* we never dereference.
     */
    struct PacketPrefix
    {
      void* buf;
      std::int64_t pts;
      std::int64_t dts;
      std::uint8_t* data;
      int size;
    };

    inline constexpr std::size_t
    align_up(std::size_t value, std::size_t alignment)
    {
      return ((value + alignment - 1) / alignment) * alignment;
    }

    /* AVFrame::pts follows format, then the deprecated key_frame int (absent
     * since libavutil 60), then pict_type and the two ints of
     * sample_aspect_ratio. Which of the two applies is decided by
     * avutil_version(), not by probing: the two candidates are 4 bytes apart
     * on i386 but 8 apart on any ILP32 ABI that aligns int64_t to 8 (AArch32
     * AAPCS, MIPS o32, x32), and 8 apart is exactly the distance to pkt_dts,
     * which av_frame_alloc() also sets to AV_NOPTS_VALUE. A sentinel probe
     * cannot tell those two apart. On LP64 both candidates coincide.
     *
     * `alignof(std::int64_t)` is the alignment the ABI gives an int64_t
     * member inside a struct, which is what decides this offset. Note it is
     * not always GCC's `__alignof__(long long)`: on i386 SysV the two differ
     * (4 vs 8), and it is the 4 that is correct here. Measured on i386
     * Debian with both GCC and Clang, and on x86_64 against libavutil 56-60.
     * The load-time check rejects the layout outright if this or the version
     * rule is ever wrong -- it reads the word below pts as well as pts
     * itself, so a shift in either direction is caught -- and a mistake here
     * therefore disables the decoder rather than misreading pts.
     *
     * The int counts below are those fields:
     *   with key_frame:    key_frame + pict_type + sar.num + sar.den = 4
     *   without key_frame: pict_type + sar.num + sar.den             = 3 */
    inline constexpr std::size_t FRAME_PTS_OFFSET_WITH_KEY_FRAME =
      align_up(sizeof(FramePrefix) + (4 * sizeof(int)), alignof(std::int64_t));
    inline constexpr std::size_t FRAME_PTS_OFFSET_WITHOUT_KEY_FRAME =
      align_up(sizeof(FramePrefix) + (3 * sizeof(int)), alignof(std::int64_t));

    /** First libavutil major that dropped the deprecated AVFrame::key_frame.
     */
    inline constexpr unsigned KEY_FRAME_REMOVED_IN_AVUTIL_MAJOR = 60;

    /* Values below are fixed points of the FFmpeg ABI: changing any of them
     * would break every application compiled against an older header. */

    /** AV_CODEC_ID_H264. */
    inline constexpr int CODEC_ID_H264 = 27;
    /** AV_PIX_FMT_YUV420P. */
    inline constexpr int PIX_FMT_YUV420P = 0;
    /** AV_PIX_FMT_YUVJ420P (deprecated upstream, still emitted by h264). */
    inline constexpr int PIX_FMT_YUVJ420P = 12;
    /** AV_NOPTS_VALUE. */
    inline constexpr std::int64_t NOPTS_VALUE = INT64_MIN;
    /** AV_ERROR_MAX_STRING_SIZE. */
    inline constexpr std::size_t ERROR_MAX_STRING_SIZE = 64;

    /** AVERROR(e); mirrors the EDOM sign convention from libavutil/error.h. */
    inline constexpr int
    averror(int posix_error)
    {
      return EDOM > 0 ? -posix_error : posix_error;
    }

    inline constexpr int
    mktag(unsigned char first,
          unsigned char second,
          unsigned char third,
          unsigned char fourth)
    {
      return static_cast<int>(static_cast<unsigned>(first) |
                              (static_cast<unsigned>(second) << 8U) |
                              (static_cast<unsigned>(third) << 16U) |
                              (static_cast<unsigned>(fourth) << 24U));
    }

    /** AVERROR_EOF, i.e. FFERRTAG('E','O','F',' '). */
    inline constexpr int ERROR_EOF = -mktag('E', 'O', 'F', ' ');

  } // namespace ff

  /**
   * Resolved libavcodec/libavutil entry points, plus the frame layout measured
   * at load time.
   */
  struct FfmpegApi
  {
    /* libavcodec */
    ff::FnAvcodecFindDecoder avcodec_find_decoder = nullptr;
    ff::FnAvcodecAllocContext3 avcodec_alloc_context3 = nullptr;
    ff::FnAvcodecOpen2 avcodec_open2 = nullptr;
    ff::FnAvcodecFreeContext avcodec_free_context = nullptr;
    ff::FnAvcodecSendPacket avcodec_send_packet = nullptr;
    ff::FnAvcodecReceiveFrame avcodec_receive_frame = nullptr;
    ff::FnAvPacketAlloc av_packet_alloc = nullptr;
    ff::FnAvPacketFree av_packet_free = nullptr;
    ff::FnAvPacketUnref av_packet_unref = nullptr;
    ff::FnAvNewPacket av_new_packet = nullptr;

    /* libavutil */
    ff::FnAvFrameAlloc av_frame_alloc = nullptr;
    ff::FnAvFrameFree av_frame_free = nullptr;
    ff::FnAvutilVersion avutil_version = nullptr;
    ff::FnAvStrerror av_strerror = nullptr;
    ff::FnAvDictSet av_dict_set = nullptr;
    ff::FnAvDictFree av_dict_free = nullptr;

    /**
     * Offset of AVFrame::pts in the loaded libavutil, established by
     * probe_frame_layout(). Only meaningful while `available` is true.
     */
    std::size_t frame_pts_offset = 0;

    /* True when every required function was resolved and the frame layout was
     * confirmed. */
    bool available = false;
    /* Human-readable reason when available is false; empty otherwise. */
    std::string error;

    /** The mirrored front of a frame owned by the loaded library. */
    [[nodiscard]] static ff::FramePrefix*
    frame(ff::AVFrame* frame)
    {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      return reinterpret_cast<ff::FramePrefix*>(frame);
    }

    /** The mirrored front of a packet owned by the loaded library. */
    [[nodiscard]] static ff::PacketPrefix*
    packet(ff::AVPacket* packet)
    {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      return reinterpret_cast<ff::PacketPrefix*>(packet);
    }

    /** Read AVFrame::pts from the offset measured at load time. */
    [[nodiscard]] std::int64_t
    frame_pts(const ff::AVFrame* frame) const
    {
      std::int64_t pts = 0;
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      const auto* base = reinterpret_cast<const unsigned char*>(frame);
      std::memcpy(&pts, base + frame_pts_offset, sizeof(pts));
      return pts;
    }
  };

  /**
   * Lazily load libavcodec/libavutil, resolve the required entry points and
   * confirm the AVFrame layout. The result is cached; the returned reference
   * is valid for the program's lifetime and the libraries are never unloaded.
   */
  const FfmpegApi& ffmpeg_api();

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_FFMPEG_LIB_HPP
