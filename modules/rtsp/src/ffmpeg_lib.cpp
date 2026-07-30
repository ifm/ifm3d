/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ffmpeg_lib.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#ifdef _WIN32
#  include <windows.h>
#else
#  include <dlfcn.h>
#endif

namespace ifm3d::rtsp
{
  namespace
  {
    /* Candidate {libavcodec, libavutil} SONAME pairs, newest first. FFmpeg
     * keeps these major versions in lock-step per release:
     *   FFmpeg 4.x -> avcodec 58 / avutil 56
     *   FFmpeg 5.x -> avcodec 59 / avutil 57
     *   FFmpeg 6.x -> avcodec 60 / avutil 58
     *   FFmpeg 7.x -> avcodec 61 / avutil 59
     *   FFmpeg 8.x -> avcodec 62 / avutil 60
     * plus recent development snapshots. */
    struct SonamePair
    {
      const char* avcodec;
      const char* avutil;
    };

#ifdef _WIN32
    constexpr std::array<SonamePair, 6> LIBRARY_CANDIDATES{{
      {"avcodec-63.dll", "avutil-61.dll"},
      {"avcodec-62.dll", "avutil-60.dll"},
      {"avcodec-61.dll", "avutil-59.dll"},
      {"avcodec-60.dll", "avutil-58.dll"},
      {"avcodec-59.dll", "avutil-57.dll"},
      {"avcodec-58.dll", "avutil-56.dll"},
    }};
#else
    constexpr std::array<SonamePair, 7> LIBRARY_CANDIDATES{{
      {"libavcodec.so.63", "libavutil.so.61"},
      {"libavcodec.so.62", "libavutil.so.60"},
      {"libavcodec.so.61", "libavutil.so.59"},
      {"libavcodec.so.60", "libavutil.so.58"},
      {"libavcodec.so.59", "libavutil.so.57"},
      {"libavcodec.so.58", "libavutil.so.56"},
      /* Dev symlinks, as a last resort (present when -dev packages installed)
       */
      {"libavcodec.so", "libavutil.so"},
    }};
#endif

    using LibHandle = void*;

    LibHandle
    open_library(const char* name)
    {
#ifdef _WIN32
      return static_cast<LibHandle>(LoadLibraryA(name));
#else
      return dlopen(name, RTLD_NOW | RTLD_LOCAL);
#endif
    }

    void*
    resolve(LibHandle handle, const char* name)
    {
#ifdef _WIN32
      return reinterpret_cast<void*>(
        GetProcAddress(static_cast<HMODULE>(handle), name));
#else
      return dlsym(handle, name);
#endif
    }

    /* Resolve @p name from either handle; sets @p ok to false on failure. */
    template <typename FN>
    FN
    bind(LibHandle avcodec,
         LibHandle avutil,
         const char* name,
         bool& ok) // NOLINT(google-runtime-references)
    {
      void* sym = resolve(avcodec, name);
      if (sym == nullptr)
        {
          sym = resolve(avutil, name);
        }
      if (sym == nullptr)
        {
          ok = false;
        }
      return reinterpret_cast<FN>(sym);
    }

    /**
     * Confirm that the loaded libavutil lays AVFrame out the way
     * ffmpeg_lib.hpp describes, and measure where AVFrame::pts actually sits.
     *
     * A freshly allocated frame carries known sentinels from FFmpeg's
     * get_frame_defaults(): the struct is zeroed, `extended_data` is aimed at
     * the frame's own `data` array, `format` is -1 and `pts` is
     * AV_NOPTS_VALUE. The first two identify the prefix beyond reasonable
     * doubt; the third then tells us which of the two candidate pts offsets is
     * real, i.e. whether this libavutil still carries the deprecated key_frame
     * int ahead of pts.
     *
     * Returns false (leaving api.error set) if anything fails to line up, so
     * the caller can report the decoder as unavailable rather than risk
     * reading a bogus pts.
     */
    bool
    probe_frame_layout(FfmpegApi& api) // NOLINT(google-runtime-references)
    {
      ff::AVFrame* frame = api.av_frame_alloc();
      if (frame == nullptr)
        {
          api.error = "av_frame_alloc failed while probing the AVFrame layout";
          return false;
        }

      const ff::FramePrefix* prefix = FfmpegApi::frame(frame);
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      const auto* base = reinterpret_cast<const unsigned char*>(frame);

      const auto read_i64 = [base](std::size_t offset) {
        std::int64_t value = 0;
        std::memcpy(&value, base + offset, sizeof(value));
        return value;
      };

      bool ok = false;
      if (prefix->extended_data != prefix->data)
        {
          api.error = "unexpected AVFrame layout: extended_data does not "
                      "alias data";
        }
      else if (prefix->format != -1)
        {
          api.error = "unexpected AVFrame layout: format is not -1 on a "
                      "freshly allocated frame";
        }
      else
        {
          /* Which layout applies is decided by the libavutil major, not by
           * probing for the sentinel. Probing cannot disambiguate: on an
           * ILP32 ABI that aligns int64_t to 8 the two candidates are 8
           * bytes apart, which is exactly where pkt_dts sits, and
           * av_frame_alloc() sets that to AV_NOPTS_VALUE too. The offset
           * chosen here is still validated against the sentinel below. */
          const unsigned major = api.avutil_version() >> 16U;
          api.frame_pts_offset =
            major >= ff::KEY_FRAME_REMOVED_IN_AVUTIL_MAJOR ?
              ff::FRAME_PTS_OFFSET_WITHOUT_KEY_FRAME :
              ff::FRAME_PTS_OFFSET_WITH_KEY_FRAME;

          ok = read_i64(api.frame_pts_offset) == ff::NOPTS_VALUE &&
               /* pkt_dts sits one int64 above pts and carries the same
                * sentinel, so the check above would also pass if the real
                * pts were 8 bytes lower and this read landed on pkt_dts.
                * The word below pts is sample_aspect_ratio (plus padding),
                * which get_frame_defaults() sets to {0, 1} and never to the
                * sentinel, so reading it rules that shift out. */
               read_i64(api.frame_pts_offset - sizeof(std::int64_t)) !=
                 ff::NOPTS_VALUE;
          if (!ok)
            {
              api.error = "unexpected AVFrame layout: pts is not at the "
                          "offset libavutil " +
                          std::to_string(major) + " implies";
            }
        }

      api.av_frame_free(&frame);
      return ok;
    }

    /**
     * Confirm that the loaded libavcodec lays the front of AVPacket out the
     * way ffmpeg_lib.hpp describes. av_packet_alloc() zeroes the struct and
     * then sets pts and dts to AV_NOPTS_VALUE, so data/size stay null while
     * both timestamps read as sentinels. Unlike AVFrame there is no variant to
     * pick between here, only a layout to validate.
     */
    bool
    probe_packet_layout(FfmpegApi& api) // NOLINT(google-runtime-references)
    {
      ff::AVPacket* packet = api.av_packet_alloc();
      if (packet == nullptr)
        {
          api.error = "av_packet_alloc failed while probing the AVPacket "
                      "layout";
          return false;
        }

      const ff::PacketPrefix* prefix = FfmpegApi::packet(packet);
      const bool ok = prefix->pts == ff::NOPTS_VALUE &&
                      prefix->dts == ff::NOPTS_VALUE &&
                      prefix->data == nullptr && prefix->size == 0;
      if (!ok)
        {
          api.error = "unexpected AVPacket layout: pts/dts/data/size do not "
                      "match a freshly allocated packet";
        }

      api.av_packet_free(&packet);
      return ok;
    }

    FfmpegApi
    load()
    {
      FfmpegApi api;

      for (const auto& candidate : LIBRARY_CANDIDATES)
        {
          LibHandle avcodec = open_library(candidate.avcodec);
          if (avcodec == nullptr)
            {
              continue;
            }
          LibHandle avutil = open_library(candidate.avutil);
          if (avutil == nullptr)
            {
              /* avcodec is intentionally left loaded; it is never unloaded. */
              continue;
            }

          bool ok = true;
          api.avcodec_find_decoder =
            bind<decltype(api.avcodec_find_decoder)>(avcodec,
                                                     avutil,
                                                     "avcodec_find_decoder",
                                                     ok);
          api.avcodec_alloc_context3 =
            bind<decltype(api.avcodec_alloc_context3)>(
              avcodec,
              avutil,
              "avcodec_alloc_context3",
              ok);
          api.avcodec_open2 =
            bind<decltype(api.avcodec_open2)>(avcodec,
                                              avutil,
                                              "avcodec_open2",
                                              ok);
          api.avcodec_free_context =
            bind<decltype(api.avcodec_free_context)>(avcodec,
                                                     avutil,
                                                     "avcodec_free_context",
                                                     ok);
          api.avcodec_send_packet =
            bind<decltype(api.avcodec_send_packet)>(avcodec,
                                                    avutil,
                                                    "avcodec_send_packet",
                                                    ok);
          api.avcodec_receive_frame =
            bind<decltype(api.avcodec_receive_frame)>(avcodec,
                                                      avutil,
                                                      "avcodec_receive_frame",
                                                      ok);
          api.av_packet_alloc =
            bind<decltype(api.av_packet_alloc)>(avcodec,
                                                avutil,
                                                "av_packet_alloc",
                                                ok);
          api.av_packet_free =
            bind<decltype(api.av_packet_free)>(avcodec,
                                               avutil,
                                               "av_packet_free",
                                               ok);
          api.av_packet_unref =
            bind<decltype(api.av_packet_unref)>(avcodec,
                                                avutil,
                                                "av_packet_unref",
                                                ok);
          api.av_new_packet =
            bind<decltype(api.av_new_packet)>(avcodec,
                                              avutil,
                                              "av_new_packet",
                                              ok);
          api.av_frame_alloc =
            bind<decltype(api.av_frame_alloc)>(avcodec,
                                               avutil,
                                               "av_frame_alloc",
                                               ok);
          api.av_frame_free =
            bind<decltype(api.av_frame_free)>(avcodec,
                                              avutil,
                                              "av_frame_free",
                                              ok);
          api.avutil_version =
            bind<decltype(api.avutil_version)>(avcodec,
                                               avutil,
                                               "avutil_version",
                                               ok);
          api.av_strerror = bind<decltype(api.av_strerror)>(avcodec,
                                                            avutil,
                                                            "av_strerror",
                                                            ok);
          api.av_dict_set = bind<decltype(api.av_dict_set)>(avcodec,
                                                            avutil,
                                                            "av_dict_set",
                                                            ok);
          api.av_dict_free = bind<decltype(api.av_dict_free)>(avcodec,
                                                              avutil,
                                                              "av_dict_free",
                                                              ok);

          if (ok && probe_frame_layout(api) && probe_packet_layout(api))
            {
              api.available = true;
              api.error.clear();
              return api;
            }

          if (ok)
            {
              /* Symbols resolved but the ABI does not look like anything we
               * know how to read. Keep the probe's diagnostic and stop: a
               * different SONAME is unlikely to behave differently, and
               * guessing is exactly what this probe exists to prevent. */
              const std::string reason = api.error;
              api = FfmpegApi{};
              api.available = false;
              api.error = std::string("libavcodec '") + candidate.avcodec +
                          "' loaded but " + reason;
              return api;
            }

          /* Found the libraries but they are missing expected symbols; keep
           * looking in case another candidate is complete. */
          api = FfmpegApi{};
          api.error = std::string("libavcodec '") + candidate.avcodec +
                      "' is missing required symbols";
        }

      if (api.error.empty())
        {
          api.error = "libavcodec/libavutil not found";
        }
      api.available = false;
      return api;
    }

  } // namespace

  const FfmpegApi&
  ffmpeg_api()
  {
    static const FfmpegApi API = load();
    return API;
  }

} // namespace ifm3d::rtsp
