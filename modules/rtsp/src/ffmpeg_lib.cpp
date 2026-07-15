/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ffmpeg_lib.hpp"

#include <array>
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
          api.av_strerror = bind<decltype(api.av_strerror)>(avcodec,
                                                            avutil,
                                                            "av_strerror",
                                                            ok);

          if (ok)
            {
              api.available = true;
              api.error.clear();
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
