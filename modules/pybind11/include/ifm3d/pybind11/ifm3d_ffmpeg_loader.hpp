/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND11_IFM3D_FFMPEG_LOADER_HPP
#define IFM3D_PYBIND11_IFM3D_FFMPEG_LOADER_HPP

#include <exception>
#include <string>

#include <ifm3d/common/logging/log.h>
#include <ifm3d/rtsp/decoder_manager.h>
#include <pybind11/pybind11.h>

namespace ifm3d
{
  inline void
  load_ifm3d_ffmpeg()
  {
    namespace py = pybind11;

    py::module_ package;
    try
      {
        package = py::module_::import("ifm3d_ffmpeg");
      }
    catch (const py::error_already_set& error)
      {
        if (error.matches(PyExc_ModuleNotFoundError) &&
            py::cast<std::string>(error.value().attr("name")) ==
              "ifm3d_ffmpeg")
          {
            LOG_INFO("ifm3d-ffmpeg not found");
            return;
          }

        LOG_WARNING("Failed to import ifm3d-ffmpeg: {}", error.what());
        return;
      }

    std::string build = "unknown";
    std::string ffmpeg_version = "unknown";
    std::string platform = "unknown";
    int avcodec_major = 0;
    int avutil_major = 0;
    try
      {
        const auto info = package.attr("abi_info")().cast<py::dict>();
        build = py::cast<std::string>(info["distribution_version"]);
        ffmpeg_version = py::cast<std::string>(info["ffmpeg_version"]);
        platform = py::cast<std::string>(info["platform"]);
        avcodec_major = py::cast<int>(info["avcodec_major"]);
        avutil_major = py::cast<int>(info["avutil_major"]);
        package.attr("activate")();
      }
    catch (const py::error_already_set& error)
      {
        LOG_WARNING(
          "Unable to load ifm3d-ffmpeg build {} (FFmpeg {}, {}, avcodec {}, "
          "avutil {}), "
          "valid: false ({})",
          build,
          ffmpeg_version,
          platform,
          avcodec_major,
          avutil_major,
          error.what());
        return;
      }
    catch (const std::exception& error)
      {
        LOG_WARNING(
          "Unable to load ifm3d-ffmpeg build {} (FFmpeg {}, {}, avcodec {}, "
          "avutil {}), "
          "valid: false ({})",
          build,
          ffmpeg_version,
          platform,
          avcodec_major,
          avutil_major,
          error.what());
        return;
      }

    bool valid = false;
    std::string validation_error;
    for (const auto& decoder : ifm3d::DecoderManager::DiscoverDecoders())
      {
        if (decoder.name == "ffmpeg")
          {
            valid = decoder.available;
            validation_error = decoder.error;
            break;
          }
      }

    if (valid)
      {
        LOG_INFO("Loaded ifm3d-ffmpeg build {} (FFmpeg {}, {}, avcodec {}, "
                 "avutil {}), "
                 "valid: true",
                 build,
                 ffmpeg_version,
                 platform,
                 avcodec_major,
                 avutil_major);
      }
    else
      {
        LOG_INFO("Loaded ifm3d-ffmpeg build {} (FFmpeg {}, {}, avcodec {}, "
                 "avutil {}), "
                 "valid: false ({})",
                 build,
                 ffmpeg_version,
                 platform,
                 avcodec_major,
                 avutil_major,
                 validation_error);
      }
  }
} // namespace ifm3d

#endif // IFM3D_PYBIND11_IFM3D_FFMPEG_LOADER_HPP
