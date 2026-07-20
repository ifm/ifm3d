/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/pybind11/bindings/ifm3d.h>
#include <ifm3d/pybind11/util.hpp> // NOLINT(misc-include-cleaner), included so clangd can infer the correct compilation settings
#if defined(IFM3D_RTSP_WITH_FFMPEG)
#  include <ifm3d/pybind11/ifm3d_ffmpeg_loader.hpp>
#endif
#include <pybind11/pybind11.h> // NOLINT(misc-include-cleaner), included for PYBIND11_MODULE

PYBIND11_MODULE(ifm3dpy, m) // NOLINT(misc-include-cleaner)
{
#if defined(IFM3D_RTSP_WITH_FFMPEG)
  ifm3d::load_ifm3d_ffmpeg();
#endif

  // Bind the ifm3d module
  bind_ifm3d(m);
}
