/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_DESERIALIZE_RGB_INFO_V2_H
#define IFM3D_PYBIND_BINDING_DESERIALIZE_RGB_INFO_V2_H

#include <ifm3d/deserialize/struct_rgb_info_v2.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

inline void
bind_struct_rgbinfov2(pybind11::module_& m)
{
  py::class_<ifm3d::RGBInfoV2, ifm3d::RGBInfoV2::Ptr, ifm3d::RGBInfoV1>
    rgb_info_v2(m,
                "RGBInfoV2",
                R"(
        Class for managing an instance of an struct RGBInfoV2
      )");

  rgb_info_v2.def(py::init<>(),
                  R"(
        Constructor
      )");

  rgb_info_v2.def_readonly("reserved",
                           &ifm3d::RGBInfoV2::reserved,
                           R"(
        Reserved for future use. This is the same 4-byte range that
        exposure_time reads as a float on RGBInfoV1, but RGB_INFO v2
        formally reserves it instead of carrying a meaningful exposure
        time; the inherited exposure_time value should therefore not be
        relied upon here.
      )");

  rgb_info_v2.def_readonly("rolling_shutter_row_offset_ns",
                           &ifm3d::RGBInfoV2::rolling_shutter_row_offset_ns,
                           R"(
        Rolling-shutter offset between consecutive row timestamps, given
        in [nanoseconds]. The exposure-center timestamp of row y is
        t_row(y) = timestamp_ns + (2 * y - (height - 1)) *
        rolling_shutter_row_offset_ns / 2
      )");

  rgb_info_v2.def_static(
    "deserialize",
    [](const py::array_t<uint8_t, py::array::c_style | py::array::forcecast>&
         in) -> ifm3d::RGBInfoV2 {
      ifm3d::RGBInfoV2 val{};
      val.Read(reinterpret_cast<const uint8_t*>(in.data(0)), in.nbytes());
      return val;
    },
    py::arg("buffer"),
    R"(
        Deserialize RGB_INFO Buffer to RGBInfoV2 struct

        Parameters
        ----------
        buffer : numpy.ndarray
            The raw buffer data as an array of ``numpy.uint8``.

        Returns
        -------
        RGBInfoV2
            The deserialized structure.
      )");
}
#endif // IFM3D_PYBIND_BINDING_DESERIALIZE_RGB_INFO_V2_H
