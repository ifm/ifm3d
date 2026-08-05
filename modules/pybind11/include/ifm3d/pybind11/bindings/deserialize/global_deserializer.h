/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef IFM3D_PYBIND_BINDING_GLOBAL_DESERIALIZER
#define IFM3D_PYBIND_BINDING_GLOBAL_DESERIALIZER

#include <ifm3d/deserialize/deserialize.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d/pybind11/util.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/typing.h>

namespace py = pybind11;

inline void
bind_deserializer(py::module_& m)
{
  using DeserializedStruct = ifm3d::Annotated<py::typing::Optional<
    py::typing::Union<ifm3d::ODSInfoV1,
                      ifm3d::TOFInfoV3,
                      ifm3d::TOFInfoV4,
                      ifm3d::RGBInfoV1,
                      ifm3d::ODSOccupancyGridV1,
                      ifm3d::ODSPolarOccupancyGridV1,
                      ifm3d::ODSExtrinsicCalibrationCorrectionV1,
                      ifm3d::IMUInfoV1>>>;

  m.def(
    "deserialize",
    [](const ifm3d::PyBuffer& buffer) -> DeserializedStruct {
      auto in_array = py::cast<py::array>(buffer);

      auto buffer_id = buffer.attr("buffer_id").cast<ifm3d::buffer_id>();

      ifm3d::VariantType result =
        ifm3d::deserialize(reinterpret_cast<const uint8_t*>(in_array.data(0)),
                           in_array.nbytes(),
                           buffer_id);

      return std::visit(
        [](auto&& val) -> py::object {
          using T = std::decay_t<decltype(val)>;
          if constexpr (std::is_same_v<T, std::monostate>)
            {
              return py::none();
            }
          else
            {
              return py::cast(val);
            }
        },
        result);
    },
    py::arg("buffer"),
    R"(
        Deserialize the given buffer and return the appropriate data structure.

        The structure returned is selected based on the ``buffer_id`` of the
        provided buffer.

        Parameters
        ----------
        buffer : ifm3dpy.Buffer
            A buffer as returned by
            :meth:`ifm3dpy.framegrabber.Frame.get_buffer`.

        Returns
        -------
        ODSInfoV1 or TOFInfoV3 or TOFInfoV4 or RGBInfoV1 or ODSOccupancyGridV1 or ODSPolarOccupancyGridV1 or ODSExtrinsicCalibrationCorrectionV1 or IMUInfoV1 or None
            The deserialized structure, or ``None`` if the buffer could not be
            interpreted.
    )");
}

#endif // IFM3D_PYBIND_BINDING_GLOBAL_DESERIALIZER
