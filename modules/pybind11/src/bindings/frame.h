/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_FRAME
#define IFM3D_PYBIND_BINDING_FRAME

#include <pybind11/pybind11.h>

void
bind_frame(pybind11::module_& m)
{
  // clang-format off
  py::class_<ifm3d::Frame, ifm3d::Frame::Ptr> frame(
    m,
    "Frame",
    R"(
    )"
  );

  frame.def(
    "timestamps",
    &ifm3d::Frame::TimeStamps,
    R"(
    )");

  frame.def(
    "has_image",
    &ifm3d::Frame::HasImage,
    py::arg("id"),
    R"(
    )");

  frame.def(
    "get_image",
    [](const ifm3d::Frame::Ptr& frame, ifm3d::ImageId id){
        return ifm3d::image_to_array(frame->GetImage(id));
    },
    py::arg("id"),
    R"(
    )");

  // clang-format on
}

#endif // IFM3D_PYBIND_BINDING_FRAME