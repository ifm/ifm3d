/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_FRAMEGRABBER
#define IFM3D_PYBIND_BINDING_FRAMEGRABBER

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <ifm3d/fg/frame_grabber.h>
#include "bindings/future.h"

void
bind_framegrabber(pybind11::module_& m)
{
  // clang-format off

  bind_future<ifm3d::Frame::Ptr>(m, "_FutureAwaitable__FramePtr");

  py::class_<ifm3d::FrameGrabber, ifm3d::FrameGrabber::Ptr> framegrabber(
    m,
    "FrameGrabber",
    R"(
      Implements a TCP FrameGrabber connected to a provided Camera
    )"
  );

  framegrabber.def(
    py::init<ifm3d::Device::Ptr, std::optional<std::uint16_t>>(),
    py::arg("cam"),
    py::arg("pcic_port") = std::nullopt,
    R"(
      Constructor

      Parameters
      ----------
      cam : ifm3dpy.Camera
          The camera instance to grab frames from.

      pcic_port : uint16
          The PCIC port
    )"
  );

  framegrabber.def(
    "start",
    &ifm3d::FrameGrabber::Start,
    py::arg("buffers") = ifm3d::FrameGrabber::BufferList{},
    py::arg("schema") = std::nullopt,
    R"(
      Starts the worker thread for streaming in pixel data from the device

      Parameters
      ----------
      buffers : List[uint64]
          A List of buffer_ids for receiving, passing in an List
          set will received all available images. The buffer_ids are specific to
          the current Organizer. See buffer_id for a list of buffer_ids available
          with the default Organizer
      
      schema : Dict
          allows to manually set a PCIC schema for
          asynchronous results. See ifm3d::make_schema for generation logic of the
          default schema. Manually setting the schema should rarely be needed and
          most usecases should be covered by the default generated schema.
      
          Note: The FrameGrabber is relying on some specific formatting rules, if
          they are missing from the schema the FrameGrabber will not be able to
          extract the image data.
    )"
  );

  framegrabber.def(
    "stop",
    &ifm3d::FrameGrabber::Stop,
    R"(
      Stops the worker thread for streaming in pixel data from the device
    )"
  );

  framegrabber.def(
    "is_running",
    &ifm3d::FrameGrabber::IsRunning,
    R"(
      Returns true if the worker thread is currently running
    )"
  );

  framegrabber.def(
    "wait_for_frame",
    [](const ifm3d::FrameGrabber::Ptr& fg) {
      return FutureAwaitable<ifm3d::Frame::Ptr>(fg->WaitForFrame());
    },
    R"(
      Returns an Awaitable that will resolve when a new frame is available
    )"
  );

  framegrabber.def(
    "on_new_frame",
    [](const ifm3d::FrameGrabber::Ptr& fg, const ifm3d::FrameGrabber::NewFrameCallback& callback) {
      if(callback) 
        {
          fg->OnNewFrame([callback](const ifm3d::Frame::Ptr& frame){
            py::gil_scoped_acquire acquire;
            callback(frame);
          });
        }
      else 
        {
          fg->OnNewFrame();
        }
    },
    py::arg("callback") = ifm3d::FrameGrabber::NewFrameCallback(),
    R"(
      The callback will be executed whenever a new frame is available.
      It receives the frame as an argument.
    )"
  );

  // clang-format on
}

#endif // IFM3D_PYBIND_BINDING_FRAMEGRABBER