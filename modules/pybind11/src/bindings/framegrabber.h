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

  bind_future<ifm3d::Frame::Ptr>(m, "FrameAwaitable", "Provides a mechanism to access the frame object");
  bind_future<void>(m, "Awaitable", "Provides a mechanism to wait for completion of a task");

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
    [](const ifm3d::FrameGrabber::Ptr& self, const ifm3d::FrameGrabber::BufferList& buffers, const std::optional<py::dict>& pcicFormat) {
      py::object json_dumps = py::module::import("json").attr("dumps");
      pcicFormat.has_value() 
        ? self->Start(buffers, json::parse(json_dumps(pcicFormat.value()).cast<std::string>())) 
        : self->Start(buffers);
    },
    py::arg("buffers") = ifm3d::FrameGrabber::BufferList{},
    py::arg("pcic_format") = std::nullopt,
    R"(
      Starts the worker thread for streaming in pixel data from the device

      Parameters
      ----------
      buffers : List[uint64]
          A List of buffer_ids for receiving, passing in an List
          set will received all available images. The buffer_ids are specific to
          the current Organizer. See buffer_id for a list of buffer_ids available
          with the default Organizer
      
      pcicFormat : Dict
          allows to manually set a PCIC pcicFormat for
          asynchronous results. See ifm3d::make_schema for generation logic of the
          default pcicFormat. Manually setting the pcicFormat should rarely be needed and
          most usecases should be covered by the default generated pcicFormat.
      
          Note: The FrameGrabber is relying on some specific formatting rules, if
          they are missing from the pcicFormat the FrameGrabber will not be able to
          extract the image data.
    )"
  );

  framegrabber.def(
    "stop",
    [](const ifm3d::FrameGrabber::Ptr& fg) {
      return FutureAwaitable<void>(fg->Stop());
      },
    R"(
      Stops the worker thread for streaming in pixel data from the device

      Returns
      -------
      FutureAwaitable

          Resolves when framgrabber stops.
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
            try 
              {
                callback(frame);
              }
            catch(py::error_already_set ex)
              {
                py::print(ex.value());
              }
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

  framegrabber.def(
    "on_async_error",
    [](const ifm3d::FrameGrabber::Ptr& fg, const ifm3d::FrameGrabber::AsyncErrorCallback& callback) {
      if(callback) 
        {
          fg->OnAsyncError([callback](int code, const std::string& message){
            py::gil_scoped_acquire acquire;
            try 
              {
                callback(code, message);
              }
            catch(py::error_already_set ex)
              {
                py::print(ex.value());
              }
          });
        }
      else 
        {
          fg->OnAsyncError();
        }
    },
    py::arg("callback") = ifm3d::FrameGrabber::AsyncErrorCallback(),
    R"(
      This function will enable the async error messages on device.
      The callback will be executed whenever a async error
      are avaliable. It receives a error code and error string
      to the received async error as an argument. 
    )"
  );

  framegrabber.def(
    "on_async_notification",
    [](const ifm3d::FrameGrabber::Ptr& fg, const ifm3d::FrameGrabber::AsyncNotificationCallback& callback) {
      if(callback) 
        {
          fg->OnAsyncNotification([callback](const std::string& message_id, const std::string& payload){
            py::gil_scoped_acquire acquire;
            try 
              {
                callback(message_id, payload);
              }
            catch(py::error_already_set ex)
              {
                py::print(ex.value());
              }
          });
        }
      else 
        {
          fg->OnAsyncNotification();
        }
    },
    py::arg("callback") = ifm3d::FrameGrabber::AsyncNotificationCallback(),
    R"(
      This function will enable the async notifications on device.
      The callback will be executed whenever a async notification
      is avaliable. It receives a message id and payload string
    )"
  );

  framegrabber.def(
    "on_error",
    [](const ifm3d::FrameGrabber::Ptr& fg, const std::function<void(const py::object&)>& callback) {
      if(callback) 
        {
            fg->OnError([callback](const ifm3d::Error& error){
            py::gil_scoped_acquire acquire;
            try
              {
                auto error_class = py::module::import("ifm3dpy").attr("Error");
                auto error_ = error_class(error.code(), error.message(),error.what());
                callback(error_);
              }
            catch(py::error_already_set ex)
              {
                py::print(ex.value());
              }
          });
        }
      else 
        {
          fg->OnError();
        }
    },
    py::arg("callback") =  std::function<void(const py::object&)>(),
    R"(
      The callback will be executed whenever an error condition
      occur while grabbing the data from device.
    )"
  );

  framegrabber.def(
    "sw_trigger",
    [](const ifm3d::FrameGrabber::Ptr& fg) {
      return FutureAwaitable<void>(fg->SWTrigger());
    },
    R"(
      Triggers the device for image acquisition

      You should be sure to set the `TriggerMode` for your application to
      `SW` in order for this to be effective. This function
      simply does the triggering, data are still received asynchronously via
      `wait_for_frame()`.

      Calling this function when the device is not in `SW` trigger mode or on
      a device that does not support software-trigger should result in a NOOP
      and no error will be returned (no exceptions thrown). However, we do not
      recommend calling this function in a tight framegrabbing loop when you
      know it is not needed. The "cost" of the NOOP is undefined and incurring
      it is not recommended.
    )"
  );

  // clang-format on
}

#endif // IFM3D_PYBIND_BINDING_FRAMEGRABBER