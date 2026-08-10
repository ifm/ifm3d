/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_FRAMEGRABBER
#define IFM3D_PYBIND_BINDING_FRAMEGRABBER

#include <ifm3d/fg/frame_grabber.h>
#include <ifm3d/pybind11/bindings/future.h>
#include <ifm3d/pybind11/util.hpp>
#include <optional>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <utility>

namespace py = pybind11;

inline void
bind_framegrabber(pybind11::module_& m)
{

  py::class_<ifm3d::FrameGrabber, ifm3d::FrameGrabber::Ptr> framegrabber(
    m,
    "FrameGrabber",
    R"(
      Implements a TCP FrameGrabber connected to a provided Camera
    )");

  framegrabber.def(
    // A factory rather than py::init<> so the holder carries the custom
    // deleter; see ifm3d::with_cleanup.
    py::init(
      [](ifm3d::Device::Ptr cam, std::optional<std::uint16_t> pcic_port) {
        return ifm3d::with_cleanup(
          new ifm3d::FrameGrabber(std::move(cam), pcic_port),
          [](ifm3d::FrameGrabber* fg) { fg->Stop().wait(); });
      }),
    py::arg("cam"),
    py::arg("pcic_port") = std::nullopt,
    R"(
      Constructor

      Parameters
      ----------
      cam : ifm3dpy.device.Device
          The camera instance to grab frames from.

      pcic_port : int
          The PCIC port
    )");

  {
    py::options options;
    options.disable_function_signatures();

    framegrabber.def(
      "start",
      [](const ifm3d::FrameGrabber::Ptr& self,
         const ifm3d::FrameGrabber::BufferList& buffers,
         const std::optional<py::dict>& pcic_format) {
        py::object json_dumps = py::module::import("json").attr("dumps");
        return FutureAwaitable<void>(
          pcic_format.has_value() ?
            self->Start(
              buffers,
              ifm3d::json::parse(
                json_dumps(pcic_format.value()).cast<std::string>())) :
            self->Start(buffers));
      },
      py::arg("buffers") = ifm3d::FrameGrabber::BufferList{},
      py::arg("pcic_format") = std::nullopt,
      py::doc(R"(
        start(self: ifm3dpy.framegrabber.FrameGrabber, buffers: typing.Sequence[Union[int, ifm3dpy.framegrabber.buffer_id]] = [], pcic_format: Optional[dict] = None) -> ifm3dpy.Awaitable


        Starts the worker thread for streaming in pixel data from the device

        Parameters
        ----------
        buffers : list[ifm3dpy.framegrabber.buffer_id]
            A list of buffer_ids for receiving. Passing in an empty list will
            receive all available images. The buffer_ids are specific to
            the current Organizer. See buffer_id for a list of buffer_ids available
            with the default Organizer

        pcic_format : dict, optional
            allows to manually set a PCIC pcic_format for
            asynchronous results. See ifm3d::make_schema for generation logic of the
            default pcic_format. Manually setting the pcic_format should rarely be needed and
            most usecases should be covered by the default generated pcic_format.

            Note: The FrameGrabber is relying on some specific formatting rules, if
            they are missing from the pcic_format the FrameGrabber will not be able to
            extract the image data.

        Returns
        -------
        ifm3dpy.Awaitable
            Resolves when framegrabber is ready to receive frames.
      )"));
  }

  framegrabber.def(
    "stop",
    [](const ifm3d::FrameGrabber::Ptr& fg) {
      return FutureAwaitable<void>(fg->Stop());
    },
    R"(
      Stops the worker thread for streaming in pixel data from the device

      Returns
      -------
      ifm3dpy.Awaitable
          Resolves when framgrabber stops.
    )");

  framegrabber.def("is_running",
                   &ifm3d::FrameGrabber::IsRunning,
                   R"(
      Returns true if the worker thread is currently running
    )");

  framegrabber.def(
    "wait_for_frame",
    [](const ifm3d::FrameGrabber::Ptr& fg) {
      return FutureAwaitable<ifm3d::Frame::Ptr>(fg->WaitForFrame());
    },
    R"(
      Returns an Awaitable that will resolve when a new frame is available
    )");

  framegrabber.def(
    "on_new_frame",
    [](const ifm3d::FrameGrabber::Ptr& fg,
       const ifm3d::FrameGrabber::NewFrameCallback& callback) {
      if (callback)
        {
          fg->OnNewFrame([callback](const ifm3d::Frame::Ptr& frame) {
            py::gil_scoped_acquire acquire;
            try
              {
                callback(frame);
              }
            catch (py::error_already_set& ex)
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
    )");

  framegrabber.def(
    "on_async_error",
    [](const ifm3d::FrameGrabber::Ptr& fg,
       const ifm3d::FrameGrabber::AsyncErrorCallback& callback) {
      if (callback)
        {
          fg->OnAsyncError([callback](int code, const std::string& message) {
            py::gil_scoped_acquire acquire;
            try
              {
                callback(code, message);
              }
            catch (py::error_already_set& ex)
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
    )");

  framegrabber.def(
    "on_async_notification",
    [](const ifm3d::FrameGrabber::Ptr& fg,
       const ifm3d::FrameGrabber::AsyncNotificationCallback& callback) {
      if (callback)
        {
          fg->OnAsyncNotification([callback](const std::string& message_id,
                                             const std::string& payload) {
            py::gil_scoped_acquire acquire;
            try
              {
                callback(message_id, payload);
              }
            catch (py::error_already_set& ex)
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
    )");

  {
    py::options options;
    options.disable_function_signatures();

    framegrabber.def(
      "on_error",
      [](const ifm3d::FrameGrabber::Ptr& fg,
         const std::function<void(const py::object&)>& callback) {
        if (callback)
          {
            fg->OnError([callback](const ifm3d::Error& ifm3d_error) {
              py::gil_scoped_acquire acquire;
              try
                {
                  auto error_class =
                    py::module::import("ifm3dpy").attr("Error");
                  auto py_error = error_class(ifm3d_error.code(),
                                              ifm3d_error.message(),
                                              ifm3d_error.what());
                  callback(py_error);
                }
              catch (py::error_already_set& ex)
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
      py::arg("callback") = std::function<void(const py::object&)>(),
      R"(
        on_error(self: ifm3dpy.framegrabber.FrameGrabber, callback: Callable[[ifm3dpy.device.Error], None] = None) -> None


        The callback will be executed whenever an error condition
        occur while grabbing the data from device.
      )");
  }

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
    )");

  framegrabber.def("set_masking",
                   &ifm3d::FrameGrabber::SetMasking,
                   py::arg("mask"),
                   R"(
      Enable/Disable masking on supported buffers
      Note: ifm3dpy.buffer_id.CONFIDENCE_IMAGE should be in schema  list passed to ifm3dpy.FrameGrabber.Start method

      Parameters
      ----------
      mask : bool
          flag to enable/disable masking.
    )");

  framegrabber.def("is_masking",
                   &ifm3d::FrameGrabber::IsMasking,
                   R"(
      Returns
      -------
      bool
          True if masking is currently enabled
    )");

  framegrabber.def(
    "send_command",
    [](const ifm3d::FrameGrabber::Ptr& fg, const ifm3d::PCICCommand& cmd) {
      return FutureAwaitable<ifm3d::FrameGrabber::PCICCommandResponse>(
        fg->SendCommand(cmd));
    },
    py::arg("command"),
    R"(
      Send a PCIC command asynchronously to the frame grabber.

      Parameters
      ----------
      command : ifm3dpy.device.PCICCommand
          The command to send to the frame grabber.

      Returns
      -------
      ifm3dpy.PCICCommandResponseAwaitable
          An awaitable object returns: None, a string, or a bytes object if the response is binary.
    )");
  // clang-format on
}

#endif // IFM3D_PYBIND_BINDING_FRAMEGRABBER