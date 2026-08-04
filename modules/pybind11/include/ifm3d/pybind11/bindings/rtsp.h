/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_RTSP
#define IFM3D_PYBIND_BINDING_RTSP

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <ifm3d/rtsp/decoder_manager.h>
#include <ifm3d/rtsp/nal_unit.h>
#include <ifm3d/rtsp/rtsp_client.h>

#include <ifm3d/pybind11/bindings/future.h>
#include <ifm3d/pybind11/util.hpp>

#include <pybind11/functional.h>
#include <pybind11/native_enum.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

inline void
bind_rtsp(pybind11::module_& m)
{
  // -------------------------------------------------------------------------
  // NalUnit
  // -------------------------------------------------------------------------
  py::class_<ifm3d::NalUnit> nal_unit(m,
                                      "NalUnit",
                                      R"(
      A single, fully reassembled H.264 NAL unit.
    )");

  py::native_enum<ifm3d::NalUnit::Type>(
    nal_unit,
    "Type",
    "enum.IntEnum",
    "H.264 NAL unit type values (ITU-T H.264 Table 7-1).")
    .value("NON_IDR_SLICE", ifm3d::NalUnit::NON_IDR_SLICE)
    .value("IDR_SLICE", ifm3d::NalUnit::IDR_SLICE)
    .value("SEI", ifm3d::NalUnit::SEI)
    .value("SPS", ifm3d::NalUnit::SPS)
    .value("PPS", ifm3d::NalUnit::PPS)
    .value("ACCESS_UNIT_DELIMITER", ifm3d::NalUnit::ACCESS_UNIT_DELIMITER)
    .finalize();

  nal_unit.def_property_readonly(
    "data",
    [](const ifm3d::NalUnit& self) {
      return py::bytes(reinterpret_cast<const char*>(self.data.data()),
                       self.data.size());
    },
    R"(
      Raw NAL payload (NAL-header byte first, without an Annex-B start code).
    )");
  nal_unit.def_readonly("nal_ref_idc",
                        &ifm3d::NalUnit::nal_ref_idc,
                        "nal_ref_idc field from the NAL header (0-3).");
  nal_unit.def_readonly("nal_unit_type",
                        &ifm3d::NalUnit::nal_unit_type,
                        "nal_unit_type field from the NAL header (1-31).");
  nal_unit.def_readonly("pts_us",
                        &ifm3d::NalUnit::pts_us,
                        "Presentation timestamp, in microseconds.");
  nal_unit.def_readonly("is_idr",
                        &ifm3d::NalUnit::is_idr,
                        "True when this NAL unit is an IDR slice.");
  nal_unit.def_readonly("first_sequence_number",
                        &ifm3d::NalUnit::first_sequence_number,
                        "RTP sequence number of the first source packet.");
  nal_unit.def_readonly("last_sequence_number",
                        &ifm3d::NalUnit::last_sequence_number,
                        "RTP sequence number of the last source packet.");
  nal_unit.def("__len__",
               [](const ifm3d::NalUnit& self) { return self.Size(); });

  // -------------------------------------------------------------------------
  // RtspClient
  // -------------------------------------------------------------------------
  py::class_<ifm3d::RtspClient, ifm3d::RtspClient::Ptr> rtsp(m,
                                                             "RtspClient",
                                                             R"(
      RTSP/1.0 client that streams H.264 video from an ifm device and,
      optionally, decodes it into buffers using a compiled-in video decoder.
    )");

  py::native_enum<ifm3d::RtspClient::Transport>(
    rtsp,
    "Transport",
    "enum.IntEnum",
    "RTP transport used for the media stream.")
    .value("INTERLEAVED", ifm3d::RtspClient::Transport::INTERLEAVED)
    .value("UDP", ifm3d::RtspClient::Transport::UDP)
    .finalize();

  py::native_enum<ifm3d::RtspClient::State>(
    rtsp,
    "State",
    "enum.IntEnum",
    "Lifecycle state of the RTSP session.")
    .value("IDLE", ifm3d::RtspClient::State::IDLE)
    .value("CONNECTING", ifm3d::RtspClient::State::CONNECTING)
    .value("READY", ifm3d::RtspClient::State::READY)
    .value("PLAYING", ifm3d::RtspClient::State::PLAYING)
    .value("STOPPED", ifm3d::RtspClient::State::STOPPED)
    .value("FAILED", ifm3d::RtspClient::State::FAILED)
    .finalize();

  rtsp.def(py::init([](ifm3d::Device::Ptr device,
                       std::optional<std::string> url,
                       std::uint16_t port,
                       std::string stream_path,
                       ifm3d::RtspClient::Transport transport,
                       std::optional<std::string> decoder) {
             ifm3d::RtspClient::Config config;
             config.url = std::move(url);
             config.port = port;
             config.stream_path = std::move(stream_path);
             config.transport = transport;
             config.decoder = std::move(decoder);
             return ifm3d::with_cleanup(
               new ifm3d::RtspClient(std::move(device), std::move(config)),
               [](ifm3d::RtspClient* client) { client->Stop().wait(); });
           }),
           py::arg("device"),
           py::arg("url") = py::none(),
           py::arg("port") = std::uint16_t{8554},
           py::arg("stream_path") = std::string("port1"),
           py::arg("transport") = ifm3d::RtspClient::Transport::INTERLEAVED,
           py::arg("decoder") = py::none(),
           R"(
      Constructs a client that streams from the given device.

      Parameters
      ----------
      device : ifm3dpy.device.Device
          Device used to discover the stream IP (unless ``url`` is set).
      url : str, optional
          Full RTSP URL override (e.g. ``rtsp://192.168.0.69:8554/port1``).
          When unset the URL is built from the device IP, ``port`` and
          ``stream_path``.
      port : int, optional
          RTSP server port, used when ``url`` is unset.
      stream_path : str, optional
          Stream path appended to the device IP, used when ``url`` is unset.
      transport : ifm3dpy.rtsp.RtspClient.Transport, optional
          RTP transport selection.
      decoder : str, optional
          Override the decoder name used for decoding h264 video into buffers.
          When unset the decoder is chosen automatically.
    )");

  rtsp.def(
    py::init([](ifm3d::Device::Ptr device,
                const ifm3d::PortInfo& port,
                std::optional<std::string> url,
                ifm3d::RtspClient::Transport transport,
                std::optional<std::string> decoder) {
      ifm3d::RtspClient::Config config;
      config.url = std::move(url);
      config.transport = transport;
      config.decoder = std::move(decoder);
      return ifm3d::with_cleanup(
        new ifm3d::RtspClient(std::move(device), port, std::move(config)),
        [](ifm3d::RtspClient* client) { client->Stop().wait(); });
    }),
    py::arg("device"),
    py::arg("port"),
    py::arg("url") = py::none(),
    py::arg("transport") = ifm3d::RtspClient::Transport::INTERLEAVED,
    py::arg("decoder") = py::none(),
    R"(
      Constructs a client for a specific port, deriving the RTSP port and
      stream path from the port's advertised RtspInfo when available.

      Parameters
      ----------
      device : ifm3dpy.device.Device
          Device used to discover the stream IP (unless ``url`` is set).
      port : ifm3dpy.device.PortInfo
          Port whose RTSP endpoint should be streamed.
      url : str, optional
          Full RTSP URL override. When unset the URL is built from the device
          IP and the port's RtspInfo.
      transport : ifm3dpy.rtsp.RtspClient.Transport, optional
          RTP transport selection.
      decoder : str, optional
          Override the decoder name used for decoding h264 video into buffers.
          When unset the decoder is chosen automatically.
    )");

  rtsp.def(
    "start",
    [](const ifm3d::RtspClient::Ptr& self,
       const ifm3d::RtspClient::BufferIdList& buffers) {
      return FutureAwaitable<void>(self->Start(buffers));
    },
    py::arg("buffers") = ifm3d::RtspClient::BufferIdList{},
    // Start() may report a configuration error through the Python on_error
    // callback, which takes the GIL from whichever thread is inside Start().
    py::call_guard<py::gil_scoped_release>(),
    R"(
      Performs the RTSP handshake and starts streaming.

      Parameters
      ----------
      buffers : list[ifm3dpy.framegrabber.buffer_id], optional
          Buffers to make available on each received frame. Supported ids are
          ``COMPRESSED_H264_FRAME``, ``RGB_IMAGE``, ``YUV420_IMAGE`` and
          ``RGB_INFO``. An empty list, the default, requests all four; each
          one costs additional per-frame work, so pass only what you consume.

      Returns
      -------
      Awaitable
          Resolves once the stream is playing, or raises on failure.
    )");

  rtsp.def(
    "stop",
    [](const ifm3d::RtspClient::Ptr& self) {
      return FutureAwaitable<void>(self->Stop());
    },
    // Stop() joins the receive thread, which may be waiting for the GIL in
    // order to deliver a frame; holding on to it here would deadlock both.
    py::call_guard<py::gil_scoped_release>(),
    R"(
      Sends TEARDOWN, stops the worker thread and closes the socket.

      Returns
      -------
      Awaitable
          Resolves once the client has stopped.
    )");

  rtsp.def("is_running",
           &ifm3d::RtspClient::IsRunning,
           R"(
      Returns true while the worker thread is running.
    )");

  rtsp.def("get_state",
           &ifm3d::RtspClient::GetState,
           R"(
      Returns the current session state.
    )");

  rtsp.def(
    "on_new_frame",
    [](const ifm3d::RtspClient::Ptr& self,
       const ifm3d::RtspClient::NewFrameCallback& callback) {
      if (callback)
        {
          self->OnNewFrame([callback](const ifm3d::Frame::Ptr& frame) {
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
          self->OnNewFrame();
        }
    },
    py::arg("callback") = ifm3d::RtspClient::NewFrameCallback(),
    R"(
      Registers a callback invoked for each received frame.
    )");

  rtsp.def(
    "on_nal_unit",
    [](const ifm3d::RtspClient::Ptr& self,
       const ifm3d::RtspClient::NalUnitCallback& callback) {
      if (callback)
        {
          self->OnNalUnit([callback](const ifm3d::NalUnit& nal) {
            py::gil_scoped_acquire acquire;
            try
              {
                callback(nal);
              }
            catch (py::error_already_set& ex)
              {
                py::print(ex.value());
              }
          });
        }
      else
        {
          self->OnNalUnit();
        }
    },
    py::arg("callback") = ifm3d::RtspClient::NalUnitCallback(),
    R"(
      Registers a callback invoked for each reassembled NAL unit.
    )");

  {
    py::options options;
    options.disable_function_signatures();

    rtsp.def(
      "on_error",
      [](const ifm3d::RtspClient::Ptr& self,
         const std::function<void(const py::object&)>& callback) {
        if (callback)
          {
            self->OnError([callback](const ifm3d::Error& ifm3d_error) {
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
            self->OnError();
          }
      },
      py::arg("callback") = std::function<void(const py::object&)>(),
      R"(
        on_error(self: ifm3dpy.rtsp.RtspClient, callback: Callable[[ifm3dpy.device.Error], None] = None) -> None


        Registers a callback invoked on terminal errors.
      )");
  }

  rtsp.def(
    "on_state_change",
    [](const ifm3d::RtspClient::Ptr& self,
       const ifm3d::RtspClient::StateChangeCallback& callback) {
      if (callback)
        {
          self->OnStateChange([callback](ifm3d::RtspClient::State state) {
            py::gil_scoped_acquire acquire;
            try
              {
                callback(state);
              }
            catch (py::error_already_set& ex)
              {
                py::print(ex.value());
              }
          });
        }
      else
        {
          self->OnStateChange();
        }
    },
    py::arg("callback") = ifm3d::RtspClient::StateChangeCallback(),
    R"(
      Registers a callback invoked on every session state transition.
    )");

  // -------------------------------------------------------------------------
  // DecoderManager (decoder discovery / diagnostics)
  // -------------------------------------------------------------------------
  py::class_<ifm3d::DecoderManager> decoder_manager(m,
                                                    "DecoderManager",
                                                    R"(
      Lists the available video decoders and reports whether each is usable.
    )");

  py::class_<ifm3d::DecoderManager::DecoderInfo>(decoder_manager,
                                                 "DecoderInfo",
                                                 R"(
      Describes a decoder and whether it is usable on this system.
    )")
    .def_readonly("name",
                  &ifm3d::DecoderManager::DecoderInfo::name,
                  "Stable decoder name (e.g. 'ffmpeg', 'null').")
    .def_readonly(
      "available",
      &ifm3d::DecoderManager::DecoderInfo::available,
      "True when the decoder can be used on the current system (e.g. the "
      "ffmpeg decoder found a usable libavcodec).")
    .def_readonly("supports_h264",
                  &ifm3d::DecoderManager::DecoderInfo::supports_h264,
                  "True when the decoder advertises H.264 decode support.")
    .def_readonly("error",
                  &ifm3d::DecoderManager::DecoderInfo::error,
                  "Reason the decoder is not usable; empty when it is usable.")
    .def("__repr__", [](const ifm3d::DecoderManager::DecoderInfo& self) {
      return "<DecoderInfo name='" + self.name +
             "' available=" + (self.available ? "True" : "False") +
             " supports_h264=" + (self.supports_h264 ? "True" : "False") + ">";
    });

  decoder_manager.def_static("discover_decoders",
                             &ifm3d::DecoderManager::DiscoverDecoders,
                             R"(
      List the video decoders and report whether each is usable.

      Returns
      -------
      List[ifm3dpy.rtsp.DecoderManager.DecoderInfo]
          One entry per decoder.
    )");
}

#endif // IFM3D_PYBIND_BINDING_RTSP
