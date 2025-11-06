/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_FG_FRAMEGRABBER_IMPL_H
#define IFM3D_FG_FRAMEGRABBER_IMPL_H
#pragma once

#include "o3d_organizer.hpp"
#include "o3r_organizer.hpp"
#include "o3r_organizer3D.hpp"
#include "o3x_organizer.hpp"
#include <asio.hpp>
#include <asio/use_future.hpp>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <fmt/core.h> // NOLINT(*)
#include <fmt/core.h>
#include <functional>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device.h>
#include <ifm3d/fg/frame_grabber.h>
#include <ifm3d/fg/module_frame_grabber.h>
#include <ifm3d/fg/schema.h>
#include <memory>
#include <mutex>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ifm3d
{
  // <Ticket><Length>CR+LF (16 bytes)
  constexpr std::size_t TICKET_SIZE = 16;
  constexpr std::size_t TICKET_ID_START = 0;
  constexpr std::size_t TICKET_ID_END = 4;
  constexpr std::size_t PAYLOAD_SIZE_START = 5;
  constexpr std::size_t PAYLOAD_SIZE_END = 14;
  static const std::string TICKET_IMAGE = "0000";
  static const std::string TICKET_ASYNC_ERROR = "0001";
  static const std::string TICKET_ASYNC_NOTIFICATION = "0010";
  static const std::string TICKET_ALGO_DGB = "0020";
  static const std::string TICKET_COMMAND_c = "1000";
  static const std::string TICKET_COMMAND_t = "1001";
  static const std::string TICKET_COMMAND_p = "1002";

  inline std::atomic<uint16_t> _next_ticket_id{1100};

  //============================================================
  // Impl interface
  //============================================================
  class IFM3D_NO_EXPORT FrameGrabber::Impl
  {
  public:
    Impl(ifm3d::Device::Ptr cam, std::optional<std::uint16_t> nat_pcic_port);
    ~Impl();

    Impl(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl& operator=(Impl&&) = delete;

    std::shared_future<void> SWTrigger();

    void OnNewFrame(NewFrameCallback callback);

    std::shared_future<void> Start(const std::set<ifm3d::buffer_id>& images,
                                   const std::optional<json>& schema);
    void SetSchema(const json& schema);
    std::shared_future<void> Stop();
    bool IsRunning();

    std::shared_future<Frame::Ptr> WaitForFrame();

    void SetOrganizer(std::unique_ptr<Organizer> organizer);
    void OnAsyncError(AsyncErrorCallback callback);
    void OnAsyncNotification(AsyncNotificationCallback callback);
    void OnError(ErrorCallback callback);

    void SetMasking(bool masking);
    bool IsMasking();
    std::shared_future<ifm3d::FrameGrabber::PCICCommandResponse> SendCommand(
      const PCICCommand& command);

  protected:
    void run(const std::optional<json>& schema);

    void send_command(const std::string& ticket_id,
                      const std::string& content);
    void send_command(const std::string& ticket_id,
                      const std::vector<std::uint8_t>& content);
    json generate_default_schema();
    std::set<ifm3d::buffer_id> get_image_chunks(ifm3d::buffer_id id);
    //
    // ASIO event handlers
    //
    void connect_handler(const std::optional<json>& schema);

    void ticket_handler(const asio::error_code& ec,
                        std::size_t bytes_xferd,
                        std::size_t bytes_read);

    void payload_handler(const asio::error_code& ec,
                         std::size_t bytes_xferd,
                         std::size_t bytes_read,
                         const std::string& ticket_id);

    void image_handler();
    void async_error_handler();
    void async_notification_handler();
    void trigger_handler();
    void report_error(const ifm3d::Error& error);
    std::string calculate_async_command();
    std::string generate_ticket_id();

    //---------------------
    // State
    //---------------------
    ifm3d::Device::Ptr _cam;

    std::string _cam_ip;
    uint16_t _pcic_port;
    std::unique_ptr<asio::io_service> _io_service;
    std::mutex _io_service_mutex;
    std::unique_ptr<asio::ip::tcp::socket> _sock;
    std::shared_future<void> _finish_future;
    std::unique_ptr<Organizer> _organizer;
    std::set<buffer_id> _requested_images;
    bool _is_ready{};
    bool _masking{};
    std::mutex _mutex_for_masking;

    //
    // Holds the raw 'Ticket' bytes received from the sensor:
    //
    // PCIC V3 (16 bytes total)
    // 4 bytes ticket number
    // 'L' + 9 bytes length + CR + LF
    //
    // The '9 bytes length' is a string of chars to be converted to an integer
    // number representing the length of the payload data.
    //
    std::vector<std::uint8_t> _ticket_buffer;

    //
    // According to the PCIC V3 protocol, this will hold from the 4 digit
    // ticket sequence up to and including the ending CR + LF ('\r\n')
    //
    std::vector<std::uint8_t> _payload_buffer;

    FrameGrabber::NewFrameCallback _new_frame_callback;
    FrameGrabber::AsyncErrorCallback _async_error_callback;
    FrameGrabber::AsyncNotificationCallback _async_notification_callback;
    FrameGrabber::ErrorCallback _error_callback;

    std::promise<Frame::Ptr> _wait_for_frame_promise;
    std::shared_future<Frame::Ptr> _wait_for_frame_future;
    std::mutex _wait_for_frame_mutex;

    std::promise<void> _trigger_feedback_promise;
    std::shared_future<void> _trigger_feedback_future;

    std::promise<void> _ready_promise;
    std::shared_future<void> _ready_future;

    std::mutex _send_command_mutex;
    std::unordered_map<std::string,
                       std::shared_ptr<std::promise<PCICCommandResponse>>>
      _send_command_promises;
  }; // end: class FrameGrabber::Impl

} // end: namespace ifm3d

//============================================================
// Impl -- Implementation Details
//============================================================

//-------------------------------------
// ctor/dtor
//-------------------------------------
inline ifm3d::FrameGrabber::Impl::Impl(ifm3d::Device::Ptr cam,
                                       std::optional<std::uint16_t> pcic_port)
  : _cam(std::move(cam)),
    _cam_ip(this->_cam->IP()),
    _pcic_port(pcic_port.value_or(ifm3d::DEFAULT_PCIC_PORT)),
    _finish_future(std::async(std::launch::async, []() {})),
    _wait_for_frame_future(_wait_for_frame_promise.get_future()),
    _trigger_feedback_future(_trigger_feedback_promise.get_future()),
    _ready_future(_ready_promise.get_future())
{
  auto device_type = this->_cam->WhoAmI();
  if (device_type == Device::DeviceFamily::O3D)
    {
      this->SetMasking(true);
      this->SetOrganizer(std::make_unique<O3DOrganizer>());
      if (!pcic_port.has_value())
        {
          try
            {
              this->_pcic_port =
                std::stoi(this->_cam->DeviceParameter("PcicTcpPort"));
            }
          catch (const ifm3d::Error& ex)
            {
              LOG_ERROR("Could not get PCIC Port of the camera: {}",
                        ex.what());
              LOG_WARNING("Assuming default PCIC port: {}",
                          ifm3d::DEFAULT_PCIC_PORT);
              this->_pcic_port = ifm3d::DEFAULT_PCIC_PORT;
            }
        }
    }

  else if (device_type == Device::DeviceFamily::O3X)
    {
      this->SetMasking(true);
      this->SetOrganizer(std::make_unique<O3XOrganizer>());
    }
  else if (device_type == Device::DeviceFamily::O3R)
    {
      this->SetMasking(false);

      auto ports_info =
        std::dynamic_pointer_cast<ifm3d::O3R>(this->_cam)->Ports();

      for (auto& port_info : ports_info)
        {
          if (port_info.pcic_port == pcic_port)
            {
              if (port_info.type == "3D")
                {
                  this->SetOrganizer(std::make_unique<O3ROrganizer3D>());
                }
              else
                {
                  this->SetOrganizer(std::make_unique<O3ROrganizer>());
                }
              LOG_INFO("Device port compatibility verified, Organizer "
                       "configuration aligned.")
            }
        }
      if (_organizer == nullptr)
        {
          this->SetOrganizer(std::make_unique<O3ROrganizer>());
          LOG_WARNING(
            "The specified port is not compatible with the connected device. "
            "Falling back to the default Organizer");
        }
    }

  if (_organizer == nullptr)
    {
      LOG_ERROR(
        "The device's PCIC port does not match any compatible organizers");
      throw ifm3d::Error(IFM3D_DEVICE_PORT_INCOMPATIBLE_WITH_ORGANIZER);
    }

  LOG_INFO("Camera connection info: ip={}, port={}",
           this->_cam_ip,
           this->_pcic_port);
}

inline ifm3d::FrameGrabber::Impl::~Impl()
{
  LOG_VERBOSE("FrameGrabber dtor running...");

  Stop().wait();

  LOG_VERBOSE("FrameGrabber destroyed.");
}

//-------------------------------------
// "Public" interface
//-------------------------------------

inline std::shared_future<void>
ifm3d::FrameGrabber::Impl::SWTrigger()
{
  this->_io_service->post([this]() {
    auto reinitialize_promise = [this]() {
      this->_trigger_feedback_promise = std::promise<void>();
      this->_trigger_feedback_future = _trigger_feedback_promise.get_future();
    };
    try
      {
        if (this->_cam->AmI(ifm3d::Device::DeviceFamily::O3X))
          {
            this->_cam->ForceTrigger();
            this->_trigger_feedback_promise.set_value();
            reinitialize_promise();
          }
        else
          {
            send_command(TICKET_COMMAND_t, "t");
          }
      }
    catch (const ifm3d::Error& ex)
      {
        LOG_ERROR("While trying to software trigger the camera: {} = {}",
                  ex.code(),
                  ex.what());
        this->_trigger_feedback_promise.set_exception(
          std::current_exception());
        reinitialize_promise();
      }
    catch (std::exception& e)
      {
        this->_trigger_feedback_promise.set_exception(
          std::current_exception());
        reinitialize_promise();
      }
  });

  return this->_trigger_feedback_future;
}

inline void
ifm3d::FrameGrabber::Impl::OnNewFrame(NewFrameCallback callback)
{
  this->_new_frame_callback = std::move(callback);
}

inline std::shared_future<ifm3d::Frame::Ptr>
ifm3d::FrameGrabber::Impl::WaitForFrame()
{
  std::lock_guard<std::mutex> lock(this->_wait_for_frame_mutex);
  return this->_wait_for_frame_future;
}

inline std::shared_future<void>
ifm3d::FrameGrabber::Impl::Start(const std::set<ifm3d::buffer_id>& images,
                                 const std::optional<json>& schema)
{
  std::lock_guard<std::mutex> lock(this->_io_service_mutex);
  if (!this->_io_service)
    {
      this->_io_service = std::make_unique<asio::io_service>();
      this->_sock = std::make_unique<asio::ip::tcp::socket>(*_io_service);
      this->_requested_images = images;

      this->_finish_future = std::async(
        std::launch::async,
        [this](const std::optional<json>& schema) { this->run(schema); },
        schema);
      return this->_ready_future;
    }

  std::promise<void> promise;
  promise.set_value();
  return promise.get_future();
}

inline std::shared_future<void>
ifm3d::FrameGrabber::Impl::Stop()
{
  std::lock_guard<std::mutex> lock(this->_io_service_mutex);
  if (this->_io_service)
    {
      this->_io_service->post(
        []() { throw ifm3d::Error(IFM3D_THREAD_INTERRUPTED); });
    }

  return this->_finish_future;
}

inline bool
ifm3d::FrameGrabber::Impl::IsRunning()
{
  return this->_io_service != nullptr;
}

inline void
ifm3d::FrameGrabber::Impl::SetOrganizer(std::unique_ptr<Organizer> organizer)
{
  this->_organizer = std::move(organizer);
}

//-------------------------------------
// "Private" interface
//-------------------------------------

inline void
ifm3d::FrameGrabber::Impl::run(const std::optional<json>& schema)
{
  LOG_DEBUG("Framegrabber thread running...");

  this->_ticket_buffer.clear();
  this->_ticket_buffer.resize(ifm3d::TICKET_SIZE);

  std::optional<ifm3d::Error> error;

  try
    {
      this->_sock->connect(
        asio::ip::tcp::endpoint(asio::ip::address::from_string(this->_cam_ip),
                                this->_pcic_port));

      this->connect_handler(schema);

      this->_io_service->run();
    }
  catch (const ifm3d::Error& ex)
    {
      if (ex.code() != IFM3D_THREAD_INTERRUPTED)
        {
          LOG_WARNING(ex.what());
          this->report_error(ex);
          error = ex;
        }
    }
  catch (const asio::error_code& err)
    {
      error =
        ifm3d::Error(IFM3D_NETWORK_ERROR,
                     fmt::format("{0}: {1}", err.value(), err.message()));
    }
  catch (const std::system_error& err)
    {
      error =
        ifm3d::Error(IFM3D_SYSTEM_ERROR,
                     fmt::format("{0}: {1}", err.code().value(), err.what()));
    }
  catch (const std::exception& err)
    {
      error = ifm3d::Error(IFM3D_SYSTEM_ERROR, fmt::format("{1}", err.what()));
    }
  try
    {
      this->_sock->shutdown(asio::socket_base::shutdown_both);
      this->_sock->close();
    }
  catch (const std::system_error& err)
    {
      this->report_error(
        ifm3d::Error(IFM3D_SYSTEM_ERROR,
                     fmt::format("{0}: {1}", err.code().value(), err.what())));
      LOG_WARNING("System error {}: {}",
                  static_cast<int>(err.code().value()),
                  err.what());
    }

  {
    std::lock_guard<std::mutex> lock(this->_io_service_mutex);
    this->_sock.reset();
    this->_io_service.reset();
  }

  this->_ready_promise = std::promise<void>();
  this->_ready_future = this->_ready_promise.get_future();
  this->_is_ready = false;

  if (error.has_value())
    {
      LOG_WARNING("Exception: {}: {}",
                  static_cast<int>(error.value().code()),
                  error.value().what());
      auto ex_ptr = std::make_exception_ptr(error.value());
      this->_wait_for_frame_promise.set_exception(ex_ptr);
      if (!this->_is_ready)
        {
          this->_ready_promise.set_exception(ex_ptr);
        }
    }

  LOG_INFO("FrameGrabber thread done.");
}

inline void
ifm3d::FrameGrabber::Impl::send_command(const std::string& ticket_id,
                                        const std::string& content)
{
  send_command(ticket_id,
               std::vector<std::uint8_t>(content.begin(), content.end()));
}

inline void
ifm3d::FrameGrabber::Impl::send_command(
  const std::string& ticket_id,
  const std::vector<std::uint8_t>& content)
{
  std::string prefix =
    fmt::format("{0}L{1:09}\r\n{0}", ticket_id, 4 + content.size() + 2);
  std::string suffix = "\r\n";

  asio::write(*this->_sock,
              std::vector<asio::const_buffer>{
                asio::buffer(prefix),
                asio::buffer(content.data(), content.size()),
                asio::buffer(suffix)});
}

inline void
ifm3d::FrameGrabber::Impl::connect_handler(
  const std::optional<ifm3d::json>& schema)
{
  // Set the schema
  if (schema.has_value())
    {
      SetSchema(schema.value());
    }
  else if (!this->_requested_images.empty())
    {
      SetSchema(generate_default_schema());
    }

  send_command(TICKET_COMMAND_p, calculate_async_command());

  this->_sock->async_read_some(
    asio::buffer(this->_ticket_buffer.data(), ifm3d::TICKET_SIZE),
    [this](auto&& error_code, auto&& bytes_xferd) {
      ticket_handler(std::forward<decltype(error_code)>(error_code),
                     std::forward<decltype(bytes_xferd)>(bytes_xferd),
                     0);
    });

  if (this->_cam->AmI(ifm3d::Device::DeviceFamily::O3X))
    {
      if (!this->_is_ready)
        {
          this->_is_ready = true;
          this->_ready_promise.set_value();
        }
    }
}

inline void
ifm3d::FrameGrabber::Impl::ticket_handler(const asio::error_code& ec,
                                          std::size_t bytes_xferd,
                                          std::size_t bytes_read)
{
  if (ec && ec.value() != asio::error::try_again)
    {
      throw ifm3d::Error(IFM3D_NETWORK_ERROR,
                         fmt::format("{0}: {1}", ec.value(), ec.message()));
    }

  bytes_read += bytes_xferd;
  if (bytes_read < ifm3d::TICKET_SIZE)
    {
      this->_sock->async_read_some(
        asio::buffer(this->_ticket_buffer.data() + bytes_read,
                     this->_ticket_buffer.size() - bytes_read),
        [this, bytes_read](auto&& error_code, auto&& bytes_transferred) {
          ticket_handler(
            std::forward<decltype(error_code)>(error_code),
            std::forward<decltype(bytes_transferred)>(bytes_transferred),
            bytes_read);
        });
      return;
    }

  std::string ticket_id(this->_ticket_buffer.begin() + TICKET_ID_START,
                        this->_ticket_buffer.begin() + TICKET_ID_END);

  std::string payload_size_str(
    this->_ticket_buffer.begin() + PAYLOAD_SIZE_START,
    this->_ticket_buffer.begin() + PAYLOAD_SIZE_END);

  std::size_t payload_size = std::stoi(payload_size_str);
  _payload_buffer.resize(payload_size);

  this->_sock->async_read_some(
    asio::buffer(this->_payload_buffer.data(), payload_size),
    [this, ticket_id](auto&& error_code, auto&& bytes_xferd) {
      payload_handler(std::forward<decltype(error_code)>(error_code),
                      std::forward<decltype(bytes_xferd)>(bytes_xferd),
                      0,
                      ticket_id);
    });
}

inline void
ifm3d::FrameGrabber::Impl::payload_handler(const asio::error_code& ec,
                                           std::size_t bytes_xferd,
                                           std::size_t bytes_read,
                                           const std::string& ticket_id)
{
  if (ec && ec.value() != asio::error::try_again)
    {
      throw ifm3d::Error(IFM3D_NETWORK_ERROR,
                         fmt::format("{0}: {1}", ec.value(), ec.message()));
    }

  bytes_read += bytes_xferd;
  if (bytes_read < this->_payload_buffer.size())
    {
      this->_sock->async_read_some(
        asio::buffer(this->_payload_buffer.data() + bytes_read,
                     this->_payload_buffer.size() - bytes_read),
        [this, bytes_read, ticket_id](auto&& error_code, auto&& bytes_xferd) {
          payload_handler(std::forward<decltype(error_code)>(error_code),
                          std::forward<decltype(bytes_xferd)>(bytes_xferd),
                          bytes_read,
                          ticket_id);
        });
      return;
    }

  if (ticket_id == ifm3d::TICKET_IMAGE || ticket_id == ifm3d::TICKET_ALGO_DGB)
    {
      if (this->_is_ready)
        {
          this->image_handler();
        }
      else
        {
          LOG_DEBUG("Waiting for pcic command ACK!");
        }
    }
  else if (ticket_id == ifm3d::TICKET_ASYNC_ERROR)
    {
      this->async_error_handler();
    }
  else if (ticket_id == ifm3d::TICKET_ASYNC_NOTIFICATION)
    {
      this->async_notification_handler();
    }
  else if (ticket_id == ifm3d::TICKET_COMMAND_c)
    {
      if (this->_payload_buffer.at(4) != '*')
        {
          LOG_ERROR("Error setting pcic schema on device");
          throw ifm3d::Error(
            IFM3D_PCIC_BAD_REPLY,
            fmt::format("Error setting pcic schema on device"));
        }
    }
  else if (ticket_id == ifm3d::TICKET_COMMAND_p)
    {
      if (!this->_is_ready)
        {
          if (this->_payload_buffer.at(4) == '*')
            {
              this->_is_ready = true;
              this->_ready_promise.set_value();
            }
          else
            {
              LOG_ERROR("Error setting pcic mode on device");
              throw ifm3d::Error(
                IFM3D_PCIC_BAD_REPLY,
                fmt::format("Error setting pcic mode on device"));
            }
        }
    }
  else if (ticket_id == ifm3d::TICKET_COMMAND_t)
    {
      this->trigger_handler();
    }
  else
    {
      std::lock_guard<std::mutex> lock(this->_send_command_mutex);
      auto it = this->_send_command_promises.find(ticket_id);

      if (it == this->_send_command_promises.end())
        {
          LOG_WARNING("Received response for unknown ticket_id: {}",
                      ticket_id);
        }
      else
        {
          auto promise = it->second;
          char response_code = this->_payload_buffer.at(4);

          switch (response_code)
            {
            case '*':
              promise->set_value(PCICCommandResponse{std::monostate{}});
              break;

              case '?': {
                auto ex = ifm3d::Error(IFM3D_PCIC_BAD_REPLY,
                                       "Parameter-Id invalid or syntax error");
                promise->set_exception(std::make_exception_ptr(ex));
              }
              break;

              case '!': {
                auto ex = ifm3d::Error(IFM3D_PCIC_BAD_REPLY,
                                       "Application-level error on parameter");
                promise->set_exception(std::make_exception_ptr(ex));
              }
              break;

              default: {
                auto ex = ifm3d::Error(
                  IFM3D_PCIC_BAD_REPLY,
                  fmt::format("Unknown response code '{}'", response_code));
                promise->set_exception(std::make_exception_ptr(ex));
              }
              break;
            }
          this->_send_command_promises.erase(it);
        }
    }

  this->_payload_buffer.clear();

  this->_sock->async_read_some(
    asio::buffer(this->_ticket_buffer.data(), ifm3d::TICKET_SIZE),
    [this](auto&& error_code, auto&& bytes_xferd) {
      ticket_handler(std::forward<decltype(error_code)>(error_code),
                     std::forward<decltype(bytes_xferd)>(bytes_xferd),
                     0);
    });
}

inline void
ifm3d::FrameGrabber::Impl::image_handler()
{
  std::size_t buffer_size = this->_payload_buffer.size();
  bool buffer_valid =
    std::string(this->_payload_buffer.begin() + 4,
                this->_payload_buffer.begin() + 8) == "star" &&
    std::string(this->_payload_buffer.end() - 6,
                this->_payload_buffer.end() - 2) == "stop" &&
    this->_payload_buffer.at(buffer_size - 2) == '\r' &&
    this->_payload_buffer.at(buffer_size - 1) == '\n';

  if (buffer_valid)
    {
      try
        {
          auto result = this->_organizer->Organize(this->_payload_buffer,
                                                   this->_requested_images,
                                                   this->IsMasking());

          auto frame = std::make_shared<Frame>(result.images,
                                               result.timestamps,
                                               result.frame_count);

          this->_wait_for_frame_promise.set_value(frame);

          {
            std::lock_guard<std::mutex> lock(this->_wait_for_frame_mutex);
            this->_wait_for_frame_promise = std::promise<Frame::Ptr>();
            this->_wait_for_frame_future =
              this->_wait_for_frame_promise.get_future();
          }

          if (this->_new_frame_callback)
            {
              this->_new_frame_callback(frame);
            }
        }
      catch (const ifm3d::Error& ex)
        {
          // We might get empty frames when we requesting only algo debug but
          // also enable async notifications or async errors, so we just ignore
          // these frames.
          if (ex.code() != IFM3D_IMG_CHUNK_NOT_FOUND ||
              this->_requested_images.count(ifm3d::buffer_id::ALGO_DEBUG) <=
                0 ||
              this->_requested_images.size() != 1)
            {
              LOG_WARNING("Bad image: {}", ex.message());
            }
        }
      catch (const std::exception& ex)
        {
          LOG_WARNING("Bad image: {}", ex.what());
        }
    }
  else
    {
      LOG_WARNING("Bad image!");
    }
}

inline void
ifm3d::FrameGrabber::Impl::async_error_handler()
{
  std::size_t buffer_size = this->_payload_buffer.size();

  bool buffer_valid = buffer_size >= 4 + 9;

  if (buffer_valid)
    {
      auto error_code =
        std::stol(std::string(this->_payload_buffer.begin() + 4,
                              this->_payload_buffer.begin() + 4 + 9));

      std::string error_message = {};
      // 4-ticket 9-error_code 2-\r\n 1-:
      if (buffer_size > 4 + 9 + 2 + 1)
        {
          error_message =
            std::string(this->_payload_buffer.begin() + 4 + 9 + 1,
                        this->_payload_buffer.end() - 2);
        }
      if (_async_error_callback)
        {
          _async_error_callback(static_cast<int>(error_code), error_message);
        }
    }
  else
    {
      LOG_WARNING("Bad error message!");
    }
}

inline void
ifm3d::FrameGrabber::Impl::async_notification_handler()
{
  std::size_t buffer_size = this->_payload_buffer.size();

  bool buffer_valid = buffer_size >= 4 + 9;

  if (buffer_valid)
    {
      auto message_id = std::string(this->_payload_buffer.begin() + 4,
                                    this->_payload_buffer.begin() + 4 + 9);

      std::string payload = {};
      // 4-ticket 9-message_id 2-\r\n 1-:
      if (buffer_size > 4 + 9 + 2 + 1)
        {
          payload = std::string(this->_payload_buffer.begin() + 4 + 9 + 1,
                                this->_payload_buffer.end() - 2);
        }
      if (_async_notification_callback)
        {
          _async_notification_callback(message_id, payload);
        }
    }
  else
    {
      LOG_WARNING("Bad error message!");
    }
}

inline void
ifm3d::FrameGrabber::Impl::trigger_handler()
{
  if (this->_payload_buffer.at(4) != '*')
    {
      LOG_ERROR("Error Sending trigger on device");
      this->_trigger_feedback_promise.set_exception(
        std::make_exception_ptr(Error(IFM3D_CANNOT_SW_TRIGGER)));
    }
  else
    {
      this->_trigger_feedback_promise.set_value();
    }

  // re-intialize promise
  this->_trigger_feedback_promise = std::promise<void>();
  this->_trigger_feedback_future = _trigger_feedback_promise.get_future();
}

inline void
ifm3d::FrameGrabber::Impl::SetSchema(const json& schema)
{
  auto json = schema.dump();
  if (this->_cam->AmI(ifm3d::Device::DeviceFamily::O3X))
    {
      // O3X does not set the schema via PCIC, rather we set it via
      // XMLRPC using the camera interface.
      LOG_VERBOSE("o3x schema: \n{}", json);
      try
        {
          this->_cam->FromJSONStr(json);
        }
      catch (const std::exception& ex)
        {
          LOG_ERROR("Failed to set schema on O3X: {}", ex.what());
          LOG_WARNING("Running with currently applied schema");
        }
      return;
    }

  // Setting schema for O3D O3R devices
  const std::string c_length = fmt::format("{0}{1:09}", "c", json.size());
  send_command(TICKET_COMMAND_c, c_length + json);
  LOG_VERBOSE("schema: {}", json);
}

inline ifm3d::json
ifm3d::FrameGrabber::Impl::generate_default_schema()
{
  std::set<ifm3d::buffer_id> image_chunk_ids;
  for (auto buffer_id : this->_requested_images)
    {
      auto buffer_ids = get_image_chunks(buffer_id);
      image_chunk_ids.insert(buffer_ids.begin(), buffer_ids.end());
    }

  // Add O3D specific invariants
  if (this->_cam->AmI(ifm3d::Device::DeviceFamily::O3D))
    {
      // Add confidence image
      image_chunk_ids.insert(ifm3d::buffer_id::CONFIDENCE_IMAGE);
      image_chunk_ids.insert(ifm3d::buffer_id::EXTRINSIC_CALIB);
    }

  auto schema = ifm3d::make_schema(image_chunk_ids, _cam->WhoAmI());

  if (this->_cam->AmI(ifm3d::Device::DeviceFamily::O3R))
    {
      schema = ifm3d::make_o3r_schema_compatible_with_firmware(
        schema,
        _cam->FirmwareVersion());
    }
  return schema;
}

inline std::set<ifm3d::buffer_id>
ifm3d::FrameGrabber::Impl::get_image_chunks(buffer_id id)
{
  auto device_type = _cam->WhoAmI();

  switch (id)
    {
    case buffer_id::XYZ:
      if (device_type == ifm3d::Device::DeviceFamily::O3R)
        {
          return {buffer_id::TOF_INFO,
                  buffer_id::RADIAL_DISTANCE_IMAGE,
                  buffer_id::NORM_AMPLITUDE_IMAGE,
                  buffer_id::CONFIDENCE_IMAGE};
        }
      else if (device_type == ifm3d::Device::DeviceFamily::O3D)
        {
          return {
            buffer_id::CARTESIAN_X_COMPONENT,
            buffer_id::CARTESIAN_Y_COMPONENT,
            buffer_id::CARTESIAN_Z_COMPONENT,
          };
        }
      else
        {
          return {id};
        }
    case buffer_id::RADIAL_DISTANCE_IMAGE:
    case buffer_id::NORM_AMPLITUDE_IMAGE:
    case buffer_id::EXTRINSIC_CALIB:
    case buffer_id::INTRINSIC_CALIB:
    case buffer_id::INVERSE_INTRINSIC_CALIBRATION:
      if (device_type == ifm3d::Device::DeviceFamily::O3R)
        {
          return {id,
                  buffer_id::TOF_INFO,
                  buffer_id::RADIAL_DISTANCE_IMAGE,
                  buffer_id::NORM_AMPLITUDE_IMAGE,
                  buffer_id::CONFIDENCE_IMAGE};
        }
      else
        {
          return {id};
        }
    case buffer_id::RADIAL_DISTANCE_NOISE:
      if (device_type == ifm3d::Device::DeviceFamily::O3R)
        {
          return {id,
                  buffer_id::TOF_INFO,
                  buffer_id::RADIAL_DISTANCE_NOISE,
                  buffer_id::CONFIDENCE_IMAGE};
        }
      return {id};
    case buffer_id::REFLECTIVITY:
      if (device_type == ifm3d::Device::DeviceFamily::O3R)
        {
          return {id, buffer_id::CONFIDENCE_IMAGE};
        }
      return {id};

    case buffer_id::ALGO_DEBUG:
      return {};
    case buffer_id::O3R_ODS_RENDERED_ZONES:
    case buffer_id::O3R_ODS_FLAGS:
    case buffer_id::O3R_MCC_LIVE_IMAGE:
    case buffer_id::O3R_MCC_MOTION_IMAGE:
    case buffer_id::O3R_MCC_STATIC_IMAGE:
      if (device_type == ifm3d::Device::DeviceFamily::O3R)
        {
          this->_requested_images.insert(ifm3d::buffer_id::O3R_RESULT_ARRAY2D);
          return {id, buffer_id::O3R_RESULT_ARRAY2D};
        }
      else
        {
          return {id};
        }
    default:
      return {id};
    }
  return {};
}

inline void
ifm3d::FrameGrabber::Impl::OnAsyncError(AsyncErrorCallback callback)
{
  this->_async_error_callback = std::move(callback);
  // enable async error outputs
  if (this->_io_service != nullptr)
    {
      this->_io_service->post([this]() {
        send_command(TICKET_COMMAND_p, calculate_async_command());
      });
    }
}

inline void
ifm3d::FrameGrabber::Impl::OnAsyncNotification(
  AsyncNotificationCallback callback)
{
  this->_async_notification_callback = std::move(callback);
  if (this->_io_service != nullptr)
    {
      // enable async error outputs
      this->_io_service->post([this]() {
        send_command(TICKET_COMMAND_p, calculate_async_command());
      });
    }
}

inline void
ifm3d::FrameGrabber::Impl::OnError(ErrorCallback callback)
{
  this->_error_callback = std::move(callback);
}

inline void
ifm3d::FrameGrabber::Impl::report_error(const ifm3d::Error& err)
{
  if (this->_error_callback)
    {
      this->_error_callback(err);
    }
}

inline std::string
ifm3d::FrameGrabber::Impl::calculate_async_command()
{
  uint8_t p = 0;
  auto device_type = _cam->WhoAmI();
  // always enable async data
  p |= (1 << 0);

  // enable async errors if a callback is set
  if (this->_async_error_callback != nullptr)
    {
      p |= (1 << 1);
    }

  // enable async notifications if a callback is set
  if (this->_async_notification_callback != nullptr)
    {
      p |= (1 << 2);
    }

  if (device_type == ifm3d::Device::DeviceFamily::O3R)
    {
      // enable algodebug
      if (this->_requested_images.count(ifm3d::buffer_id::ALGO_DEBUG) > 0)
        {
          p = this->_requested_images.size() == 1 && p == 0x1 ? 0x8 : 0xF;
        }
    }
  return fmt::format("p{0:X}", p);
}

inline void
ifm3d::FrameGrabber::Impl::SetMasking(const bool masking)
{
  std::scoped_lock guard{this->_mutex_for_masking};
  _masking = masking;
}

inline bool
ifm3d::FrameGrabber::Impl::IsMasking()
{
  std::scoped_lock guard{this->_mutex_for_masking};
  return this->_masking;
}

std::string
ifm3d::FrameGrabber::Impl::generate_ticket_id()
{
  std::uint32_t id =
    (static_cast<std::uint32_t>(_next_ticket_id++) % 8900) + 1100;
  return std::to_string(id);
}

std::shared_future<ifm3d::FrameGrabber::PCICCommandResponse>
ifm3d::FrameGrabber::Impl::SendCommand(const PCICCommand& command)
{
  auto payload = command.SerializeData();

  auto promise = std::make_shared<std::promise<PCICCommandResponse>>();
  auto future = promise->get_future().share();

  if (!this->_sock || !this->_sock->is_open())
    {
      LOG_ERROR("PCIC socket is not connected");
      try
        {
          throw ifm3d::Error(IFM3D_NETWORK_ERROR,
                             "PCIC socket is not connected");
        }
      catch (...)
        {
          promise->set_exception(std::current_exception());
        }
      return future;
    }

  std::string ticket_id = this->generate_ticket_id();

  {
    std::lock_guard<std::mutex> lock(this->_send_command_mutex);
    this->_send_command_promises[ticket_id] = promise;
  }

  send_command(ticket_id, payload);
  return future;
}
#endif // IFM3D_FG_FRAMEGRABBER_IMPL_H
