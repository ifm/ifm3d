/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_FG_FRAMEGRABBER_IMPL_H
#define IFM3D_FG_FRAMEGRABBER_IMPL_H

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>
#include <mutex>
#include <asio/use_future.hpp>
#include <asio.hpp>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/fg/frame_grabber_export.h>
#include <ifm3d/device.h>
#include <ifm3d/fg/schema.h>
#include <o3d_organizer.hpp>
#include <o3x_organizer.hpp>
#include <o3r_organizer3D.hpp>
#include <o3r_organizer.hpp>
#include <fmt/core.h>

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

  //============================================================
  // Impl interface
  //============================================================
  class IFM3D_FRAME_GRABBER_LOCAL FrameGrabber::Impl
  {
  public:
    Impl(ifm3d::Device::Ptr cam, std::optional<std::uint16_t> nat_pcic_port);
    ~Impl();

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

    void SetMasking(const bool masking);
    bool IsMasking();

  protected:
    void Run(const std::optional<json>& schema);

    void SendCommand(const std::string& ticket_id, const std::string& command);
    void SendCommand(const std::string& ticket_id,
                     const std::vector<std::uint8_t>& command);
    json GenerateDefaultSchema();
    std::set<ifm3d::buffer_id> GetImageChunks(ifm3d::buffer_id id);
    //
    // ASIO event handlers
    //
    void ConnectHandler(const std::optional<json>& schema);

    void TicketHandler(const asio::error_code& ec,
                       std::size_t bytes_xferd,
                       std::size_t bytes_read);

    void PayloadHandler(const asio::error_code& ec,
                        std::size_t bytes_xferd,
                        std::size_t bytes_read,
                        const std::string& ticket_id);

    void ImageHandler();
    void AsyncErrorHandler();
    void AsyncNotificationHandler();
    void TriggerHandler();
    void ReportError(const ifm3d::Error& error);
    std::string CalculateAsyncCommand();

    //---------------------
    // State
    //---------------------
    ifm3d::Device::Ptr cam_;

    std::string cam_ip_;
    uint16_t pcic_port_;
    std::unique_ptr<asio::io_service> io_service_;
    std::mutex io_service_mutex_;
    std::unique_ptr<asio::ip::tcp::socket> sock_;
    std::shared_future<void> finish_future_;
    std::unique_ptr<Organizer> organizer_;
    std::set<buffer_id> requested_images_;
    bool is_ready_;
    bool masking_;
    std::mutex mutex_for_masking_;

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
    std::vector<std::uint8_t> ticket_buffer_;

    //
    // According to the PCIC V3 protocol, this will hold from the 4 digit
    // ticket sequence up to and including the ending CR + LF ('\r\n')
    //
    std::vector<std::uint8_t> payload_buffer_;

    FrameGrabber::NewFrameCallback new_frame_callback_;
    FrameGrabber::AsyncErrorCallback async_error_callback_;
    FrameGrabber::AsyncNotificationCallback async_notification_callback_;
    FrameGrabber::ErrorCallback error_callback_;

    std::promise<Frame::Ptr> wait_for_frame_promise;
    std::shared_future<Frame::Ptr> wait_for_frame_future;
    std::mutex wait_for_frame_mutex_;

    std::promise<void> trigger_feedback_promise_;
    std::shared_future<void> trigger_feedback_future_;

    std::promise<void> ready_promise_;
    std::shared_future<void> ready_future_;

  }; // end: class FrameGrabber::Impl

} // end: namespace ifm3d

//============================================================
// Impl -- Implementation Details
//============================================================

//-------------------------------------
// ctor/dtor
//-------------------------------------
ifm3d::FrameGrabber::Impl::Impl(ifm3d::Device::Ptr cam,
                                std::optional<std::uint16_t> pcic_port)
  : cam_(cam),
    cam_ip_(this->cam_->IP()),
    pcic_port_(pcic_port.value_or(ifm3d::DEFAULT_PCIC_PORT)),
    io_service_(),
    sock_(),
    wait_for_frame_future(wait_for_frame_promise.get_future()),
    trigger_feedback_future_(trigger_feedback_promise_.get_future()),
    ready_future_(ready_promise_.get_future()),
    finish_future_(std::async(std::launch::async, []() {})),
    is_ready_(false)
{
  auto device_type = this->cam_->WhoAmI();
  if (device_type == Device::device_family::O3D)
    {
      this->SetMasking(true);
      this->SetOrganizer(std::make_unique<O3DOrganizer>());
      if (!pcic_port.has_value())
        {
          try
            {
              this->pcic_port_ =
                std::stoi(this->cam_->DeviceParameter("PcicTcpPort"));
            }
          catch (const ifm3d::Error& ex)
            {
              LOG_ERROR("Could not get PCIC Port of the camera: {}",
                        ex.what());
              LOG_WARNING("Assuming default PCIC port: {}",
                          ifm3d::DEFAULT_PCIC_PORT);
              this->pcic_port_ = ifm3d::DEFAULT_PCIC_PORT;
            }
        }
    }

  else if (device_type == Device::device_family::O3X)
    {
      this->SetMasking(true);
      this->SetOrganizer(std::make_unique<O3XOrganizer>());
    }
  else if (device_type == Device::device_family::O3R)
    {
      this->SetMasking(false);

      auto ports_info =
        std::dynamic_pointer_cast<ifm3d::O3R>(this->cam_)->Ports();

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
      if (organizer_ == nullptr)
        {
          this->SetOrganizer(std::make_unique<O3ROrganizer>());
          LOG_WARNING(
            "The specified port is not compatible with the connected device. "
            "Falling back to the default Organizer");
        }
    }

  if (organizer_ == nullptr)
    {
      LOG_ERROR(
        "The device's PCIC port does not match any compatible organizers");
      throw ifm3d::Error(IFM3D_DEVICE_PORT_INCOMPATIBLE_WITH_ORGANIZER);
    }

  LOG_INFO("Camera connection info: ip={}, port={}",
           this->cam_ip_,
           this->pcic_port_);
}

ifm3d::FrameGrabber::Impl::~Impl()
{
  LOG_VERBOSE("FrameGrabber dtor running...");

  Stop().wait();

  LOG_VERBOSE("FrameGrabber destroyed.");
}

//-------------------------------------
// "Public" interface
//-------------------------------------

std::shared_future<void>
ifm3d::FrameGrabber::Impl::SWTrigger()
{
  this->io_service_->post([this]() {
    auto reinitialize_promise = [this]() {
      this->trigger_feedback_promise_ = std::promise<void>();
      this->trigger_feedback_future_ = trigger_feedback_promise_.get_future();
    };
    try
      {
        if (this->cam_->AmI(ifm3d::Device::device_family::O3X))
          {
            this->cam_->ForceTrigger();
            this->trigger_feedback_promise_.set_value();
            reinitialize_promise();
          }
        else
          {
            SendCommand(TICKET_COMMAND_t, "t");
          }
      }
    catch (const ifm3d::Error& ex)
      {
        LOG_ERROR("While trying to software trigger the camera: {} = {}",
                  ex.code(),
                  ex.what());
        this->trigger_feedback_promise_.set_exception(
          std::current_exception());
        reinitialize_promise();
      }
    catch (std::exception& e)
      {
        this->trigger_feedback_promise_.set_exception(
          std::current_exception());
        reinitialize_promise();
      }
  });

  return this->trigger_feedback_future_;
}

void
ifm3d::FrameGrabber::Impl::OnNewFrame(NewFrameCallback callback)
{
  this->new_frame_callback_ = callback;
}

std::shared_future<ifm3d::Frame::Ptr>
ifm3d::FrameGrabber::Impl::WaitForFrame()
{
  std::lock_guard<std::mutex> lock(this->wait_for_frame_mutex_);
  return this->wait_for_frame_future;
}

std::shared_future<void>
ifm3d::FrameGrabber::Impl::Start(const std::set<ifm3d::buffer_id>& images,
                                 const std::optional<json>& schema)
{
  std::lock_guard<std::mutex> lock(this->io_service_mutex_);
  if (!this->io_service_)
    {
      this->io_service_ = std::make_unique<asio::io_service>();
      this->sock_ = std::make_unique<asio::ip::tcp::socket>(*io_service_);
      this->requested_images_ = images;

      this->finish_future_ = std::async(
        std::launch::async,
        [this](const std::optional<json>& schema) { this->Run(schema); },
        schema);
      return this->ready_future_;
    }

  std::promise<void> promise;
  promise.set_value();
  return promise.get_future();
}

std::shared_future<void>
ifm3d::FrameGrabber::Impl::Stop()
{
  std::lock_guard<std::mutex> lock(this->io_service_mutex_);
  if (this->io_service_)
    {
      this->io_service_->post(
        []() { throw ifm3d::Error(IFM3D_THREAD_INTERRUPTED); });
    }

  return this->finish_future_;
}

bool
ifm3d::FrameGrabber::Impl::IsRunning()
{
  return this->io_service_ != nullptr;
}

void
ifm3d::FrameGrabber::Impl::SetOrganizer(std::unique_ptr<Organizer> organizer)
{
  this->organizer_ = std::move(organizer);
}

//-------------------------------------
// "Private" interface
//-------------------------------------

void
ifm3d::FrameGrabber::Impl::Run(const std::optional<json>& schema)
{
  LOG_DEBUG("Framegrabber thread running...");

  this->ticket_buffer_.clear();
  this->ticket_buffer_.resize(ifm3d::TICKET_SIZE);

  std::optional<ifm3d::Error> error;

  try
    {
      this->sock_->connect(
        asio::ip::tcp::endpoint(asio::ip::address::from_string(this->cam_ip_),
                                this->pcic_port_));

      this->ConnectHandler(schema);

      this->io_service_->run();
    }
  catch (const ifm3d::Error& ex)
    {
      if (ex.code() != IFM3D_THREAD_INTERRUPTED)
        {
          LOG_WARNING(ex.what());
          this->ReportError(ex);
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
      error = ifm3d::Error(IFM3D_SYSTEM_ERROR,
                           fmt::format("{0}: {1}",
                                       static_cast<int>(err.code().value()),
                                       err.what()));
    }
  catch (const std::exception& err)
    {
      error = ifm3d::Error(IFM3D_SYSTEM_ERROR, fmt::format("{1}", err.what()));
    }
  try
    {
      this->sock_->shutdown(asio::socket_base::shutdown_both);
      this->sock_->close();
    }
  catch (const std::system_error& err)
    {
      this->ReportError(
        ifm3d::Error(IFM3D_SYSTEM_ERROR,
                     fmt::format("{0}: {1}",
                                 static_cast<int>(err.code().value()),
                                 err.what())));
      LOG_WARNING("System error {}: {}",
                  static_cast<int>(err.code().value()),
                  err.what());
    }

  {
    std::lock_guard<std::mutex> lock(this->io_service_mutex_);
    this->sock_.reset();
    this->io_service_.reset();
  }

  this->ready_promise_ = std::promise<void>();
  this->ready_future_ = this->ready_promise_.get_future();
  this->is_ready_ = false;

  if (error.has_value())
    {
      LOG_WARNING("Exception: {}: {}",
                  static_cast<int>(error.value().code()),
                  error.value().what());
      auto ex_ptr = std::make_exception_ptr(error.value());
      this->wait_for_frame_promise.set_exception(ex_ptr);
      if (!this->is_ready_)
        {
          this->ready_promise_.set_exception(ex_ptr);
        }
    }

  LOG_INFO("FrameGrabber thread done.");
}

void
ifm3d::FrameGrabber::Impl::SendCommand(const std::string& ticket_id,
                                       const std::string& content)
{
  return SendCommand(
    ticket_id,
    std::vector<std::uint8_t>(content.begin(), content.end()));
}

void
ifm3d::FrameGrabber::Impl::SendCommand(
  const std::string& ticket_id,
  const std::vector<std::uint8_t>& content)
{
  std::string prefix =
    fmt::format("{0}L{1:09}\r\n{0}", ticket_id, 4 + content.size() + 2);
  std::string suffix = "\r\n";

  asio::write(*this->sock_,
              std::vector<asio::const_buffer>{
                asio::buffer(prefix),
                asio::buffer(content.data(), content.size()),
                asio::buffer(suffix)});
}

void
ifm3d::FrameGrabber::Impl::ConnectHandler(const std::optional<json>& schema)
{
  // Set the schema
  if (schema.has_value())
    {
      SetSchema(schema.value());
    }
  else if (!this->requested_images_.empty())
    {
      SetSchema(GenerateDefaultSchema());
    }

  SendCommand(TICKET_COMMAND_p, CalculateAsyncCommand());

  this->sock_->async_read_some(
    asio::buffer(this->ticket_buffer_.data(), ifm3d::TICKET_SIZE),
    std::bind(&ifm3d::FrameGrabber::Impl::TicketHandler,
              this,
              std::placeholders::_1,
              std::placeholders::_2,
              0));

  if (this->cam_->AmI(ifm3d::Device::device_family::O3X))
    {
      if (!this->is_ready_)
        {
          this->is_ready_ = true;
          this->ready_promise_.set_value();
        }
    }
}

void
ifm3d::FrameGrabber::Impl::TicketHandler(const asio::error_code& ec,
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
      this->sock_->async_read_some(
        asio::buffer(this->ticket_buffer_.data() + bytes_read,
                     this->ticket_buffer_.size() - bytes_read),
        std::bind(&ifm3d::FrameGrabber::Impl::TicketHandler,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  bytes_read));
      return;
    }

  std::string ticket_id(this->ticket_buffer_.begin() + TICKET_ID_START,
                        this->ticket_buffer_.begin() + TICKET_ID_END);

  std::string payload_size_str(
    this->ticket_buffer_.begin() + PAYLOAD_SIZE_START,
    this->ticket_buffer_.begin() + PAYLOAD_SIZE_END);

  std::size_t payload_size = std::stoi(payload_size_str);
  payload_buffer_.resize(payload_size);

  this->sock_->async_read_some(
    asio::buffer(this->payload_buffer_.data(), payload_size),
    std::bind(&ifm3d::FrameGrabber::Impl::PayloadHandler,
              this,
              std::placeholders::_1,
              std::placeholders::_2,
              0,
              ticket_id));
}

void
ifm3d::FrameGrabber::Impl::PayloadHandler(const asio::error_code& ec,
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
  if (bytes_read < this->payload_buffer_.size())
    {
      this->sock_->async_read_some(
        asio::buffer(this->payload_buffer_.data() + bytes_read,
                     this->payload_buffer_.size() - bytes_read),
        std::bind(&ifm3d::FrameGrabber::Impl::PayloadHandler,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  bytes_read,
                  ticket_id));
      return;
    }

  if (ticket_id == ifm3d::TICKET_IMAGE || ticket_id == ifm3d::TICKET_ALGO_DGB)
    {
      this->ImageHandler();
    }
  else if (ticket_id == ifm3d::TICKET_ASYNC_ERROR)
    {
      this->AsyncErrorHandler();
    }
  else if (ticket_id == ifm3d::TICKET_ASYNC_NOTIFICATION)
    {
      this->AsyncNotificationHandler();
    }
  else if (ticket_id == ifm3d::TICKET_COMMAND_c)
    {
      if (this->payload_buffer_.at(4) != '*')
        {
          LOG_ERROR("Error setting pcic schema on device");
          throw ifm3d::Error(
            IFM3D_PCIC_BAD_REPLY,
            fmt::format("Error setting pcic schema on device"));
        }
    }
  else if (ticket_id == ifm3d::TICKET_COMMAND_p)
    {
      if (!this->is_ready_)
        {
          if (this->payload_buffer_.at(4) == '*')
            {
              this->is_ready_ = true;
              this->ready_promise_.set_value();
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
      this->TriggerHandler();
    }

  this->payload_buffer_.clear();

  this->sock_->async_read_some(
    asio::buffer(this->ticket_buffer_.data(), ifm3d::TICKET_SIZE),
    std::bind(&ifm3d::FrameGrabber::Impl::TicketHandler,
              this,
              std::placeholders::_1,
              std::placeholders::_2,
              0));
}

void
ifm3d::FrameGrabber::Impl::ImageHandler()
{
  std::size_t buffer_size = this->payload_buffer_.size();

  bool buffer_valid =
    std::string(this->payload_buffer_.begin() + 4,
                this->payload_buffer_.begin() + 8) == "star" &&
    std::string(this->payload_buffer_.end() - 6,
                this->payload_buffer_.end() - 2) == "stop" &&
    this->payload_buffer_.at(buffer_size - 2) == '\r' &&
    this->payload_buffer_.at(buffer_size - 1) == '\n';

  if (buffer_valid)
    {
      try
        {
          auto result = this->organizer_->Organize(this->payload_buffer_,
                                                   this->requested_images_,
                                                   this->IsMasking());
          auto frame = std::make_shared<Frame>(result.images,
                                               result.timestamps,
                                               result.frame_count);

          this->wait_for_frame_promise.set_value(frame);

          {
            std::lock_guard<std::mutex> lock(this->wait_for_frame_mutex_);
            this->wait_for_frame_promise = std::promise<Frame::Ptr>();
            this->wait_for_frame_future =
              this->wait_for_frame_promise.get_future();
          }

          if (this->new_frame_callback_)
            {
              this->new_frame_callback_(frame);
            }
        }
      catch (std::exception ex)
        {
          LOG_WARNING("Bad image: {}", ex.what());
        }
    }
  else
    {
      LOG_WARNING("Bad image!");
    }
}

void
ifm3d::FrameGrabber::Impl::AsyncErrorHandler()
{
  std::size_t buffer_size = this->payload_buffer_.size();

  bool buffer_valid = buffer_size >= 4 + 9;

  if (buffer_valid)
    {
      auto error_code =
        std::stol(std::string(this->payload_buffer_.begin() + 4,
                              this->payload_buffer_.begin() + 4 + 9));

      std::string error_message = {};
      // 4-ticket 9-error_code 2-\r\n 1-:
      if (buffer_size > 4 + 9 + 2 + 1)
        {
          error_message =
            std::string(this->payload_buffer_.begin() + 4 + 9 + 1,
                        this->payload_buffer_.end() - 2);
        }
      if (async_error_callback_)
        {
          async_error_callback_(error_code, error_message);
        }
    }
  else
    {
      LOG_WARNING("Bad error message!");
    }
}

void
ifm3d::FrameGrabber::Impl::AsyncNotificationHandler()
{
  std::size_t buffer_size = this->payload_buffer_.size();

  bool buffer_valid = buffer_size >= 4 + 9;

  if (buffer_valid)
    {
      auto message_id = std::string(this->payload_buffer_.begin() + 4,
                                    this->payload_buffer_.begin() + 4 + 9);

      std::string payload = {};
      // 4-ticket 9-message_id 2-\r\n 1-:
      if (buffer_size > 4 + 9 + 2 + 1)
        {
          payload = std::string(this->payload_buffer_.begin() + 4 + 9 + 1,
                                this->payload_buffer_.end() - 2);
        }
      if (async_notification_callback_)
        {
          async_notification_callback_(message_id, payload);
        }
    }
  else
    {
      LOG_WARNING("Bad error message!");
    }
}

void
ifm3d::FrameGrabber::Impl::TriggerHandler()
{
  if (this->payload_buffer_.at(4) != '*')
    {
      LOG_ERROR("Error Sending trigger on device");
      this->trigger_feedback_promise_.set_exception(
        std::make_exception_ptr(Error(IFM3D_CANNOT_SW_TRIGGER)));
    }
  else
    {
      this->trigger_feedback_promise_.set_value();
    }

  // re-intialize promise
  this->trigger_feedback_promise_ = std::promise<void>();
  this->trigger_feedback_future_ = trigger_feedback_promise_.get_future();
  return;
}

void
ifm3d::FrameGrabber::Impl::SetSchema(const json& schema)
{
  auto json = schema.dump();
  if (this->cam_->AmI(ifm3d::Device::device_family::O3X))
    {
      // O3X does not set the schema via PCIC, rather we set it via
      // XMLRPC using the camera interface.
      LOG_VERBOSE("o3x schema: \n{}", json);
      try
        {
          this->cam_->FromJSONStr(json);
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
  SendCommand(TICKET_COMMAND_c, c_length + json);
  LOG_VERBOSE("schema: {}", json);
}

ifm3d::json
ifm3d::FrameGrabber::Impl::GenerateDefaultSchema()
{
  std::set<ifm3d::buffer_id> image_chunk_ids;
  for (auto buffer_id : this->requested_images_)
    {
      auto buffer_ids = GetImageChunks(buffer_id);
      image_chunk_ids.insert(buffer_ids.begin(), buffer_ids.end());
    }

  // Add O3D specific invariants
  if (this->cam_->AmI(ifm3d::Device::device_family::O3D))
    {
      // Add confidence image
      image_chunk_ids.insert(ifm3d::buffer_id::CONFIDENCE_IMAGE);
      image_chunk_ids.insert(ifm3d::buffer_id::EXTRINSIC_CALIB);
    }

  auto schema = ifm3d::make_schema(image_chunk_ids, cam_->WhoAmI());

  if (this->cam_->AmI(ifm3d::Device::device_family::O3R))
    {
      schema = ifm3d::make_o3r_schema_compatible_with_firmware(
        schema,
        cam_->FirmwareVersion());
    }
  return schema;
}

std::set<ifm3d::buffer_id>
ifm3d::FrameGrabber::Impl::GetImageChunks(buffer_id id)
{
  auto device_type = cam_->WhoAmI();

  switch (static_cast<buffer_id>(id))
    {
    case buffer_id::XYZ:
      if (device_type == ifm3d::Device::device_family::O3R)
        return {buffer_id::TOF_INFO,
                buffer_id::RADIAL_DISTANCE_IMAGE,
                buffer_id::NORM_AMPLITUDE_IMAGE,
                buffer_id::CONFIDENCE_IMAGE};
      else if (device_type == ifm3d::Device::device_family::O3D)
        return {
          buffer_id::CARTESIAN_X_COMPONENT,
          buffer_id::CARTESIAN_Y_COMPONENT,
          buffer_id::CARTESIAN_Z_COMPONENT,
        };
      else
        return {id};
    case buffer_id::RADIAL_DISTANCE_IMAGE:
    case buffer_id::NORM_AMPLITUDE_IMAGE:
    case buffer_id::EXPOSURE_TIME:
    case buffer_id::EXTRINSIC_CALIB:
    case buffer_id::INTRINSIC_CALIB:
    case buffer_id::INVERSE_INTRINSIC_CALIBRATION:
      if (device_type == ifm3d::Device::device_family::O3R)
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
      if (device_type == ifm3d::Device::device_family::O3R)
        {
          return {id,
                  buffer_id::TOF_INFO,
                  buffer_id::RADIAL_DISTANCE_NOISE,
                  buffer_id::CONFIDENCE_IMAGE};
        }
      return {id};
    case buffer_id::REFLECTIVITY:
      if (device_type == ifm3d::Device::device_family::O3R)
        {
          return {id, buffer_id::CONFIDENCE_IMAGE};
        }
      return {id};

    case buffer_id::ALGO_DEBUG:
      return {};
    default:
      return {id};
    }
  return {};
}

void
ifm3d::FrameGrabber::Impl::OnAsyncError(AsyncErrorCallback callback)
{
  this->async_error_callback_ = callback;
  // enable async error outputs
  if (this->io_service_ != nullptr)
    {
      this->io_service_->post(
        [this]() { SendCommand(TICKET_COMMAND_p, CalculateAsyncCommand()); });
    }
}

void
ifm3d::FrameGrabber::Impl::OnAsyncNotification(
  AsyncNotificationCallback callback)
{
  this->async_notification_callback_ = callback;
  if (this->io_service_ != nullptr)
    {
      // enable async error outputs
      this->io_service_->post(
        [this]() { SendCommand(TICKET_COMMAND_p, CalculateAsyncCommand()); });
    }
}

void
ifm3d::FrameGrabber::Impl::OnError(ErrorCallback callback)
{
  this->error_callback_ = callback;
}

void
ifm3d::FrameGrabber::Impl::ReportError(const ifm3d::Error& err)
{
  if (this->error_callback_)
    {
      this->error_callback_(err);
    }
}

std::string
ifm3d::FrameGrabber::Impl::CalculateAsyncCommand()
{
  uint8_t p = 0;

  // always enable async data
  p |= (1 << 0);

  // enable async errors if a callback is set
  if (this->async_error_callback_ != nullptr)
    {
      p |= (1 << 1);
    }

  // enable async notifications if a callback is set
  if (this->async_notification_callback_ != nullptr)
    {
      p |= (1 << 2);
    }

  // enable  algodebug
  if (this->requested_images_.count(ifm3d::buffer_id::ALGO_DEBUG) > 0)
    {
      p = this->requested_images_.size() == 1 && p == 0x1 ? 0x8 : 0xF;
    }
  return fmt::format("p{0:X}", p);
}

void
ifm3d::FrameGrabber::Impl::SetMasking(const bool masking)
{
  std::scoped_lock guard{this->mutex_for_masking_};
  masking_ = masking;
}

bool
ifm3d::FrameGrabber::Impl::IsMasking()
{
  std::scoped_lock guard{this->mutex_for_masking_};
  return this->masking_;
}

#endif // IFM3D_FG_FRAMEGRABBER_IMPL_H
