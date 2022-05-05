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
#include <asio/use_future.hpp>
#include <asio.hpp>
#include <glog/logging.h>
#include <ifm3d/camera.h>
#include <ifm3d/fg/schema.h>
#include <default_organizer.hpp>
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
  static const std::string TICKET_ERROR = "0001";
  static const std::string TICKET_ALGO_DGB = "0020";
  static const std::string TICKET_COMMAND_c = "1000";
  static const std::string TICKET_COMMAND_t = "1001";
  static const std::string TICKET_COMMAND_p = "1002";

  //============================================================
  // Impl interface
  //============================================================
  class FrameGrabber::Impl
  {
  public:
    Impl(ifm3d::CameraBase::Ptr cam,
         std::optional<std::uint16_t> nat_pcic_port);
    ~Impl();

    void SWTrigger();

    void OnNewFrame(NewFrameCallback callback);

    bool Start(const std::set<ifm3d::image_id>& images);
    bool Stop();
    bool IsRunning();

    std::shared_future<Frame::Ptr> WaitForFrame();

    void SetOrganizer(std::unique_ptr<Organizer> organizer);

  protected:
    void Run();

    void SendCommand(const std::string& ticket_id, const std::string& command);
    void SendCommand(const std::string& ticket_id,
                     const std::vector<std::uint8_t>& command);
    void SetSchema();
    std::set<ifm3d::image_id> GetImageChunks(ifm3d::image_id id);
    //
    // ASIO event handlers
    //
    void ConnectHandler(const asio::error_code& ec);

    void TicketHandler(const asio::error_code& ec,
                       std::size_t bytes_xferd,
                       std::size_t bytes_read);

    void PayloadHandler(const asio::error_code& ec,
                        std::size_t bytes_xferd,
                        std::size_t bytes_read,
                        const std::string& ticket_id);

    void ImageHandler();
    void ErrorHandler();

    //---------------------
    // State
    //---------------------
    ifm3d::CameraBase::Ptr cam_;

    std::string cam_ip_;
    uint16_t pcic_port_;
    asio::io_service io_service_;
    asio::ip::tcp::socket sock_;
    asio::ip::tcp::endpoint endpoint_;
    std::unique_ptr<std::thread> thread_;
    std::unique_ptr<Organizer> organizer_;
    std::set<image_id> requested_images_;
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
    std::promise<Frame::Ptr> wait_for_frame_promise;
    std::shared_future<Frame::Ptr> wait_for_frame_future;

  }; // end: class FrameGrabber::Impl

} // end: namespace ifm3d

//============================================================
// Impl -- Implementation Details
//============================================================

//-------------------------------------
// ctor/dtor
//-------------------------------------
ifm3d::FrameGrabber::Impl::Impl(ifm3d::CameraBase::Ptr cam,
                                std::optional<std::uint16_t> pcic_port)
  : cam_(cam),
    cam_ip_(this->cam_->IP()),
    pcic_port_(pcic_port.value_or(ifm3d::DEFAULT_PCIC_PORT)),
    io_service_(),
    sock_(io_service_),
    organizer_(std::make_unique<DefaultOrganizer>()),
    wait_for_frame_future(wait_for_frame_promise.get_future())
{
  if (!pcic_port.has_value() && this->cam_->AmI(Camera::device_family::O3D))
    {
      try
        {
          this->pcic_port_ =
            std::stoi(this->cam_->DeviceParameter("PcicTcpPort"));
        }
      catch (const ifm3d::error_t& ex)
        {
          LOG(ERROR) << "Could not get PCIC Port of the camera: " << ex.what();
          LOG(WARNING) << "Assuming default PCIC port: "
                       << ifm3d::DEFAULT_PCIC_PORT;
          this->pcic_port_ = ifm3d::DEFAULT_PCIC_PORT;
        }
    }

  LOG(INFO) << "Camera connection info: ip=" << this->cam_ip_
            << ", port=" << this->pcic_port_;

  this->endpoint_ =
    asio::ip::tcp::endpoint(asio::ip::address::from_string(this->cam_ip_),
                            this->pcic_port_);
}

ifm3d::FrameGrabber::Impl::~Impl()
{
  VLOG(IFM3D_TRACE) << "FrameGrabber dtor running...";

  Stop();

  VLOG(IFM3D_TRACE) << "FrameGrabber destroyed.";
}

//-------------------------------------
// "Public" interface
//-------------------------------------

void
ifm3d::FrameGrabber::Impl::SWTrigger()
{
  if (this->cam_->AmI(ifm3d::CameraBase::device_family::O3X))
    {
      try
        {
          this->cam_->ForceTrigger();
        }
      catch (const ifm3d::error_t& ex)
        {
          LOG(ERROR) << "While trying to software trigger the camera: "
                     << ex.code() << " - " << ex.what();
        }

      return;
    }
  //
  // For O3D and other bi-directional PCIC implementations
  //

  try
    {
      this->io_service_.post([this]() { SendCommand(TICKET_COMMAND_t, "t"); });
    }
  catch(std::exception &e)
    {
      std::cout << e.what() << std::endl;
    }

}

void
ifm3d::FrameGrabber::Impl::OnNewFrame(NewFrameCallback callback)
{
  this->new_frame_callback_ = callback;
}

std::shared_future<ifm3d::Frame::Ptr>
ifm3d::FrameGrabber::Impl::WaitForFrame()
{
  return this->wait_for_frame_future;
}

bool
ifm3d::FrameGrabber::Impl::Start(const std::set<ifm3d::image_id>& images)
{
  if (!this->thread_)
    {
      this->requested_images_ = images;
      this->thread_ = std::make_unique<std::thread>(
        std::bind(&ifm3d::FrameGrabber::Impl::Run, this));

    //  std::this_thread::sleep_for(std::chrono::milliseconds(100));

      return true;
    }

  return false;
}

bool
ifm3d::FrameGrabber::Impl::Stop()
{
  if (this->thread_ && this->thread_->joinable())
    {
      this->io_service_.post(
        []() { throw ifm3d::error_t(IFM3D_THREAD_INTERRUPTED); });
      this->thread_->join();
      this->thread_ = nullptr;
      return true;
    }

  return false;
}

bool
ifm3d::FrameGrabber::Impl::IsRunning()
{
  return this->thread_ != nullptr;
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
ifm3d::FrameGrabber::Impl::Run()
{
  VLOG(IFM3D_TRACE) << "Framegrabber thread running...";
  asio::io_service::work work(this->io_service_);

  this->ticket_buffer_.clear();
  this->ticket_buffer_.resize(ifm3d::TICKET_SIZE);

  try
    {
      this->sock_.async_connect(
        this->endpoint_,
        std::bind(&Impl::ConnectHandler, this, std::placeholders::_1));

      this->io_service_.run();
    }
  catch (const ifm3d::error_t& ex)
    {
      if (ex.code() != IFM3D_THREAD_INTERRUPTED)
        {
          LOG(WARNING) << ex.what();
        }
    }
  catch (const std::exception& ex)
    {
      LOG(WARNING) << "Exception: " << ex.what();
    }

  LOG(INFO) << "FrameGrabber thread done.";
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

  asio::write(this->sock_,
              std::vector<asio::const_buffer>{
                asio::buffer(prefix),
                asio::buffer(content.data(), content.size()),
                asio::buffer(suffix)});
}

void
ifm3d::FrameGrabber::Impl::ConnectHandler(const asio::error_code& ec)
{
  if (ec)
    {
      throw ifm3d::error_t(ec.value());
    }

  // Set the schema
  SetSchema();

  if (requested_images_.find(static_cast<image_id>(image_chunk::ALGO_DEBUG)) !=
      requested_images_.end())
    {
      SendCommand(TICKET_COMMAND_p, "p8");
    }

  this->sock_.async_read_some(
    asio::buffer(this->ticket_buffer_.data(), ifm3d::TICKET_SIZE),
    std::bind(&ifm3d::FrameGrabber::Impl::TicketHandler,
              this,
              std::placeholders::_1,
              std::placeholders::_2,
              0));
}

void
ifm3d::FrameGrabber::Impl::TicketHandler(const asio::error_code& ec,
                                         std::size_t bytes_xferd,
                                         std::size_t bytes_read)
{
  if (ec)
    {
      throw ifm3d::error_t(ec.value());
    }

  bytes_read += bytes_xferd;
  if (bytes_read < ifm3d::TICKET_SIZE)
    {
      this->sock_.async_read_some(
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

  this->ticket_buffer_.clear();

  std::size_t payload_size = std::stoi(payload_size_str);
  payload_buffer_.resize(payload_size);

  this->sock_.async_read_some(
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
  if (ec)
    {
      throw ifm3d::error_t(ec.value());
    }

  bytes_read += bytes_xferd;
  if (bytes_read < this->payload_buffer_.size())
    {
      this->sock_.async_read_some(
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
  else if (ticket_id == ifm3d::TICKET_ERROR)
    {
      this->ErrorHandler();
    }
  else if (ticket_id == ifm3d::TICKET_COMMAND_c)
    {
      if (this->payload_buffer_.at(4) != '*')
        {
          LOG(ERROR) << "Error Setting Schema on device";
        }
    }
  else if (ticket_id == ifm3d::TICKET_COMMAND_t)
    {
      if (this->payload_buffer_.at(4) != '*')
        {
          LOG(ERROR) << "Error Sending trigger on device";
        }
    }

  this->payload_buffer_.clear();

  this->sock_.async_read_some(
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
      auto result = this->organizer_->Organize(this->payload_buffer_,
                                               this->requested_images_);
      auto frame = std::make_shared<Frame>(result.images, result.timestamps);

      this->wait_for_frame_promise.set_value(frame);

      this->wait_for_frame_promise = std::promise<Frame::Ptr>();
      this->wait_for_frame_future = this->wait_for_frame_promise.get_future();

      if (this->new_frame_callback_)
        {
          this->new_frame_callback_(frame);
        }
    }
  else
    {
      LOG(WARNING) << "Bad image!";
    }
}

void
ifm3d::FrameGrabber::Impl::ErrorHandler()
{
  std::size_t buffer_size = this->payload_buffer_.size();

  bool buffer_valid = buffer_size >= 4 + 9;

  if (buffer_valid)
    {
      auto error_code = std::stoi(std::string(this->payload_buffer_.end() - 9,
                                              this->payload_buffer_.end()));
      auto error_message = std::string(this->payload_buffer_.begin() + 4,
                                       this->payload_buffer_.end() - 9);

      // TODO
    }
  else
    {
      LOG(WARNING) << "Bad error message!";
    }
}

void
ifm3d::FrameGrabber::Impl::SetSchema()
{
  std::set<ifm3d::image_id> image_chunk_ids;
  for (auto image_id : this->requested_images_)
    {
      auto image_ids = GetImageChunks(image_id);
      image_chunk_ids.insert(image_ids.begin(), image_ids.end());
    }

  // Add confidence image
  image_chunk_ids.insert(ifm3d::image_id::CONFIDENCE);
  // Add O3D specific invariants
  if (this->cam_->AmI(ifm3d::CameraBase::device_family::O3D))
    {
      image_chunk_ids.insert(ifm3d::image_id::EXTRINSIC_CALIBRATION);
    }

  std::string schema = ifm3d::make_schema(image_chunk_ids, cam_->WhoAmI());

  if (this->cam_->AmI(ifm3d::CameraBase::device_family::O3X))
    {
      // O3X does not set the schema via PCIC, rather we set it via
      // XMLRPC using the camera interface.
      VLOG(IFM3D_PROTO_DEBUG) << "o3x schema: " << std::endl << schema;
      try
        {
          this->cam_->FromJSONStr(schema);
        }
      catch (const std::exception& ex)
        {
          LOG(ERROR) << "Failed to set schema on O3X: " << ex.what();
          LOG(WARNING) << "Running with currently applied schema";
        }
      return;
    }

  // Setting schema for O3D O3R devices
  const std::string c_length = fmt::format("{0}{1:09}", "c", schema.size());
  SendCommand(TICKET_COMMAND_c, c_length + schema);
  VLOG(IFM3D_PROTO_DEBUG) << "schema: " << schema;
  return;
}

std::set<ifm3d::image_id>
ifm3d::FrameGrabber::Impl::GetImageChunks(image_id id)
{
  auto device_type = cam_->WhoAmI();

  switch (static_cast<image_id>(id))
    {
      case image_id::XYZ: {
        if (device_type == ifm3d::CameraBase::device_family::O3R)
          return {
            image_id::XYZ,
            image_id::O3R_DISTANCE_IMAGE_INFORMATION,
            image_id::RADIAL_DISTANCE,
            image_id::AMPLITUDE,
          };
        else if (device_type == ifm3d::CameraBase::device_family::O3D)
          return {
            image_id::CARTESIAN_X,
            image_id::CARTESIAN_Y,
            image_id::CARTESIAN_Z,
          };
        else
          return {id};
      }
    case image_id::RADIAL_DISTANCE:
    case image_id::AMPLITUDE:
    case image_id::EXPOSURE_TIME:
    case image_id::EXTRINSIC_CALIBRATION:
    case image_id::INTRINSIC_CALIBRATION:
      case image_id::INVERSE_INTRINSIC_CALIBRATION: {
        if (device_type == ifm3d::CameraBase::device_family::O3R)
          {
            return {id,
                    image_id::O3R_DISTANCE_IMAGE_INFORMATION,
                    image_id::RADIAL_DISTANCE,
                    image_id::AMPLITUDE};
          }
        else
          {
            return {id};
          }
      }

    default:
      return {id};
    }
  return {};
}

#endif // IFM3D_FG_FRAMEGRABBER_IMPL_H
