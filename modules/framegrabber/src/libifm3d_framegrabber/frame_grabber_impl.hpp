// -*- c++ -*-
/*
 * Copyright (C) 2017 Love Park Robotics, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __IFM3D_FRAME_GRABBER_FRAME_GRABBER_IMPL_H__
#define __IFM3D_FRAME_GRABBER_FRAME_GRABBER_IMPL_H__

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
#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>
#include <glog/logging.h>
#include <ifm3d/camera/camera.h>
#include <ifm3d/camera/err.h>
#include <ifm3d/camera/logging.h>
#include <ifm3d/fg/byte_buffer.h>
#include <ifm3d/fg/schema.h>

namespace ifm3d
{
  // <Ticket><Length>CR+LF (16 bytes)
  const std::size_t TICKET_ID_SZ = 16;
  const std::string TICKET_image = "0000";
  const std::string TICKET_c = "1000";
  const std::string TICKET_t = "1001";

  //============================================================
  // Impl interface
  //============================================================
  class FrameGrabber::Impl
  {
  public:
    Impl(ifm3d::Camera::Ptr cam, std::uint16_t mask);
    ~Impl();

    virtual void SWTrigger();
    bool WaitForFrame(
      long timeout_millis,
      std::function<void(std::vector<std::uint8_t>&)> set_bytes);

  protected:
    void Run();
    void Stop();
    void SetUVecBuffer(std::uint16_t mask);
    void SetSchemaBuffer(std::uint16_t mask);
    void SetTriggerBuffer();

    //
    // ASIO event handlers
    //
    void TicketHandler(const boost::system::error_code& ec,
                       std::size_t bytes_xferd,
                       std::size_t bytes_read);

    void ImageHandler(const boost::system::error_code& ec,
                      std::size_t bytes_xferd,
                      std::size_t bytes_read);

    //---------------------
    // State
    //---------------------
    ifm3d::Camera::Ptr cam_;
    std::uint16_t mask_;

    std::string cam_ip_;
    int cam_port_;
    boost::asio::io_service io_service_;
    boost::asio::ip::tcp::socket sock_;
    boost::asio::ip::tcp::endpoint endpoint_;
    std::unique_ptr<std::thread> thread_;
    std::atomic<bool> pcic_ready_;
    std::vector<std::uint8_t> schema_buffer_;
    std::vector<std::uint8_t> trigger_buffer_;
    std::vector<std::uint8_t> uvec_buffer_;

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
    // Our frame grabber double buffers images when acquiring from the
    // camera. This implements the "back buffer" in the double buffering
    // scheme.
    //
    // According to the PCIC V3 protocol, this will hold from the 4 digit
    // ticket sequence up to and including the ending CR + LF ('\r\n')
    //
    std::vector<std::uint8_t> back_buffer_;

    //
    // The buffer that subscribed clients fetch data from
    //
    std::vector<std::uint8_t> front_buffer_;

    //
    // synchronization/notification structures for the `front_buffer_`
    //
    std::mutex front_buffer_mutex_;
    std::condition_variable front_buffer_cv_;

  }; // end: class FrameGrabber::Impl

} // end: namespace ifm3d

//============================================================
// Impl -- Implementation Details
//============================================================

//-------------------------------------
// ctor/dtor
//-------------------------------------
ifm3d::FrameGrabber::Impl::Impl(ifm3d::Camera::Ptr cam,
                                std::uint16_t mask)
  : cam_(cam),
    mask_(mask),
    cam_ip_(this->cam_->IP()),
    cam_port_(ifm3d::DEFAULT_PCIC_PORT),
    io_service_(),
    sock_(io_service_),
    pcic_ready_(false)
{
  this->SetSchemaBuffer(this->mask_);
  this->SetTriggerBuffer();
  this->SetUVecBuffer(this->mask_);

  if (! this->cam_->IsO3X())
    {
      try
        {
          this->cam_ip_ = this->cam_->IP();
          this->cam_port_ =
            std::stoi(this->cam_->DeviceParameter("PcicTcpPort"));
        }
      catch (const ifm3d::error_t& ex)
        {
          LOG(ERROR) << "Could not get PCIC Port of the camera: " << ex.what();
          LOG(WARNING) << "Assuming default PCIC port: "
                       << ifm3d::DEFAULT_PCIC_PORT;
          this->cam_port_ = ifm3d::DEFAULT_PCIC_PORT;
        }
    }

  LOG(INFO) << "Camera connection info: ip=" << this->cam_ip_
            << ", port=" << this->cam_port_;

  this->endpoint_ =
    boost::asio::ip::tcp::endpoint(
      boost::asio::ip::address::from_string(this->cam_ip_), this->cam_port_);

  //
  // XXX: Make this work on older C++/gcc versions
  //
  //  this->thread_ =
  //    std::make_unique<std::thread>(
  //      std::bind(&ifm3d::FrameGrabber::Impl::Run, this));
  this->thread_ =
    std::unique_ptr<std::thread>(
      new std::thread(std::bind(&ifm3d::FrameGrabber::Impl::Run, this)));
}

ifm3d::FrameGrabber::Impl::~Impl()
{
  VLOG(IFM3D_TRACE) << "FrameGrabber dtor running...";

  if (this->thread_ && this->thread_->joinable())
    {
      this->Stop();
      this->thread_->join();
    }

  VLOG(IFM3D_TRACE) << "FrameGrabber destroyed.";
}

//-------------------------------------
// "Public" interface
// -- methods run in client's "main" thread
//-------------------------------------

void
ifm3d::FrameGrabber::Impl::SWTrigger()
{
  if (this->cam_->IsO3X())
    {
      try
        {
          this->cam_->ForceTrigger();
        }
      catch(const ifm3d::error_t& ex)
        {
          LOG(ERROR) << "While trying to software trigger the camera: "
                     << ex.code() << " - " << ex.what();
        }

      return;
    }

  //
  // For O3D and other bi-directional PCIC implementations
  //
  int i = 0;
  while (! this->pcic_ready_.load())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      i++;

      if (i > 2000)
        {
          LOG(WARNING) << "pcic_ready_ flag not set!";
          return;
        }
    }

  this->io_service_.post(
    [this]()
    {
      boost::asio::async_write(
        this->sock_,
        boost::asio::buffer(this->trigger_buffer_.data(),
                            this->trigger_buffer_.size()),
        [](const boost::system::error_code& ec,
           std::size_t bytes_xferd)
        {
          if (ec) { throw ifm3d::error_t(ec.value()); }
        });
    });
}

bool
ifm3d::FrameGrabber::Impl::WaitForFrame(
  long timeout_millis,
  std::function<void(std::vector<std::uint8_t>&)> set_bytes)
{
  // mutex will unlock in `unique_lock` dtor if not explicitly unlocked prior
  // -- we use it here to ensure no deadlocks
  std::unique_lock<std::mutex> lock(this->front_buffer_mutex_);

  try
    {
      // Store the current pointer backing the front buffer - this will be the
      // condition checked by the condition_variable predicate (pointer should
      // have changed) below
      std::uint8_t* initial_buff_ptr = this->front_buffer_.data();
      auto predicate =
        [this, initial_buff_ptr]()
        {
          return this->front_buffer_.data() != initial_buff_ptr;
        };

      if (timeout_millis <= 0)
        {
          this->front_buffer_cv_.wait(lock, predicate);
        }
      else
        {
          if (!this->front_buffer_cv_.wait_for(
                lock,
                std::chrono::milliseconds(timeout_millis),
                predicate))
            {
              VLOG(IFM3D_TRACE)
                << "Timeout waiting for image buffer from camera";
              return false;
            }
        }
    }
  catch (const std::system_error& ex)
    {
      LOG(WARNING) << "WaitForFrame: " << ex.what();
      return false;
    }

  // if (copy_buff)
  //   {
  //     std::size_t sz = this->front_buffer_.size();
  //     buff.resize(sz);

  //     std::copy(this->front_buffer_.begin(),
  //               this->front_buffer_.begin() + sz,
  //               buff.begin());
  //   }
  // else
  //   {
  //     buff.swap(this->front_buffer_);
  //   }
  set_bytes(this->front_buffer_);

  lock.unlock();
  return true;
}

//-------------------------------------
// "Private" interface
//-------------------------------------
void
ifm3d::FrameGrabber::Impl::SetUVecBuffer(std::uint16_t mask)
{
  //
  // For O3X, we cache the unit vectors and fake it to the
  // ByteBuffer that these data came over PCIC rather than
  // over XML-RPC
  //

  if (! this->cam_->IsO3X())
    {
      return;
    }

  if ((mask & ifm3d::IMG_UVEC) != ifm3d::IMG_UVEC)
    {
      return;
    }

  try
    {
      VLOG(IFM3D_TRACE) << "Caching unit vectors from xmlrpc...";
      this->uvec_buffer_ = this->cam_->UnitVectors();
      if (FLAGS_v >= IFM3D_PROTO_DEBUG)
        {
          std::stringstream ss;
          ss << "[";
          std::size_t len = this->uvec_buffer_.size();
          for (std::size_t i = 0; i < len; ++i)
            {
              ss << std::hex << std::setw(2)
                 << std::setfill('0') << (int) this->uvec_buffer_.at(i);

              if (i < (len - 1))
                {
                  ss << ",";
                }
            }
          ss << "]";

          VLOG(IFM3D_PROTO_DEBUG) << "Unit vectors: " << std::endl
                                  << ss.str();
        }
    }
  catch (const ifm3d::error_t& ex)
    {
      LOG(ERROR) << "Could not fetch unit vectors from XML-RPC!";
      LOG(ERROR) << ex.code() << " : " << ex.what();
    }
}


void
ifm3d::FrameGrabber::Impl::SetSchemaBuffer(std::uint16_t mask)
{
  if((mask & ifm3d::INTR_CAL) == ifm3d::INTR_CAL && (!this->cam_->IsO3D()))
    {
      LOG(ERROR) << "Failed to set schema on O3X: "
                 << "Intrinsic parameter not supported by Device";
      throw ifm3d::error_t(IFM3D_INTRINSIC_CALIBRATION_UNSUPPORTED_DEVICE);
    }

  if((mask & ifm3d::INTR_CAL) == ifm3d::INTR_CAL && this->cam_->IsO3D()
     && ! this->cam_->CheckMinimumFirmwareVersion(
                        ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_MAJOR,
                        ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_MINOR,
                        ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_PATCH))
    {
      LOG(ERROR) << "Failed to set schema on O3D: "
                 << "Intrinsic parameter not supported by Firmware";
      throw ifm3d::error_t(IFM3D_INTRINSIC_CALIBRATION_UNSUPPORTED_FIRMWARE);
    }

  if(((mask & ifm3d::INV_INTR_CAL) == ifm3d::INV_INTR_CAL)
     && (!this->cam_->IsO3D()))
	  {
	    LOG(ERROR) << "Failed to set schema on O3X: "
                 << "Inverse intrinsic parameter not supported by Device";
	    throw ifm3d::error_t(
                    IFM3D_INVERSE_INTRINSIC_CALIBRATION_UNSUPPORTED_DEVICE);
	  }

  if((mask & ifm3d::INV_INTR_CAL) == ifm3d::INV_INTR_CAL && this->cam_->IsO3D()
	   && !this->cam_->CheckMinimumFirmwareVersion(
                       ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MAJOR,
                       ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MINOR,
                       ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_PATCH))
	  {
	    LOG(ERROR) << "Failed to set schema on O3D: "
                 << "Inverse intrinsic parameter not supported by Firmware";
	    throw ifm3d::error_t(
                    IFM3D_INVERSE_INTRINSIC_CALIBRATION_UNSUPPORTED_FIRMWARE);
	  }

  if((mask & ifm3d::JSON_MODEL) == ifm3d::JSON_MODEL && (this->cam_->IsO3X()))
    {
      LOG(ERROR) << "Failed to set schema on O3X: "
                 << "json data not supported on O3X";
      throw ifm3d::error_t(IFM3D_INVALID_PARAM);
    }

  if(this->cam_->IsO3X())
    {
      // O3X does not set the schema via PCIC, rather we set it via
      // XMLRPC using the camera interface.
      // NOTE: Our internal `this->schema_buffer_` is not needed for O3X, so we
      // don't waste any time filling it.
      std::string o3xjson = ifm3d::make_o3x_json_from_mask(mask);
      VLOG(IFM3D_PROTO_DEBUG) << "o3x schema: "
                              << std::endl
                              << o3xjson;
      try
        {
          this->cam_->FromJSONStr(o3xjson);
        }
      catch (const std::exception& ex)
        {
          LOG(ERROR) << "Failed to set schema on O3X: "
                     << ex.what();
          LOG(WARNING) << "Running with currently applied schema";
        }

      return;
    }

  // For now, we assume all other cameras will work like the O3D

  // O3D does not support an ambient light grayscale image, so, we just
  // make sure the mask does not specify it
  mask &= ~ifm3d::IMG_GRAY;

  std::string schema = ifm3d::make_schema(mask);
  std::size_t c_len = 4 + 1 + 9 + schema.size() + 2;
  std::ostringstream str;
  str << ifm3d::TICKET_c
      << 'L' << std::setfill('0') << std::setw(9) << c_len
      << '\r' << '\n'
      << ifm3d::TICKET_c << 'c'
      << std::setfill('0') << std::setw(9)
      << schema.size()
      << schema
      << '\r' << '\n';

  std::string c_command = str.str();
  this->schema_buffer_.assign(c_command.begin(), c_command.end());
  VLOG(IFM3D_PROTO_DEBUG) << "c_command: " << c_command;
}

void
ifm3d::FrameGrabber::Impl::SetTriggerBuffer()
{
  if (this->cam_->IsO3X())
    {
      // O3X does not S/W trigger over PCIC, so, no need to set the trigger
      // buffer
      return;
    }

  int t_len = 4 + 1 + 2;
  std::ostringstream str;
  str << ifm3d::TICKET_t
      << 'L' << std::setfill('0') << std::setw(9) << t_len
      << '\r' << '\n'
      << ifm3d::TICKET_t << 't' << '\r' << '\n';

  std::string t_command = str.str();
  this->trigger_buffer_.assign(t_command.begin(), t_command.end());
  VLOG(IFM3D_PROTO_DEBUG) << "t_command: " << t_command;
}

void
ifm3d::FrameGrabber::Impl::Stop()
{
  this->io_service_.post(
    []() { throw ifm3d::error_t(IFM3D_THREAD_INTERRUPTED); });
}

//---------------------------------------
// The rest of these functions are running
// in our worker thread
//---------------------------------------

void
ifm3d::FrameGrabber::Impl::Run()
{
  VLOG(IFM3D_TRACE) << "Framegrabber thread running...";
  boost::asio::io_service::work work(this->io_service_);

  // For non-O3X devices setting the schema via PCIC, we get acknowledgement of
  // our schema, then start processing the stream of pixel bytes
  auto result_schema_write_handler =
    [this](const boost::system::error_code& ec, std::size_t bytes_xferd)
    {
      if (ec) { throw ifm3d::error_t(ec.value()); }
      this->ticket_buffer_.clear();
      this->ticket_buffer_.resize(ifm3d::TICKET_ID_SZ);

      this->sock_.async_read_some(
        boost::asio::buffer(this->ticket_buffer_.data(),
                            ifm3d::TICKET_ID_SZ),
        std::bind(&ifm3d::FrameGrabber::Impl::TicketHandler,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  0));

      this->pcic_ready_.store(true);
    };

  try
    {
      // O3X should just start reading in pixel bytes once we establish our
      // connection to the PCIC daemon (PCIC data goes one-way on O3X)
      VLOG(IFM3D_TRACE) << "Connecting to PCIC...";
      if (this->cam_->IsO3X())
        {
          this->pcic_ready_.store(true);

          this->sock_.async_connect(
            this->endpoint_,
            [this](const boost::system::error_code& ec)
            {
              if (ec) { throw ifm3d::error_t(ec.value()); }
              this->ticket_buffer_.clear();
              this->ticket_buffer_.resize(ifm3d::TICKET_ID_SZ);

              this->sock_.async_read_some(
                boost::asio::buffer(this->ticket_buffer_.data(),
                                    ifm3d::TICKET_ID_SZ),
                std::bind(&ifm3d::FrameGrabber::Impl::TicketHandler,
                          this,
                          std::placeholders::_1,
                          std::placeholders::_2,
                          0));
            });
        }
      else
        {
          // assume everything else behaves like an O3D --
          // we need to first write our desired schema to the camera
          this->sock_.async_connect(
            this->endpoint_,
            [&, this](const boost::system::error_code& ec)
            {
              if (ec) { throw ifm3d::error_t(ec.value()); }
              boost::asio::async_write(
                this->sock_,
                boost::asio::buffer(this->schema_buffer_.data(),
                                    this->schema_buffer_.size()),
                result_schema_write_handler);
            });
        }

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
ifm3d::FrameGrabber::Impl::TicketHandler(const boost::system::error_code& ec,
                                         std::size_t bytes_xferd,
                                         std::size_t bytes_read)
{
  if (ec) { throw ifm3d::error_t(ec.value()); }

  bytes_read += bytes_xferd;
  if (bytes_read < ifm3d::TICKET_ID_SZ)
    {
      bytes_read +=
        boost::asio::read(
          this->sock_,
          boost::asio::buffer(&this->ticket_buffer_[bytes_read],
                              ifm3d::TICKET_ID_SZ - bytes_read));

      if (bytes_read != ifm3d::TICKET_ID_SZ)
        {
          LOG(ERROR) << "Timeout reading ticket!";
          throw ifm3d::error_t(IFM3D_IO_ERROR);
        }
    }

  std::string ticket;
  ticket.assign(this->ticket_buffer_.begin(),
                this->ticket_buffer_.begin() + 4);

  std::string payload_sz_str;
  payload_sz_str.assign(this->ticket_buffer_.begin() + 5,
                        this->ticket_buffer_.begin() + 14);
  int payload_sz = std::stoi(payload_sz_str);
  int ticket_sz = ifm3d::TICKET_ID_SZ;

  if (ticket != ifm3d::TICKET_image)
    {
      this->ticket_buffer_.resize(ticket_sz + payload_sz);

      bytes_read +=
        boost::asio::read(
          this->sock_,
          boost::asio::buffer(&this->ticket_buffer_[bytes_read],
                              (ticket_sz + payload_sz) - bytes_read));

      if (bytes_read != (ticket_sz + payload_sz))
        {
          LOG(ERROR) << "Timeout reading whole response!";
          LOG(ERROR) << "Got " << bytes_read << " bytes of "
                     << ticket_sz << " bytes expected";

          throw ifm3d::error_t(IFM3D_IO_ERROR);
        }
    }

  std::string ticket_str;
  ticket_str.assign(this->ticket_buffer_.begin(),
                    this->ticket_buffer_.end());
  VLOG(IFM3D_PROTO_DEBUG) << "Full ticket: '" << ticket_str << "'";

  if (ticket == ifm3d::TICKET_image)
    {
      if (ifm3d::verify_ticket_buffer(this->ticket_buffer_))
        {
          this->back_buffer_.resize(
            ifm3d::get_image_buffer_size(this->ticket_buffer_));

          this->sock_.async_read_some(
            boost::asio::buffer(this->back_buffer_.data(),
                                this->back_buffer_.size()),
            std::bind(&ifm3d::FrameGrabber::Impl::ImageHandler,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      0));
          return;
        }
      else
        {
          LOG(ERROR) << "Bad image ticket: " << ticket_str;
          throw ifm3d::error_t(IFM3D_PCIC_BAD_REPLY);
        }
    }
  else if ((ticket == ifm3d::TICKET_c) ||
           (ticket == ifm3d::TICKET_t))
    {
      if (this->ticket_buffer_.at(20) != '*')
        {
          LOG(ERROR) << "Bad ticket: " << ticket_str;

          if ((ticket == ifm3d::TICKET_t) &&
              (this->ticket_buffer_.at(20) == '!'))
            {
              LOG(WARNING) << "Are you software triggering in free-run mode?";
            }
          else
            {
              throw ifm3d::error_t(IFM3D_PCIC_BAD_REPLY);
            }
        }

      this->ticket_buffer_.clear();
      this->ticket_buffer_.resize(ifm3d::TICKET_ID_SZ);
      this->sock_.async_read_some(
        boost::asio::buffer(this->ticket_buffer_.data(),
                            ifm3d::TICKET_ID_SZ),
        std::bind(&ifm3d::FrameGrabber::Impl::TicketHandler,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  0));

      return;
    }
  else
    {
      LOG(ERROR) << "Unexpected ticket: " << ticket;
      throw std::logic_error("Unexpected ticket type: " + ticket);
    }
}

void
ifm3d::FrameGrabber::Impl::ImageHandler(const boost::system::error_code& ec,
                                        std::size_t bytes_xferd,
                                        std::size_t bytes_read)
{
  if (ec) { throw ifm3d::error_t(ec.value()); }

  bytes_read += bytes_xferd;

  if (bytes_read != this->back_buffer_.size())
    {
      this->sock_.async_read_some(
        boost::asio::buffer(&this->back_buffer_[bytes_read],
                            this->back_buffer_.size() - bytes_read),
        std::bind(&ifm3d::FrameGrabber::Impl::ImageHandler,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  bytes_read));
      return;
    }

  if (ifm3d::verify_image_buffer(this->back_buffer_))
    {
      // Move data to the front buffer in O(1)
      this->front_buffer_mutex_.lock();
      this->back_buffer_.swap(this->front_buffer_);
      // For O3X, copy in the unit vectors if necessary
      if (this->cam_->IsO3X() &&
          ((this->mask_ & ifm3d::IMG_UVEC) == ifm3d::IMG_UVEC))
        {
          VLOG(IFM3D_TRACE) << "Inserting unit vectors to front buffer";
          this->front_buffer_.insert(
            this->front_buffer_.begin()+ifm3d::IMG_BUFF_START,
            this->uvec_buffer_.begin(), this->uvec_buffer_.end());
        }
      this->front_buffer_mutex_.unlock();

      // notify waiting client
      this->front_buffer_cv_.notify_all();
    }
  else
    {
      LOG(WARNING) << "Bad image!";
    }

  this->ticket_buffer_.clear();
  this->ticket_buffer_.resize(ifm3d::TICKET_ID_SZ);
  this->sock_.async_read_some(
    boost::asio::buffer(this->ticket_buffer_.data(),
                        ifm3d::TICKET_ID_SZ),
    std::bind(&ifm3d::FrameGrabber::Impl::TicketHandler,
              this,
              std::placeholders::_1,
              std::placeholders::_2,
              0));
}

#endif // __IFM3D_FRAME_GRABBER_FRAME_GRABBER_IMPL_H__
