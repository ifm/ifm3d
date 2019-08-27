// -*- c++ -*-
/*
 * Copyright (C) 2019 ifm electronics GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
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
#include <ifm3d/camera/err.h>
#include <ifm3d/camera/logging.h>
#include <ifm3d/fg/byte_buffer.h>
#include <ifm3d/fg_udp/frame_grabber_udp.h>
#include <ifm3d/fg_udp/contrib/resulttransport/resulttransportclient.hpp>

using ifm::ResultTransportClient;

namespace ifm3d
{
  constexpr size_t MAX_DATAGRAM_SIZE = 65507;

  //============================================================
  // Impl interface
  //============================================================
  class FrameGrabberUdp::Impl
  {
  public:
    Impl(int port);
    ~Impl();

    bool WaitForFrame(
      long timeout_millis,
      std::function<void(std::vector<std::uint8_t>&)> set_bytes);

  protected:
    void Run();
    void Stop();

    //
    // ASIO event handlers
    //
    void ImageHandler(const boost::system::error_code& ec,
                      std::size_t bytes_xferd);

    //---------------------
    // State
    //---------------------
    int cam_port_;
    boost::asio::io_service io_service_;
    boost::asio::ip::udp::endpoint endpoint_;
    std::unique_ptr<boost::asio::ip::udp::socket> sock_;
    std::unique_ptr<std::thread> thread_;
    std::array<uint8_t, MAX_DATAGRAM_SIZE> receiveBuffer_;
    ResultTransportClient client_;
    std::unique_ptr<ResultTransportClient::Frame> frame_;

    //
    // synchronization/notification structures for the `frame_`
    //
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;

  }; // end: class FrameGrabberUdp::Impl

} // namespace ifm3d

//============================================================
// Impl -- Implementation Details
//============================================================

//-------------------------------------
// ctor/dtor
//-------------------------------------
ifm3d::FrameGrabberUdp::Impl::Impl(int port)
  : cam_port_(port), io_service_(), client_()
{
  this->endpoint_ =
      boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), cam_port_);

  this->sock_ = std::unique_ptr<boost::asio::ip::udp::socket>(
      new boost::asio::ip::udp::socket(io_service_, endpoint_));

  this->thread_ = std::unique_ptr<std::thread>(
      new std::thread(std::bind(&ifm3d::FrameGrabberUdp::Impl::Run, this)));
}

ifm3d::FrameGrabberUdp::Impl::~Impl()
{
  VLOG(IFM3D_TRACE) << "FrameGrabberUdp dtor running...";

  if (this->thread_ && this->thread_->joinable())
    {
      this->Stop();
      this->thread_->join();
    }

  VLOG(IFM3D_TRACE) << "FrameGrabberUdp destroyed.";
}

//-------------------------------------
// "Public" interface
// -- methods run in client's "main" thread
//-------------------------------------

bool ifm3d::FrameGrabberUdp::Impl::WaitForFrame(
    long timeout_millis,
    std::function<void(std::vector<std::uint8_t>&)> set_bytes)
{
  // mutex will unlock in `unique_lock` dtor if not explicitly unlocked prior
  // -- we use it here to ensure no deadlocks
  std::unique_lock<std::mutex> lock(this->frame_mutex_);

  try
    {
      if (timeout_millis <= 0)
        {
          this->frame_cv_.wait(lock);
        }
      else
        {
          if (this->frame_cv_.wait_for(
                  lock, std::chrono::milliseconds(timeout_millis)) ==
              std::cv_status::timeout)
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

  std::vector<image_chunk> incomplete_chunks;
  incomplete_chunks.resize(this->frame_->incompleteChannels.size());
  std::transform(
      this->frame_->incompleteChannels.begin(),
      this->frame_->incompleteChannels.end(), incomplete_chunks.begin(),
      [](ifm::ChannelType ch) { return static_cast<image_chunk>(ch); });

  set_bytes(this->frame_->frameData);

  this->frame_.reset();
  lock.unlock();
  return true;
}

//-------------------------------------
// "Private" interface
//-------------------------------------

void ifm3d::FrameGrabberUdp::Impl::Stop()
{
  this->io_service_.post(
      []() { throw ifm3d::error_t(IFM3D_THREAD_INTERRUPTED); });
}

//---------------------------------------
// The rest of these functions are running
// in our worker thread
//---------------------------------------

void ifm3d::FrameGrabberUdp::Impl::Run()
{
  VLOG(IFM3D_TRACE) << "FrameGrabberUdp thread running...";
  boost::asio::io_service::work work(this->io_service_);

  try
    {
      this->sock_->async_receive_from(
          boost::asio::buffer(this->receiveBuffer_), this->endpoint_,
          std::bind(&ifm3d::FrameGrabberUdp::Impl::ImageHandler, this,
                    std::placeholders::_1, std::placeholders::_2));

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

  LOG(INFO) << "FrameGrabberUdp thread done.";
}

void ifm3d::FrameGrabberUdp::Impl::ImageHandler(
    const boost::system::error_code& ec, std::size_t bytes_xferd)
{
  if (ec)
    {
      throw ifm3d::error_t(ec.value());
    }

  std::unique_ptr<ifm::ResultTransportClient::Frame> frame;
  if (auto e = client_.handlePacket(receiveBuffer_.data(), bytes_xferd, frame))
    {
      throw ifm3d::error_t(e.code);
    }

  if (frame)
    {
      if (std::find(frame->incompleteChannels.begin(),
                    frame->incompleteChannels.end(),
                    ifm::ChannelType::CONFIDENCE_IMAGE) !=
          frame->incompleteChannels.end())
        {
          LOG(WARNING) << "confidence image of frame corrupted, "
                          "frame will be discarded";
        }
      else
        {
          this->frame_mutex_.lock();
          this->frame_ = std::move(frame);
          this->frame_mutex_.unlock();

          // notify waiting client
          this->frame_cv_.notify_all();
        }
    }

  this->sock_->async_receive_from(
      boost::asio::buffer(this->receiveBuffer_), this->endpoint_,
      std::bind(&ifm3d::FrameGrabberUdp::Impl::ImageHandler, this,
                std::placeholders::_1, std::placeholders::_2));
}

#endif // __IFM3D_FRAME_GRABBER_FRAME_GRABBER_IMPL_H__
