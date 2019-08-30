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

#ifndef __IFM3D_FRAME_GRABBER_UDP_FRAME_GRABBER_UDP_IMPL_H__
#define __IFM3D_FRAME_GRABBER_UDP_FRAME_GRABBER_UDP_IMPL_H__

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
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>
#include <glog/logging.h>
#include <ifm3d/camera/err.h>
#include <ifm3d/camera/logging.h>
#include <ifm3d/fg/byte_buffer.h>
#include <ifm3d/fg_udp/frame_grabber_udp.h>

namespace ifm3d
{
  constexpr size_t MAX_DATAGRAM_SIZE = 65507;

  const std::size_t PACKET_HEADER_SZ = 28;

  constexpr uint8_t  FRAME_HEADER[]      = { 's', 't', 'a', 'r' };
  constexpr uint8_t  FRAME_FOOTER[]      = { 's', 't', 'o', 'p' };
  constexpr uint32_t PACKET_MAGIC_NUMBER = static_cast<uint32_t>('#') << 0 |
                                           static_cast<uint32_t>('i') << 8 |
                                           static_cast<uint32_t>('f') << 16 |
                                           static_cast<uint32_t>('m') << 24;

  #pragma pack(push, 1)
  struct PacketHeader
  {
    uint32_t magic;
    uint32_t packet_counter;
    uint32_t frame_counter;
    uint16_t number_of_packets_in_frame;
    uint16_t packet_index_in_frame;
    uint16_t number_of_packets_in_channel;
    uint16_t index_of_packet_in_channel;
    uint32_t channel_id;
    uint32_t total_channel_length;
  };
  #pragma pack(pop)

  //============================================================
  // Impl interface
  //============================================================
  class FrameGrabberUdp::Impl
  {
  public:
    Impl(int port, std::uint16_t max_packet_size);
    ~Impl();

    bool WaitForFrame(
      long timeout_millis,
      std::function<void(std::vector<std::uint8_t>&)> set_bytes);

  protected:
    void Run();
    void Stop();

    void DeliverFrame();
    void ResetReceiveState();

    //
    // ASIO event handlers
    //
    void PacketHandler(const boost::system::error_code& ec,
                       std::size_t bytes_xferd);

    //---------------------
    // State
    //---------------------
    int cam_port_;
    boost::asio::io_service io_service_;
    boost::asio::ip::udp::endpoint endpoint_;
    std::unique_ptr<boost::asio::ip::udp::socket> sock_;
    std::unique_ptr<std::thread> thread_;
    std::uint16_t max_payload_size_;

    //
    // State of the current frame/channel
    //
    std::uint32_t frame_counter_;
    std::uint32_t channel_id_;
    std::uint16_t expected_channel_idx_;
    std::size_t bytes_xferd_for_channel_;

    //
    // Data buffers
    //
    // This implementation leverages boost::asio's support for scatter/gather
    // I/O to read image chunk data directly in to the proper spot in memory.
    // The idea is that in the best-case scenario (no dropped packets) there
    // will be no memcpys required to organize the data.
    //
    // To do this, we read the UDP header into a separate buffer than the frame
    // data, and manage the frame buffer with std::vector::resize as needed.
    //

    //
    // Holds the raw UDP packet header received from the sensor
    //
    std::array<std::uint8_t, PACKET_HEADER_SZ> header_buffer_;

    //
    // Our frame grabber double buffers images when acquiring from the
    // camera. This implements the "back buffer" in the double buffering
    // scheme.
    //
    std::vector<std::uint8_t> back_buffer_;

    //
    // The buffer that subscribed clients fetch data from
    //
    std::vector<std::uint8_t> front_buffer_;

    //
    // synchronization/notification structures for the `frame_`
    //
    std::mutex front_buffer_mutex_;
    std::condition_variable front_buffer_cv_;

  }; // end: class FrameGrabberUdp::Impl

} // namespace ifm3d

//============================================================
// Impl -- Implementation Details
//============================================================

//-------------------------------------
// ctor/dtor
//-------------------------------------
ifm3d::FrameGrabberUdp::Impl::Impl(int port, std::uint16_t max_packet_size)
  : cam_port_(port), io_service_(),
    max_payload_size_(max_packet_size - ifm3d::PACKET_HEADER_SZ)
{
  if (max_packet_size < ifm3d::MIN_UDP_PAYLOAD_SZ ||
      max_packet_size > ifm3d::MAX_UDP_PAYLOAD_SZ)
    {
      LOG(WARNING) << "UDP packet size out of bounds";
      throw ifm3d::error_t(IFM3D_VALUE_OUT_OF_RANGE);
    }

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

bool
ifm3d::FrameGrabberUdp::Impl::WaitForFrame(
  long timeout_millis,
  std::function<void(std::vector<std::uint8_t>&)> set_bytes)
{
  // mutex will unlock in `unique_lock` dtor if not explicitly unlocked prior
  // -- we use it here to ensure no deadlocks
  std::unique_lock<std::mutex> lock(this->front_buffer_mutex_);

  try
    {
      if (timeout_millis <= 0)
        {
          this->front_buffer_cv_.wait(lock);
        }
      else
        {
          if (this->front_buffer_cv_.wait_for(
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

  set_bytes(this->front_buffer_);

  lock.unlock();
  return true;
}

//-------------------------------------
// "Private" interface
//-------------------------------------

void
ifm3d::FrameGrabberUdp::Impl::Stop()
{
  this->io_service_.post(
      []() { throw ifm3d::error_t(IFM3D_THREAD_INTERRUPTED); });
}

//---------------------------------------
// The rest of these functions are running
// in our worker thread
//---------------------------------------

void
ifm3d::FrameGrabberUdp::Impl::DeliverFrame()
{
  // The first chunk must be the confidence image. If not, discard the frame
  // directly.
  std::size_t cidx =
    ifm3d::get_chunk_index(this->back_buffer_,
                           ifm3d::image_chunk::CONFIDENCE);
  if (cidx != 0)
    {
      LOG(WARNING) << "The first channel in frame is not confidence_image."
                   << " Discarding frame.";
      return;
    }

  this->front_buffer_mutex_.lock();
  this->back_buffer_.swap(this->front_buffer_);
  this->front_buffer_mutex_.unlock();
  this->front_buffer_cv_.notify_all();
}

void
ifm3d::FrameGrabberUdp::Impl::ResetReceiveState()
{
  this->back_buffer_.clear();
  this->expected_channel_idx_ = 0;
  this->bytes_xferd_for_channel_ = 0;
}

void
ifm3d::FrameGrabberUdp::Impl::Run()
{
  VLOG(IFM3D_TRACE) << "FrameGrabberUdp thread running...";
  boost::asio::io_service::work work(this->io_service_);

  try
    {
      this->ResetReceiveState();
      this->back_buffer_.resize(this->max_payload_size_);

      boost::array<boost::asio::mutable_buffer, 2> buffs = {
        boost::asio::buffer(this->header_buffer_),
        boost::asio::buffer(this->back_buffer_)
      };

      this->sock_->async_receive_from(
        buffs,
        this->endpoint_,
        std::bind(&ifm3d::FrameGrabberUdp::Impl::PacketHandler, this,
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

void
ifm3d::FrameGrabberUdp::Impl::PacketHandler(
    const boost::system::error_code& ec, std::size_t bytes_xferd)
{
  if (ec)
    {
      throw ifm3d::error_t(ec.value());
    }

  if (bytes_xferd >= sizeof(ifm3d::PacketHeader))
    {
      PacketHeader* header =
        reinterpret_cast<PacketHeader*>(this->header_buffer_.data());

      if (header->magic == ifm3d::PACKET_MAGIC_NUMBER)
        {
          // Start/stop are transmit on channel 0
          if (header->channel_id == 0)
            {
              if (header->total_channel_length==sizeof(ifm3d::FRAME_HEADER) &&
                  memcmp(&this->back_buffer_[this->back_buffer_.size() -
                                             this->max_payload_size_],
                         ifm3d::FRAME_HEADER,
                         sizeof(ifm3d::FRAME_HEADER)) == 0)
                {
                  // New frame is beginning -- if there's any incomplete data
                  // for the previous frame, deliver what we can.
                  if (this->back_buffer_.size() > this->max_payload_size_)
                    {
                      LOG(WARNING) << "START received with incomplete previous"
                                   << " frame. Delivering incomplete frame.";

                      // Drop the allocation for this packet and any incomplete
                      // payload information from the buffer
                      this->back_buffer_.resize(
                        this->back_buffer_.size() -
                        this->max_payload_size_ -
                        this->bytes_xferd_for_channel_);

                      // Signal we have a complete frame
                      this->DeliverFrame();
                    }

                  // Reset back buffer in anticipation of frame data
                  this->ResetReceiveState();
                  this->frame_counter_ = header->frame_counter;
                }
              else if (header->total_channel_length ==
                       sizeof(ifm3d::FRAME_FOOTER) &&
                      (memcmp(
                         &this->back_buffer_[this->back_buffer_.size() -
                                             this->max_payload_size_],
                         ifm3d::FRAME_FOOTER,
                         sizeof(ifm3d::FRAME_FOOTER)) == 0))
                {
                  // Drop the allocation for this packet and any incomplete
                  // payload information from the buffer
                  this->back_buffer_.resize(this->back_buffer_.size() -
                                            this->max_payload_size_ -
                                            this->bytes_xferd_for_channel_);

                  // Signal we have a complete frame
                  this->DeliverFrame();

                  // Reset the back buffer state
                  this->ResetReceiveState();
                }
              else
                {
                  LOG(WARNING) << "Received an unknown FRAME packet!";
                  this->ResetReceiveState();
                }
            }
          else
            {
              // Verify the frame counter (detect whether we've missed one or
              // more stop/star pairs)
              if (this->frame_counter_ == header->frame_counter)
                {
                  // Check the packet index to see if anything was dropped
                  if (header->index_of_packet_in_channel ==
                      this->expected_channel_idx_)
                    {
                      std::size_t payload_size =
                        bytes_xferd - ifm3d::PACKET_HEADER_SZ;
                      this->bytes_xferd_for_channel_ += payload_size;

                      // Previously we had reserved space for the maximum
                      // possible datagram size. Resize the buffer down to what
                      // was actually transmitted so the next packet is written
                      // continguously.
                      //
                      // If the max_payload_size was properly specified at
                      // creation, this resize will be a no-op for all but
                      // the last packet in the channel.
                      this->back_buffer_.resize(
                        this->back_buffer_.size() -
                        (this->max_payload_size_ - payload_size));

                      // Increment the expected index, wrapping as needed
                      this->expected_channel_idx_ =
                        (this->expected_channel_idx_ + 1) %
                        header->number_of_packets_in_channel;

                      // This channel is complete! Look for the start of the
                      // next channel.
                      if(this->expected_channel_idx_ == 0)
                        {
                          this->bytes_xferd_for_channel_ = 0;
                        }
                    }
                  else
                    {
                      LOG(WARNING) << "Packet was dropped or received out of "
                                   << "order. Dropping channel from frame.";

                      // Remove the space allocated for this frame, along with
                      // all of the data written thus far for the current
                      // channel.
                      this->back_buffer_.resize(
                        this->back_buffer_.size() -
                        this->max_payload_size_ -
                        this->bytes_xferd_for_channel_);

                      // Wait for the start of the next channel.
                      this->expected_channel_idx_ = 0;
                      this->bytes_xferd_for_channel_ = 0;
                    }
                }
              else
                {
                  LOG(WARNING) << "Received unexpected frame counter"
                               << " (missed a stop/star sequence)";
                  this->ResetReceiveState();
                }
            }
        }
      else
        {
          LOG(WARNING) << "Discarding packet with invalid MAGIC number.";
        }
    }

  // Resize to be current size + a full datagram
  std::size_t curr_idx = this->back_buffer_.size();
  this->back_buffer_.resize(curr_idx + this->max_payload_size_);

  boost::array<boost::asio::mutable_buffer, 2> buffs = {
    boost::asio::buffer(this->header_buffer_),
    boost::asio::buffer(&this->back_buffer_[curr_idx],
                        this->max_payload_size_)
  };

  this->sock_->async_receive_from(
    buffs,
    this->endpoint_,
    std::bind(&ifm3d::FrameGrabberUdp::Impl::PacketHandler, this,
              std::placeholders::_1, std::placeholders::_2));
}

#endif // __IFM3D_FRAME_GRABBER_UDP_FRAME_GRABBER_UDP_IMPL_H__
