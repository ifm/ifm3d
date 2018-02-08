// -*- c++ -*-
/*
 * Copyright (C) 2017 Kuhn & Völkel GmbH
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

#include <ifm3d/pcicclient.h>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <glog/logging.h>
#include <ifm3d/camera/err.h>


// Init command sequence to deactivate asynchronous result messages and
// activate asynchronous error and notification messages (command: p6)
const std::string ifm3d::PCICClient::init_command = "9999L000000008\r\n9999p6\r\n";

ifm3d::PCICClient::PCICClient(ifm3d::Camera::Ptr cam)
  : cam_(cam),
    connected_(false),
    io_service_(),
    sock_(io_service_),
    current_callback_id_(0),
    in_pre_content_buffer_(20, ' '),
    in_content_buffer_(),
    in_post_content_buffer_(2, ' '),
    out_pre_content_buffer_(20, ' '),
    out_post_content_buffer_("\r\n")
{
  try
    {
      this->cam_ip_ = this->cam_->IP();
      this->cam_port_ = std::stoi(this->cam_->DeviceParameter("PcicTcpPort"));
    }
  catch (const ifm3d::error_t& ex)
    {
      LOG(ERROR) << "Could not get IP/Port of the camera: "
                 << ex.what();
      // NOTE: GetIP() won't throw, so, the problem must be getting the PCIC
      // port. Here we assume the default. Former behavior was to throw!
      LOG(WARNING) << "Assuming default PCIC port!";
      this->cam_port_ = ifm3d::DEFAULT_PCIC_PORT;
  }

  LOG(INFO) << "Camera connection info: ip=" << this->cam_ip_
            << ", port=" << this->cam_port_;

  if (this->cam_->IsO3X())
    {
      throw ifm3d::error_t(IFM3D_PCICCLIENT_UNSUPPORTED_DEVICE);
    }

  this->endpoint_ =
    boost::asio::ip::tcp::endpoint(
      boost::asio::ip::address::from_string(this->cam_ip_), this->cam_port_);

  this->thread_ =
    std::unique_ptr<std::thread>(
     new std::thread(std::bind(&ifm3d::PCICClient::DoConnect, this)));
}

ifm3d::PCICClient::~PCICClient()
{
  DLOG(INFO) << "PCICClient dtor running...";

  if(this->thread_ && this->thread_->joinable())
    {
      // NOTE: If Stop() was already called, that is fine
      // because the ASIO event loop is already done so the posted exception
      // will never get emitted.
      this->Stop();
      this->thread_->join();
    }

  DLOG(INFO) << "PCICClient done.";
}

void
ifm3d::PCICClient::Stop()
{
  this->io_service_.post([]() {
    throw ifm3d::error_t(IFM3D_THREAD_INTERRUPTED); });
}

long
ifm3d::PCICClient::Call(const std::string& request,
			 std::function<void(const std::string& response)> callback)
{
  // TODO Better solution for this connection waiting ..
  int i = 0;
  while (! this->connected_.load())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      i++;

      if (i > 2000)
        {
          LOG(WARNING) << "connected_ flag not set!";
          return -1;
        }
    }

  // PCICClient is unbuffered, so block further calls
  std::unique_lock<std::mutex> out_mutex_lock(this->out_mutex_);

  this->out_completed_.store(false);

  // Sync access to ticket and id generation
  // as well as to ticket/id and id/callback maps
  std::unique_lock<std::mutex> data_sync_lock(this->data_sync_mutex_);

  // Get next command ticket and callback id
  int ticket = this->NextCommandTicket();
  long callback_id = this->NextCallbackId();

  // Add mappings: ticket -> callback id; callback id -> callback
  this->ticket_to_callback_id_[ticket] = callback_id;
  this->pending_callbacks_[callback_id] = callback;

  data_sync_lock.unlock();


  // Transform ticket and length to string
  std::ostringstream pre_content_ss;
  pre_content_ss << ticket << 'L' << std::setw(9) << std::setfill('0')
		 << (request.size()+6) << "\r\n" << ticket;

  // Prepare pre content buffer
  this->out_pre_content_buffer_ = pre_content_ss.str();

  DLOG(INFO) << "Client sending request";

  // Send command
  this->DoWrite(State::PRE_CONTENT, request);

  // Wait until sending is complete
  while(!this->out_completed_.load())
    {
      this->out_cv_.wait(out_mutex_lock);
    }
  out_mutex_lock.unlock();

  return callback_id;
}

std::string
ifm3d::PCICClient::Call(const std::string& request)
{
  std::atomic_bool has_result(false);
  std::string result;

  Call(request, [&](const std::string& content)
       {
	 // Copy content, notify and leave callback
	 result = content;
	 std::unique_lock<std::mutex> lock(this->in_mutex_);
	 has_result.store(true);
	 this->in_cv_.notify_all();
       });

  std::unique_lock<std::mutex> lock(this->in_mutex_);
  while(!has_result.load())
    {
      this->in_cv_.wait(lock);
    }
  lock.unlock();

  return result;
}

long
ifm3d::PCICClient
::SetErrorCallback(std::function<void(const std::string& error)> callback)
{
  std::unique_lock<std::mutex> lock(this->data_sync_mutex_);
  long callback_id = this->NextCallbackId();

  // Asynchronous error messages always have ticket '0001'
  this->ticket_to_callback_id_[1] = callback_id;
  this->pending_callbacks_[callback_id] = callback;
  lock.unlock();
  return callback_id;
}

long
ifm3d::PCICClient
::SetNotificationCallback(std::function<void(const std::string& notification)> callback)
{
  std::unique_lock<std::mutex> lock(this->data_sync_mutex_);
  long callback_id = this->NextCallbackId();

  // Asynchronous notification messages always have ticket '0010'
  this->ticket_to_callback_id_[10] = callback_id;
  this->pending_callbacks_[callback_id] = callback;
  lock.unlock();
  return callback_id;
}

void
ifm3d::PCICClient::CancelCallback(long callback_id)
{
  std::unique_lock<std::mutex> lock(this->data_sync_mutex_);
  this->pending_callbacks_.erase(callback_id);
  lock.unlock();
}

void
ifm3d::PCICClient::DoConnect()
{
  boost::asio::io_service::work work(this->io_service_);

  // Establish TCP connection to sensor
  try
    {
      this->sock_.async_connect(this->endpoint_,
				std::bind(&ifm3d::PCICClient::ConnectHandler,
					  this, std::placeholders::_1));
      this->io_service_.run();
    }
  catch(const std::exception& ex)
    {
        LOG(WARNING) << "Exception: " << ex.what();
    }

  LOG(INFO) << "PCICClient thread done.";
}

void
ifm3d::PCICClient::ConnectHandler(const boost::system::error_code& ec)
{
  if(ec) { throw ifm3d::error_t(ec.value()); }
  this->DoRead(State::PRE_CONTENT);

  // Write init command sequence to turn off asynchronous messages
  boost::asio::async_write(this->sock_,
			   boost::asio::buffer(&init_command[0],
					       init_command.size()),
			   [&, this]
			   (const boost::system::error_code& ec,
			    std::size_t bytes_transferred)
			   {
			     if (ec) { throw ifm3d::error_t(ec.value()); }
			     this->connected_.store(true);
			   });
}

void
ifm3d::PCICClient::DoRead(State state, int bytes_remaining)
{
  std::string &buffer = this->InBufferByState(state);
  if(bytes_remaining==UNSET)
    {
      bytes_remaining = buffer.size();
    }

  this->sock_.async_read_some(
			      boost::asio::buffer(&buffer[buffer.size()-bytes_remaining],
						  bytes_remaining),
			      std::bind(&ifm3d::PCICClient::ReadHandler,
					this,
					state,
					std::placeholders::_1,
					std::placeholders::_2,
					bytes_remaining));
}

void
ifm3d::PCICClient::ReadHandler(State state, const boost::system::error_code& ec,
				std::size_t bytes_transferred, std::size_t bytes_remaining)
{
  if(ec) { throw ifm3d::error_t(ec.value()); }

  if(bytes_remaining - bytes_transferred > 0)
    {
      this->DoRead(state, bytes_remaining - bytes_transferred);
    }
  else
    {
      int ticket;
      int length;
      switch(state)
	{
	case State::PRE_CONTENT:
	  length = std::stoi(this->in_pre_content_buffer_.substr(5, 9));
	  this->in_content_buffer_.resize(length-6);
	  this->DoRead(State::CONTENT);
	  break;

	case State::CONTENT:
	  this->DoRead(State::POST_CONTENT);
	  break;

	case State::POST_CONTENT:
	  ticket = std::stoi(this->in_pre_content_buffer_.substr(0, 4));
	  this->data_sync_mutex_.lock();
	  try
	    {
	      // Get callback id
	      long callback_id = this->ticket_to_callback_id_.at(ticket);

	      // Erase mapping if it is a one-time ticket triggered by a Call method
	      if(ticket >= 1000 && ticket <= 9999)
		{
		  this->ticket_to_callback_id_.erase(ticket);
		}

	      // Execute callback
	      this->pending_callbacks_.at(callback_id)(this->in_content_buffer_);

	      // Erase mapping if it is a one-time ticket triggered by a Call method
	      if(ticket >= 1000 && ticket <= 9999)
		{
		  this->pending_callbacks_.erase(callback_id);
		}
	    }
	  catch(std::out_of_range ex)
	    {
	      DLOG(INFO) << "No callback for ticket " << ticket << " found!";
	    }
	  this->data_sync_mutex_.unlock();
	  this->DoRead(State::PRE_CONTENT);
	  break;
	}
    }
}

std::string&
ifm3d::PCICClient::InBufferByState(State state)
{
  switch(state)
    {
    case State::PRE_CONTENT: return this->in_pre_content_buffer_;
    case State::CONTENT: return this->in_content_buffer_;
    case State::POST_CONTENT: return this->in_post_content_buffer_;
    }
}

void
ifm3d::PCICClient::DoWrite(State state,
			    const std::string& out_content_buffer,
			    int bytes_remaining)
{
  const std::string &buffer = this->OutBufferByState(state, out_content_buffer);
  if(bytes_remaining==UNSET)
    {
      bytes_remaining = buffer.size();
    }

  this->sock_.async_write_some(
			      boost::asio::buffer(&buffer[buffer.size()
							  -bytes_remaining],
						  bytes_remaining),
			      std::bind(&ifm3d::PCICClient::WriteHandler,
					this,
					state,
					std::placeholders::_1,
					std::placeholders::_2,
					out_content_buffer,
					bytes_remaining));
}

void
ifm3d::PCICClient::WriteHandler(State state,
				 const boost::system::error_code& ec,
				 std::size_t bytes_transferred,
				 const std::string& out_content_buffer,
				 std::size_t bytes_remaining)
{
  if(ec) { throw ifm3d::error_t(ec.value()); }

  if(bytes_remaining - bytes_transferred > 0)
    {
      this->DoWrite(state, out_content_buffer,
		    bytes_remaining - bytes_transferred);
    }
  else
    {
      switch(state)
	{
	case State::PRE_CONTENT:
	  this->DoWrite(State::CONTENT, out_content_buffer);
	  break;

	case State::CONTENT:
	  this->DoWrite(State::POST_CONTENT, out_content_buffer);
	  break;

	case State::POST_CONTENT:
	  std::unique_lock<std::mutex> lock(this->out_mutex_);
	  this->out_completed_.store(true);
	  this->out_cv_.notify_all();
	  break;
	}
    }
}

const std::string&
ifm3d::PCICClient::OutBufferByState(State state,
				     const std::string& out_content_buffer)
{
  switch(state)
    {
    case State::PRE_CONTENT: return this->out_pre_content_buffer_;
    case State::CONTENT: return out_content_buffer;
    case State::POST_CONTENT: return this->out_post_content_buffer_;
    }
}

int
ifm3d::PCICClient::NextCommandTicket()
{
  int ticket = 1000;
  if(!this->ticket_to_callback_id_.empty())
    {
      ticket = (this->ticket_to_callback_id_.rbegin()->first)-1000;

      // Ignore error/notification message tickets when generating
      // new command tickets
      if(ticket<0) { ticket = 0; }

      while(this->ticket_to_callback_id_.find(((++ticket)%9000) + 1000)
	    != this->ticket_to_callback_id_.end());
      ticket = (ticket%9000) + 1000;
    }
  return ticket;
}

long
ifm3d::PCICClient::NextCallbackId()
{
  // In case of long overflow, reset to 1
  if(++this->current_callback_id_ <= 0)
    {
      this->current_callback_id_ = 1;
    }

  return this->current_callback_id_;
}
