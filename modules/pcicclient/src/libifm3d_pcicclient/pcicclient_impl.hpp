// -*- c++ -*-
/*
 * Copyright (C) 2018 ifm electronics, gmbh
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

#ifndef __IFM3D_PCICCLIENT_PCICCLIENT_IMPL_H__
#define __IFM3D_PCICCLIENT_PCICCLIENT_IMPL_H__

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

namespace ifm3d
{
	const int ONE_TIME_TICKET_LOWER_RANGE  = 1000;
	const int ONE_TIME_TICKET_HIGHER_RANGE = 9999;
	const int CONNECTED_FLAG_TIMEOUT = 2000;
	const int PRE_CONTENT_BUFFER_LENGTH = 20;
	const int POST_CONTENT_BUFFER_LENGTH = 2;


  //============================================================
  // Impl interface
  //============================================================
  class PCICClient::Impl
  {
  public:
    Impl(ifm3d::Camera::Ptr cam);
    ~Impl();

	/**
	* Interrupts the running thread by throwing an ifm3d::error_t with code
	* IFM3D_THREAD_INTERRUPTED.
	*
	* While this is (currently) part of the public interface, clients should
	* be aware that there is really no way to restart a stopped PCICClient
	* instance. To do that, you would need to instantiate a new PCICClient.
	*/
	void Stop();

	/**
	* Sends a PCIC command to the camera and returns the response
	* asynchronously through a callback (, which is automatically
	* removed internally after the callback returns).
	*
	* Note: Since the PCICClient is unbuffered, the calling thread
	* will be blocked while the request is not completely sent. Also,
	* the receiving thread will be blocked while the response callback
	* has not returned.
	*
	* @param[in] request String containing the plain command
	* (without any header information, like ticket, length, etc.)
	*
	* @param[in] callback Function, called after receiving the response
	* from the camera, providing the plain response data as string
	* (without any header information, like ticket, length, etc.)
	*
	* @return Callback id, which can be used to cancel this Call before
	* receiving the response.
	*/
	long Call(const std::string& request,
		std::function<void(const std::string& response)> callback);

	/**
	* Sends a PCIC command to the camera and returns the response
	* as soon as it has been received. In the meanwhile, this call
	* is blocked.
	*
	* @param[in] request String containing the plain command
	* (without any header infomration, like ticket, length, etc.)
	*
	* @return Copy of received plain response data as string
	* (without any header information, like ticket, length, etc.)
	*
	* NOTE: This Call can block and hang indefinitely depending upon PCIC response.
	*/
	std::string Call(const std::string& request);

	/**
	* Similar to the Call function above.
	*
	* Sends a PCIC command to the camera and returns the response
	* as soon as it has been received. In the meanwhile, this call
	* is blocked.
	*
	* @param[in] request String containing the plain command
	* (without any header infomration, like ticket, length, etc.)
	*
	* @param[in] response String containing the response from the camera
	*  providing the plain response data as string
	* (without any header information, like ticket, length, etc.)
	*
	* @param[in] timeout in milliseconds, in case, the PCIC fails to come
	* through.
	*
	* NOTE: This Call can fail with no response if supplied with an
	* unsuitable "timeout_millis" value. Providing timeout_millis value as 0
	* results in behaviour similar to the above Call method.
	*
	* @return true if Call succeeded, false if failed.
	*/
	bool Call(const std::string& request, std::string &response, long timeout_millis);

	/**
	* Sets the specified callback for receiving asynchronous error messages
	* until it is replaced by a new callback or canceled via
	* @see CancelCallback.
	*
	* Note: Since the PCICClient is unbuffered, the receiving thread will be
	* blocked while the error callback has not returned.
	*
	* @param[in] callback Function, called after receiving an error message
	* from camera (without any header information, like ticket, length, etc.)
	*
	* @return Callback id, which can be used to cancel receiving errors.
	*/
	long SetErrorCallback(
		std::function<void(const std::string& error)> callback);

	/**
	* Sets the specified callback for receiving asynchronous notification
	* messages until it is replaced by a new callback or canceled via
	* @see CancelCallback.
	*
	* Note: Since the PCICClient is unbuffered, the receiving thread will be
	* blocked while the notification callback has not returned.
	*
	* @param[in] callback Function, called after receiving an notification
	*            message from camera (without any header information, like
	*            ticket, length, etc.)
	*
	* @return Callback id, which can be used to cancel receiving notifications.
	*/
	long SetNotificationCallback(
		std::function<void(const std::string& notification)> callback);

	/**
	* Cancels registered callbacks. Must be called in case references/pointers
	* provided through callbacks get invalid. If callback id isn't present
	* internally anymore, i.e. if callback was replaced, already canceled or
	* automatically removed (in case of the Call method), it is simply ignored.
	*
	* @param[in] callback_id Callback id, returned by methods which take a
	*                        callback as parameter.
	*/
	void CancelCallback(long callback_id);

  private:
	  /**
	  * Commands consist of content data surrounded by some meta data.
	  * The State enum provides information which buffer is currently
	  * used in writing to and reading from network
	  */
	  enum class State { PRE_CONTENT, CONTENT, POST_CONTENT };

	  /**
	  * Connects to the camera
	  */
	  void DoConnect();

	  /**
	  * Handles DoConnect results
	  */
	  void ConnectHandler(const boost::system::error_code& ec);

	  /**
	  * Reads data from network into one of the three "in" buffers
	  * depending on current reading state.
	  */
	  void DoRead(State state, std::size_t bytes_remaining = UNSET);

	  /**
	  * Handles DoRead results: Triggers further reads and in
	  * case an incoming message is completely received, does
	  * the callback (if existent).
	  */
	  void ReadHandler(State state, const boost::system::error_code& ec,
		  std::size_t bytes_transferred,
		  std::size_t bytes_remaining);

	  /**
	  * Returns buffer to be filled from network depending on
	  * specified reading state
	  */
	  std::string& InBufferByState(State state);

	  /**
	  * Writes data to network from one of the three "out" buffers
	  * depending on current writing state.
	  */
	  void DoWrite(State state,
		  const std::string &out_content_buffer,
		  std::size_t bytes_remaining = UNSET);

	  /**
	  * Handles DoWrite results: Triggers further writes and in
	  * case a request is completely sent, unblocks calling thread.
	  */
	  void WriteHandler(State state,
		  const boost::system::error_code& ec,
		  std::size_t bytes_transferred,
		  const std::string& out_content_buffer,
		  std::size_t bytes_remaining);

	  /**
	  * Returns buffer containing data to be written to network
	  * depending on specified writing state. (In case of state CONTENT,
	  * the specified out_content_buffer is returned.)
	  */
	  const std::string& OutBufferByState(State state,
		  const std::string& out_content_buffer);

	  /**
	  * Finds and returns the next free ticket for a command
	  */
	  int NextCommandTicket();

	  /**
	  * Calculates and returns next callback id
	  */
	  long NextCallbackId();

  private:
  /**
     * Init command sequence
     */
    static const std::string init_command;

    /**
     * Shared pointer to the camera this PCIC client will communicate with.
     */
    ifm3d::Camera::Ptr cam_;

    /**
     * Cached copy of the camera IP address
     */
    std::string cam_ip_;

    /**
     * Cached copy of the camera PCIC TCP port
     */
    int cam_port_;

    /**
     * Flag indicating that client is connected.
     */
    std::atomic_bool connected_;

    /**
     * Flag which is used as default parameter in DoRead and DoWrite
     * in order to indicate that a new buffer is used for read/write
     * so that the remaining size can be initialized by buffers size.
     */
    static const int UNSET = -1;

    /**
     * The ASIO event loop handle
     */
    boost::asio::io_service io_service_;

    /**
     * The ASIO socket to PCIC
     */
    boost::asio::ip::tcp::socket sock_;

    /**
     * The ASIO endpoint to PCIC
     */
    boost::asio::ip::tcp::endpoint endpoint_;

    /**
     * A pointer to the wrapped thread object. This is the thread that
     * communicates directly with the sensor.
     */
    std::unique_ptr<std::thread> thread_;

    /**
     * Sequential id for callbacks. Used in two-stage mapping:
     * 1) ticket to callback id
     * 2) callback id to callback
     * Using callback ids for cancelling callbacks instead of tickets eliminates
     * the risk of cancelling a newer callback with the same ticket.
     */
    unsigned long current_callback_id_;

    /**
     * Maps PCIC tickets to callback ids. When receiving an incoming message,
     * the accordant callback id and thus the pending callback can be found via
     * pending_callbacks_
     */
    std::map<int, long> ticket_to_callback_id_;

    /**
     * Maps callback ids to callbacks. When receiving an incoming message,
     * the accordant callback can be found (and triggered).
     */
    std::map<long, std::function<void(const std::string& content)>>
      pending_callbacks_;

    /**
     * Pre-content buffer for incoming messages (<ticket><length>\r\n<ticket>)
     */
    std::string in_pre_content_buffer_;

    /**
     * Content buffer for incoming messages, which is provided
     * through the callback to the caller
     */
    std::string in_content_buffer_;

    /**
     * Post-content buffer for incoming messages (\r\n)
     */
    std::string in_post_content_buffer_;

    /**
     * Pre-content buffer for outgoing requests (<ticket><length>\r\n<ticket>)
     */
    std::string out_pre_content_buffer_;

    /**
     * Post-content buffer for outgoing messages (\r\n)
     */
    std::string out_post_content_buffer_;

    /**
     * Flag that indicates whether an incoming messages is completely read.
     */
    std::atomic_bool out_completed_;

    /**
     * Ensures synchronized access to ticket and id generation
     * as well as ticket/id and id/callback maps
     */
    std::mutex data_sync_mutex_;

    /**
     * Ensures single outgoing message
     */
    std::mutex out_mutex_;

    /**
     * Condition variable used to unblock call
     */
    std::condition_variable out_cv_;

    /**
     * Ensures incoming response (used in synchronous Call)
     */
    std::mutex in_mutex_;

    /**
     * Condition variable used to unblock synchronous Call
     */
    std::condition_variable in_cv_;

  }; // end: class FrameGrabber::Impl

} // end: namespace ifm3d

//============================================================
// Impl -- Implementation Details
//============================================================

// Init command sequence to deactivate asynchronous result messages and
// activate asynchronous error and notification messages (command: p6)
const std::string ifm3d::PCICClient::Impl::init_command = "9999L000000008\r\n9999p6\r\n";

//-------------------------------------
// ctor/dtor
//-------------------------------------
ifm3d::PCICClient::Impl::Impl(ifm3d::Camera::Ptr cam)
	: cam_(cam),
	connected_(false),
	io_service_(),
	sock_(io_service_),
	current_callback_id_(0),
	in_pre_content_buffer_(ifm3d::PRE_CONTENT_BUFFER_LENGTH, ' '),
	in_content_buffer_(),
	in_post_content_buffer_(ifm3d::POST_CONTENT_BUFFER_LENGTH, ' '),
	out_pre_content_buffer_(ifm3d::PRE_CONTENT_BUFFER_LENGTH, ' '),
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
			new std::thread(std::bind(&ifm3d::PCICClient::Impl::DoConnect, this)));
}

ifm3d::PCICClient::Impl::~Impl()
{
  VLOG(IFM3D_TRACE) << "FrameGrabber dtor running...";

  if (this->thread_ && this->thread_->joinable())
    {
      this->Stop();
      this->thread_->join();
    }

  VLOG(IFM3D_TRACE) << "FrameGrabber destroyed.";
}

void
ifm3d::PCICClient::Impl::Stop()
{
	this->io_service_.post([]() {
		throw ifm3d::error_t(IFM3D_THREAD_INTERRUPTED); });
}

long
ifm3d::PCICClient::Impl::Call(const std::string& request,
	std::function<void(const std::string& response)> callback)
{
	// TODO Better solution for this connection waiting ..
	int i = 0;
	while (!this->connected_.load())
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		i++;

		if (i > ifm3d::CONNECTED_FLAG_TIMEOUT)
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
		<< (request.size() + 6) << "\r\n" << ticket;

	// Prepare pre content buffer
	this->out_pre_content_buffer_ = pre_content_ss.str();

	DLOG(INFO) << "Client sending request";

	// Send command
	this->DoWrite(State::PRE_CONTENT, request);

	// Wait until sending is complete
	while (!this->out_completed_.load())
	{
		this->out_cv_.wait(out_mutex_lock);
	}
	out_mutex_lock.unlock();

	return callback_id;
}

std::string
ifm3d::PCICClient::Impl::Call(const std::string& request)
{
  std::string response;
  Call(request, response, 0);
  return response;
}

bool/* @R Consider boost::asio::error return type here */
ifm3d::PCICClient::Impl::Call(const std::string& request, std::string& response, long timeout_millis)
{
  std::atomic_bool has_result(false);
  long call_output = -1;

  // Handle the PCIC Call

  std::unique_ptr<std::thread> call_thread_ = std::make_unique<std::thread>([&]{
  call_output = Call(request, [&](const std::string& content)
  {
    // Copy content, notify and leave callback
    response = content;
    std::unique_lock<std::mutex> lock(this->in_mutex_);
    has_result.store(true);
    this->in_cv_.notify_all();

  });

  });

  if(call_thread_ && call_thread_->joinable()) call_thread_->join();
  
  // Check the return value of our PCIC Call
  if(call_output > 0)
  {
    std::unique_lock<std::mutex> lock(this->in_mutex_);
    try
    {
      if (timeout_millis <= 0)
      {
        this->in_cv_.wait(lock, [&]{return has_result.load();});
      }

      else
      {
        if (this->in_cv_.wait_for(
              lock, std::chrono::milliseconds(timeout_millis)) ==
            std::cv_status::timeout)
        {
          this->in_cv_.notify_all();
          if(this->thread_ && this->thread_->joinable())
          {
          	this->Stop();
          	this->thread_->join();
          }
          return has_result.load();
        }

        else
        {
          this->in_cv_.wait(lock, [&]{return has_result.load();});
        }
      }
    }

    catch (const std::system_error& ex)
    {
      LOG(WARNING) << "PCICClient::Call: " << ex.what();
      return has_result.load();
    }
  }

  return has_result.load();
}

long
ifm3d::PCICClient
::Impl::SetErrorCallback(std::function<void(const std::string& error)> callback)
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
::Impl::SetNotificationCallback(std::function<void(const std::string& notification)> callback)
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
ifm3d::PCICClient::Impl::CancelCallback(long callback_id)
{
	std::unique_lock<std::mutex> lock(this->data_sync_mutex_);
	this->pending_callbacks_.erase(callback_id);
	lock.unlock();
}

void
ifm3d::PCICClient::Impl::DoConnect()
{
  boost::asio::io_service::work work(this->io_service_);

  // Establish TCP connection to sensor
  try
    {
      this->sock_.async_connect(this->endpoint_,
				std::bind(&ifm3d::PCICClient::Impl::ConnectHandler,
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
ifm3d::PCICClient::Impl::ConnectHandler(const boost::system::error_code& ec)
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
ifm3d::PCICClient::Impl::DoRead(State state, std::size_t bytes_remaining)
{
  std::string &buffer = this->InBufferByState(state);
  if(bytes_remaining==UNSET)
    {
      bytes_remaining = buffer.size();
    }

  this->sock_.async_read_some(
			      boost::asio::buffer(&buffer[buffer.size()-bytes_remaining],
						  bytes_remaining),
			      std::bind(&ifm3d::PCICClient::Impl::ReadHandler,
					this,
					state,
					std::placeholders::_1,
					std::placeholders::_2,
					bytes_remaining));
}

void
ifm3d::PCICClient::Impl::ReadHandler(State state, const boost::system::error_code& ec,
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
	      if(ticket >= ifm3d::ONE_TIME_TICKET_LOWER_RANGE && ticket <= ifm3d::ONE_TIME_TICKET_HIGHER_RANGE)
		  {
			this->ticket_to_callback_id_.erase(ticket);
		  }

	      // Execute callback
	      this->pending_callbacks_.at(callback_id)(this->in_content_buffer_);

	      // Erase mapping if it is a one-time ticket triggered by a Call method
		  if (ticket >= ifm3d::ONE_TIME_TICKET_LOWER_RANGE && ticket <= ifm3d::ONE_TIME_TICKET_HIGHER_RANGE)
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
ifm3d::PCICClient::Impl::InBufferByState(State state)
{
  switch(state)
    {
    case State::PRE_CONTENT: return this->in_pre_content_buffer_;
    case State::CONTENT: return this->in_content_buffer_;
    case State::POST_CONTENT: return this->in_post_content_buffer_;
    }
  throw;
}

void
ifm3d::PCICClient::Impl::DoWrite(State state,
			    const std::string& out_content_buffer,
			    std::size_t bytes_remaining)
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
			      std::bind(&ifm3d::PCICClient::Impl::WriteHandler,
					this,
					state,
					std::placeholders::_1,
					std::placeholders::_2,
					out_content_buffer,
					bytes_remaining));
}

void
ifm3d::PCICClient::Impl::WriteHandler(State state,
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
ifm3d::PCICClient::Impl::OutBufferByState(State state,
				     const std::string& out_content_buffer)
{
  switch(state)
    {
    case State::PRE_CONTENT: return this->out_pre_content_buffer_;
    case State::CONTENT: return out_content_buffer;
    case State::POST_CONTENT: return this->out_post_content_buffer_;
    }
  throw;
}

int
ifm3d::PCICClient::Impl::NextCommandTicket()
{
  const auto number_of_one_time_tickets = ONE_TIME_TICKET_HIGHER_RANGE - ONE_TIME_TICKET_LOWER_RANGE + 1;
  int ticket = ifm3d::ONE_TIME_TICKET_LOWER_RANGE;
  if(!this->ticket_to_callback_id_.empty())
    {
      ticket = (this->ticket_to_callback_id_.rbegin()->first)- ifm3d::ONE_TIME_TICKET_LOWER_RANGE;

      // Ignore error/notification message tickets when generating
      // new command tickets
      if(ticket<0) { ticket = 0; }

      while(this->ticket_to_callback_id_.find(((++ticket)% number_of_one_time_tickets) + ifm3d::ONE_TIME_TICKET_LOWER_RANGE)
	    != this->ticket_to_callback_id_.end());
      ticket = (ticket% number_of_one_time_tickets) + ifm3d::ONE_TIME_TICKET_LOWER_RANGE;
    }
  return ticket;
}

long
ifm3d::PCICClient::Impl::NextCallbackId()
{
  return (this->current_callback_id_ == 0)? (this->current_callback_id_ = 1) : (++this->current_callback_id_) ;
}


#endif // __IFM3D_PCICCLIENT_PCICCLIENT_IMPL_H__
