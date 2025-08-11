// -*- c++ -*-
/*
 * Copyright 2018 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PCICCLIENT_PCICCLIENT_IMPL_H
#define IFM3D_PCICCLIENT_PCICCLIENT_IMPL_H
#pragma once

#include <algorithm>
#include <asio.hpp>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <functional>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/device.h>
#include <ifm3d/device/err.h>
#include <ifm3d/pcicclient/pcicclient.h>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>

namespace ifm3d
{
  const int ONE_TIME_TICKET_LOWER_RANGE = 1000;
  const int ONE_TIME_TICKET_HIGHER_RANGE = 9999;
  const int CONNECTED_FLAG_TIMEOUT = 2000;
  const int PRE_CONTENT_BUFFER_LENGTH = 20;
  const int POST_CONTENT_BUFFER_LENGTH = 2;

  //============================================================
  // Impl interface
  //============================================================
  class IFM3D_NO_EXPORT PCICClient::Impl
  {
  public:
    Impl(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl& operator=(Impl&&) = delete;
    Impl(ifm3d::LegacyDevice::Ptr cam, const std::uint16_t& pcic_port);
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
     * NOTE: This Call can block and hang indefinitely depending upon PCIC
     * response.
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
    bool Call(const std::string& request,
              std::string& response,
              long timeout_millis);

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
     * @return Callback id, which can be used to cancel receiving
     * notifications.
     */
    long SetNotificationCallback(
      std::function<void(const std::string& notification)> callback);

    /**
     * Cancels registered callbacks. Must be called in case references/pointers
     * provided through callbacks get invalid. If callback id isn't present
     * internally anymore, i.e. if callback was replaced, already canceled or
     * automatically removed (in case of the Call method), it is simply
     * ignored.
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
    enum class State : std::uint8_t
    {
      PRE_CONTENT,
      CONTENT,
      POST_CONTENT
    };

    /**
     * Connects to the camera
     */
    void do_connect();

    /**
     * Handles DoConnect results
     */
    void connect_handler(const asio::error_code& ec);

    /**
     * Reads data from network into one of the three "in" buffers
     * depending on current reading state.
     */
    void do_read(State state, std::size_t bytes_remaining = UNSET);

    /**
     * Handles DoRead results: Triggers further reads and in
     * case an incoming message is completely received, does
     * the callback (if existent).
     */
    void read_handler(State state,
                      const asio::error_code& ec,
                      std::size_t bytes_transferred,
                      std::size_t bytes_remaining);

    /**
     * Returns buffer to be filled from network depending on
     * specified reading state
     */
    std::string& in_buffer_by_state(State state);

    /**
     * Writes data to network from one of the three "out" buffers
     * depending on current writing state.
     */
    void do_write(State state,
                  const std::string& out_content_buffer,
                  std::size_t bytes_remaining = UNSET);

    /**
     * Handles DoWrite results: Triggers further writes and in
     * case a request is completely sent, unblocks calling thread.
     */
    void write_handler(State state,
                       const asio::error_code& ec,
                       std::size_t bytes_transferred,
                       const std::string& out_content_buffer,
                       std::size_t bytes_remaining);

    /**
     * Returns buffer containing data to be written to network
     * depending on specified writing state. (In case of state CONTENT,
     * the specified out_content_buffer is returned.)
     */
    const std::string& out_buffer_by_state(
      State state,
      const std::string& out_content_buffer);

    /**
     * Finds and returns the next free ticket for a command
     */
    int next_command_ticket();

    /**
     * Calculates and returns next callback id
     */
    long next_callback_id();

  private:
    /**
     * Init command sequence
     */
    static const std::string INIT_COMMAND;

    /**
     * Shared pointer to the camera this PCIC client will communicate with.
     */
    ifm3d::LegacyDevice::Ptr _cam;

    /**
     * Cached copy of the camera IP address
     */
    std::string _cam_ip;

    /**
     * Cached copy of the camera PCIC TCP port
     */
    int _cam_port;

    /**
     * Flag indicating that client is connected.
     */
    std::atomic_bool _connected;

    /**
     * Flag which is used as default parameter in DoRead and DoWrite
     * in order to indicate that a new buffer is used for read/write
     * so that the remaining size can be initialized by buffers size.
     */
    static const int UNSET = -1;

    /**
     * The ASIO event loop handle
     */
    asio::io_service _io_service;

    /**
     * The ASIO socket to PCIC
     */
    asio::ip::tcp::socket _sock;

    /**
     * The ASIO endpoint to PCIC
     */
    asio::ip::tcp::endpoint _endpoint;

    /**
     * A pointer to the wrapped thread object. This is the thread that
     * communicates directly with the sensor.
     */
    std::unique_ptr<std::thread> _thread;

    /**
     * Sequential id for callbacks. Used in two-stage mapping:
     * 1) ticket to callback id
     * 2) callback id to callback
     * Using callback ids for cancelling callbacks instead of tickets
     * eliminates the risk of cancelling a newer callback with the same ticket.
     */
    unsigned long _current_callback_id{};

    /**
     * Maps PCIC tickets to callback ids. When receiving an incoming message,
     * the accordant callback id and thus the pending callback can be found via
     * pending_callbacks_
     */
    std::map<int, long> _ticket_to_callback_id;

    /**
     * Maps callback ids to callbacks. When receiving an incoming message,
     * the accordant callback can be found (and triggered).
     */
    std::map<long, std::function<void(const std::string& content)>>
      _pending_callbacks;

    /**
     * Pre-content buffer for incoming messages (<ticket><length>\r\n<ticket>)
     */
    std::string _in_pre_content_buffer;

    /**
     * Content buffer for incoming messages, which is provided
     * through the callback to the caller
     */
    std::string _in_content_buffer;

    /**
     * Post-content buffer for incoming messages (\r\n)
     */
    std::string _in_post_content_buffer;

    /**
     * Pre-content buffer for outgoing requests (<ticket><length>\r\n<ticket>)
     */
    std::string _out_pre_content_buffer;

    /**
     * Post-content buffer for outgoing messages (\r\n)
     */
    std::string _out_post_content_buffer;

    /**
     * Flag that indicates whether an incoming messages is completely read.
     */
    std::atomic_bool _out_completed{};

    /**
     * Ensures synchronized access to ticket and id generation
     * as well as ticket/id and id/callback maps
     */
    std::mutex _data_sync_mutex;

    /**
     * Ensures single outgoing message at a time
     */
    std::mutex _call_mutex;

    /**
     * Ensures outgoing message
     */
    std::mutex _out_mutex;

    /**
     * Condition variable used to unblock call
     */
    std::condition_variable _out_cv;

    /**
     * Ensures incoming response (used in synchronous Call)
     */
    std::mutex _in_mutex;

    /**
     * Condition variable used to unblock synchronous Call
     */
    std::condition_variable _in_cv;

  }; // end: class FrameGrabber::Impl

} // end: namespace ifm3d

//============================================================
// Impl -- Implementation Details
//============================================================

// Init command sequence to deactivate asynchronous result messages and
// activate asynchronous error and notification messages (command: p6)
const std::string ifm3d::PCICClient::Impl::
  INIT_COMMAND = // NOLINT(misc-definitions-in-headers)
  "9999L000000008\r\n9999p6\r\n";

//-------------------------------------
// ctor/dtor
//-------------------------------------
inline ifm3d::PCICClient::Impl::Impl(ifm3d::LegacyDevice::Ptr cam,
                                     const std::uint16_t& pcic_port)
  : _cam(std::move(cam)),
    _cam_port(pcic_port == ifm3d::PCIC_PORT ? ifm3d::DEFAULT_PCIC_PORT :
                                              pcic_port),
    _connected(false),
    _sock(_io_service),
    _in_pre_content_buffer(ifm3d::PRE_CONTENT_BUFFER_LENGTH, ' '),
    _in_post_content_buffer(ifm3d::POST_CONTENT_BUFFER_LENGTH, ' '),
    _out_pre_content_buffer(ifm3d::PRE_CONTENT_BUFFER_LENGTH, ' '),
    _out_post_content_buffer("\r\n")
{
  try
    {
      this->_cam_ip = this->_cam->IP();
      if (_cam_port != pcic_port)
        {
          this->_cam_port =
            std::stoi(this->_cam->DeviceParameter("PcicTcpPort"));
        }
    }
  catch (const ifm3d::Error& ex)
    {
      LOG_ERROR("Could not get IP/Port of the camera: {}", ex.what());
      // NOTE: GetIP() won't throw, so, the problem must be getting the PCIC
      // port. Here we assume the default. Former behavior was to throw!
      LOG_WARNING("Assuming default PCIC port!");
      this->_cam_port = ifm3d::DEFAULT_PCIC_PORT;
    }

  LOG_INFO("Camera connection info: ip={} , port={}",
           this->_cam_ip,
           this->_cam_port);

  if (this->_cam->AmI(Device::DeviceFamily::O3X))
    {
      throw ifm3d::Error(IFM3D_PCICCLIENT_UNSUPPORTED_DEVICE);
    }

  this->_endpoint =
    asio::ip::tcp::endpoint(asio::ip::address::from_string(this->_cam_ip),
                            this->_cam_port);

  this->_thread = std::make_unique<std::thread>([this] { do_connect(); });
}

inline ifm3d::PCICClient::Impl::~Impl()
{
  LOG_VERBOSE("FrameGrabber dtor running...");

  if (this->_thread && this->_thread->joinable())
    {
      this->Stop();
      this->_thread->join();
    }

  LOG_VERBOSE("FrameGrabber destroyed.");
}

inline void
ifm3d::PCICClient::Impl::Stop()
{
  this->_io_service.post(
    []() { throw ifm3d::Error(IFM3D_THREAD_INTERRUPTED); });
}

inline long
ifm3d::PCICClient::Impl::Call(
  const std::string& request,
  std::function<void(const std::string& response)> callback)
{
  // TODO Better solution for this connection waiting ..
  int i = 0;
  while (!this->_connected.load())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      i++;

      if (i > ifm3d::CONNECTED_FLAG_TIMEOUT)
        {
          LOG_WARNING("connected_ flag not set!");
          return -1;
        }
    }

  // PCICClient is unbuffered, so block further calls
  std::lock_guard<std::mutex> call_mutex_lock(this->_call_mutex);

  this->_out_completed.store(false);

  int ticket = 0;
  long callback_id = 0;

  // Sync access to ticket and id generation
  // as well as to ticket/id and id/callback maps
  {
    std::lock_guard<std::mutex> data_sync_lock(this->_data_sync_mutex);

    // Get next command ticket and callback id
    ticket = this->next_command_ticket();
    callback_id = this->next_callback_id();

    // Add mappings: ticket -> callback id; callback id -> callback
    this->_ticket_to_callback_id[ticket] = callback_id;
    this->_pending_callbacks[callback_id] = std::move(callback);
  }

  // Transform ticket and length to string
  std::ostringstream pre_content_ss;
  pre_content_ss << ticket << 'L' << std::setw(9) << std::setfill('0')
                 << (request.size() + 6) << "\r\n"
                 << ticket;

  // Prepare pre content buffer
  this->_out_pre_content_buffer = pre_content_ss.str();

  LOG_DEBUG("Client sending request");

  // Send command
  this->do_write(State::PRE_CONTENT, request);

  // Wait until sending is complete
  std::unique_lock<std::mutex> out_mutex_lock(this->_out_mutex);
  this->_out_cv.wait(out_mutex_lock,
                     [this] { return this->_out_completed.load(); });

  return callback_id;
}

inline std::string
ifm3d::PCICClient::Impl::Call(const std::string& request)
{
  std::string response;
  Call(request, response, 0);
  return response;
}

inline bool /* @R Consider asio::error return type here */
ifm3d::PCICClient::Impl::Call(const std::string& request,
                              std::string& response,
                              long timeout_millis)
{
  std::atomic_bool has_result(false);
  long call_output = -1;

  // Handle the PCIC Call

  std::unique_ptr<std::thread> call_thread =
    std::make_unique<std::thread>([&] {
      call_output = Call(request, [&](const std::string& content) {
        // Copy content, notify and leave callback
        response = content;
        std::lock_guard<std::mutex> lock(this->_in_mutex);
        has_result.store(true);
        this->_in_cv.notify_all();
      });
    });

  if (call_thread && call_thread->joinable())
    {
      call_thread->join();
    }

  // Check the return value of our PCIC Call
  auto predicate = [&has_result] { return has_result.load(); };
  if (call_output > 0)
    {
      std::unique_lock<std::mutex> lock(this->_in_mutex);
      try
        {
          if (timeout_millis <= 0)
            {
              this->_in_cv.wait(lock, predicate);
            }

          else
            {
              if (!this->_in_cv.wait_for(
                    lock,
                    std::chrono::milliseconds(timeout_millis),
                    predicate))
                {
                  this->_in_cv.notify_all();
                  if (this->_thread && this->_thread->joinable())
                    {
                      LOG_WARNING("PCICClient::Call: Timed out waiting for a "
                                  "response, stopping thread...");
                      this->Stop();
                      this->_thread->join();
                    }
                  return false;
                }
            }
        }

      catch (const std::system_error& ex)
        {
          LOG_WARNING("PCICClient::Call: {}", ex.what());
          return has_result.load();
        }
    }

  return has_result.load();
}

inline long
ifm3d::PCICClient ::Impl::SetErrorCallback(
  std::function<void(const std::string& error)> callback)
{
  std::lock_guard<std::mutex> lock(this->_data_sync_mutex);
  long callback_id = this->next_callback_id();

  // Asynchronous error messages always have ticket '0001'
  this->_ticket_to_callback_id[1] = callback_id;
  this->_pending_callbacks[callback_id] = std::move(callback);
  return callback_id;
}

inline long
ifm3d::PCICClient ::Impl::SetNotificationCallback(
  std::function<void(const std::string& notification)> callback)
{
  std::lock_guard<std::mutex> lock(this->_data_sync_mutex);
  long callback_id = this->next_callback_id();

  // Asynchronous notification messages always have ticket '0010'
  this->_ticket_to_callback_id[10] = callback_id;
  this->_pending_callbacks[callback_id] = std::move(callback);
  return callback_id;
}

inline void
ifm3d::PCICClient::Impl::CancelCallback(long callback_id)
{
  std::lock_guard<std::mutex> lock(this->_data_sync_mutex);
  this->_pending_callbacks.erase(callback_id);
}

inline void
ifm3d::PCICClient::Impl::do_connect()
{
  asio::io_service::work work(this->_io_service);

  // Establish TCP connection to sensor
  try
    {
      this->_sock.async_connect(this->_endpoint, [this](auto&& error_code) {
        connect_handler(std::forward<decltype(error_code)>(error_code));
      });
      this->_io_service.run();
    }
  catch (const std::exception& ex)
    {
      LOG_WARNING("Exception: {}", ex.what());
    }

  LOG_INFO("PCICClient thread done.");
}

inline void
ifm3d::PCICClient::Impl::connect_handler(const asio::error_code& ec)
{
  if (ec)
    {
      throw ifm3d::Error(ec.value());
    }
  this->do_read(State::PRE_CONTENT);

  // Write init command sequence to turn off asynchronous messages
  asio::async_write(
    this->_sock,
    asio::buffer(INIT_COMMAND.data(), INIT_COMMAND.size()),
    [&, this](const asio::error_code& ec, std::size_t /*bytes_transferred*/) {
      if (ec)
        {
          throw ifm3d::Error(ec.value());
        }
      this->_connected.store(true);
    });
}

inline void
ifm3d::PCICClient::Impl::do_read(ifm3d::PCICClient::Impl::State state,
                                 std::size_t bytes_remaining)
{
  std::string& buffer = this->in_buffer_by_state(state);
  if (bytes_remaining == UNSET)
    {
      bytes_remaining = buffer.size();
    }

  this->_sock.async_read_some(
    asio::buffer(&buffer[buffer.size() - bytes_remaining], bytes_remaining),
    [this, state, bytes_remaining](auto&& error_code,
                                   auto&& bytes_transferred) {
      read_handler(
        state,
        std::forward<decltype(error_code)>(error_code),
        std::forward<decltype(bytes_transferred)>(bytes_transferred),
        bytes_remaining);
    });
}

inline void
ifm3d::PCICClient::Impl::read_handler(State state,
                                      const asio::error_code& ec,
                                      std::size_t bytes_transferred,
                                      std::size_t bytes_remaining)
{
  if (ec)
    {
      throw ifm3d::Error(ec.value());
    }

  if (bytes_remaining - bytes_transferred > 0)
    {
      this->do_read(state, bytes_remaining - bytes_transferred);
    }
  else
    {
      int ticket{};
      int length{};
      switch (state)
        {
        case State::PRE_CONTENT:
          length = std::stoi(this->_in_pre_content_buffer.substr(5, 9));
          this->_in_content_buffer.resize(length - 6);
          this->do_read(State::CONTENT);
          break;

        case State::CONTENT:
          this->do_read(State::POST_CONTENT);
          break;

        case State::POST_CONTENT:
          ticket = std::stoi(this->_in_pre_content_buffer.substr(0, 4));

          // Sync access to ticket
          {
            std::lock_guard<std::mutex> data_sync_mutex_lock(
              this->_data_sync_mutex);
            try
              {
                // Get callback id
                long callback_id = this->_ticket_to_callback_id.at(ticket);

                // Erase mapping if it is a one-time ticket triggered by a Call
                // method
                if (ticket >= ifm3d::ONE_TIME_TICKET_LOWER_RANGE &&
                    ticket <= ifm3d::ONE_TIME_TICKET_HIGHER_RANGE)
                  {
                    this->_ticket_to_callback_id.erase(ticket);
                  }

                // Execute callback
                this->_pending_callbacks.at(callback_id)(
                  this->_in_content_buffer);

                // Erase mapping if it is a one-time ticket triggered by a Call
                // method
                if (ticket >= ifm3d::ONE_TIME_TICKET_LOWER_RANGE &&
                    ticket <= ifm3d::ONE_TIME_TICKET_HIGHER_RANGE)
                  {
                    this->_pending_callbacks.erase(callback_id);
                  }
              }
            catch (std::out_of_range& ex)
              {
                LOG_DEBUG("No callback for ticket {} found", ticket);
              }
          }
          this->do_read(State::PRE_CONTENT);
          break;
        }
    }
}

inline std::string&
ifm3d::PCICClient::Impl::in_buffer_by_state(State state)
{
  switch (state)
    {
    case State::PRE_CONTENT:
      return this->_in_pre_content_buffer;
    case State::CONTENT:
      return this->_in_content_buffer;
    case State::POST_CONTENT:
      return this->_in_post_content_buffer;
    }
  throw;
}

inline void
ifm3d::PCICClient::Impl::do_write(State state,
                                  const std::string& out_content_buffer,
                                  std::size_t bytes_remaining)
{
  const std::string& buffer =
    this->out_buffer_by_state(state, out_content_buffer);
  if (bytes_remaining == UNSET)
    {
      bytes_remaining = buffer.size();
    }

  this->_sock.async_write_some(
    asio::buffer(&buffer[buffer.size() - bytes_remaining], bytes_remaining),
    [this, state, out_content_buffer, bytes_remaining](
      auto&& error_code,
      auto&& bytes_transferred) {
      write_handler(
        state,
        std::forward<decltype(error_code)>(error_code),
        std::forward<decltype(bytes_transferred)>(bytes_transferred),
        out_content_buffer,
        bytes_remaining);
    });
}

inline void
ifm3d::PCICClient::Impl::write_handler(State state,
                                       const asio::error_code& ec,
                                       std::size_t bytes_transferred,
                                       const std::string& out_content_buffer,
                                       std::size_t bytes_remaining)
{
  if (ec)
    {
      throw ifm3d::Error(ec.value());
    }

  if (bytes_remaining - bytes_transferred > 0)
    {
      this->do_write(state,
                     out_content_buffer,
                     bytes_remaining - bytes_transferred);
    }
  else
    {
      switch (state)
        {
        case State::PRE_CONTENT:
          this->do_write(State::CONTENT, out_content_buffer);
          break;

        case State::CONTENT:
          this->do_write(State::POST_CONTENT, out_content_buffer);
          break;

        case State::POST_CONTENT:
          std::lock_guard<std::mutex> lock(this->_out_mutex);
          this->_out_completed.store(true);
          this->_out_cv.notify_all();
          break;
        }
    }
}

inline const std::string&
ifm3d::PCICClient::Impl::out_buffer_by_state(
  State state,
  const std::string& out_content_buffer)
{
  switch (state)
    {
    case State::PRE_CONTENT:
      return this->_out_pre_content_buffer;
    case State::CONTENT:
      return out_content_buffer; // NOLINT(bugprone-return-const-ref-from-parameter)
    case State::POST_CONTENT:
      return this->_out_post_content_buffer;
    }
  throw;
}

inline int
ifm3d::PCICClient::Impl::next_command_ticket()
{
  const auto number_of_one_time_tickets =
    ONE_TIME_TICKET_HIGHER_RANGE - ONE_TIME_TICKET_LOWER_RANGE + 1;
  int ticket = ifm3d::ONE_TIME_TICKET_LOWER_RANGE;
  if (!this->_ticket_to_callback_id.empty())
    {
      ticket = (this->_ticket_to_callback_id.rbegin()->first) -
               ifm3d::ONE_TIME_TICKET_LOWER_RANGE;

      // Ignore error/notification message tickets when generating
      // new command tickets
      ticket = std::max(ticket, 0);

      while (this->_ticket_to_callback_id.find(
               ((++ticket) % number_of_one_time_tickets) +
               ifm3d::ONE_TIME_TICKET_LOWER_RANGE) !=
             this->_ticket_to_callback_id.end())
        {}
      ticket = (ticket % number_of_one_time_tickets) +
               ifm3d::ONE_TIME_TICKET_LOWER_RANGE;
    }
  return ticket;
}

inline long
ifm3d::PCICClient::Impl::next_callback_id()
{
  return (this->_current_callback_id == 0) ?
           static_cast<long>(this->_current_callback_id = 1) :
           static_cast<long>(++this->_current_callback_id);
}

#endif // IFM3D_PCICCLIENT_PCICCLIENT_IMPL_H
