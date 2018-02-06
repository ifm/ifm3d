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
#ifndef __IFM3D_PCICCLIENT_PCICCLIENT_H__
#define __IFM3D_PCICCLIENT_PCICCLIENT_H__

#include <atomic>
#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>
#include <ifm3d/camera.h>

namespace ifm3d
{
  /**
   * The PCICClient is a class that, when given access to an
   * ifm3d::Camera::Ptr, it provides unbuffered communication with
   * the PCIC interface.
   */
  class PCICClient
  {
  public:
    using Ptr = std::shared_ptr<PCICClient>;

    /**
     * Stores reference to the passed in camera and starts connect/receive
     * thread
     *
     * @param[in] cam The camera instance to grab frames from
     */
    PCICClient(ifm3d::Camera::Ptr cam);

    /**
     * Cleans up any resources held by the receive thread object and
     * blocks until the operating system thread stops.
     */
    virtual ~PCICClient();

    // copy and move semantics
    PCICClient(PCICClient&&) = delete;
    PCICClient& operator=(PCICClient&&) = delete;
    PCICClient(PCICClient&) = delete;
    PCICClient& operator=(const PCICClient&) = delete;

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
     */
    std::string Call(const std::string& request);

    /**
     * Sets the specified callback for receiving asynchronous error messages
     * until it is replaced by a new callback or canceled via @see CancelCallback.
     *
     * Note: Since the PCICClient is unbuffered, the receiving thread will be
     * blocked while the error callback has not returned.
     *
     * @param[in] callback Function, called after receiving an error message
     * from camera (without any header information, like ticket, length, etc.)
     *
     * @return Callback id, which can be used to cancel receiving errors.
     */
    long SetErrorCallback(std::function<void(const std::string& error)> callback);

    /**
     * Sets the specified callback for receiving asynchronous notification
     * messages until it is replaced by a new callback or canceled via
     * @see CancelCallback.
     *
     * Note: Since the PCICClient is unbuffered, the receiving thread will be
     * blocked while the notification callback has not returned.
     *
     * @param[in] callback Function, called after receiving an notification message
     * from camera (without any header information, like ticket, length, etc.)
     *
     * @return Callback id, which can be used to cancel receiving notifications.
     */
    long SetNotificationCallback(std::function<void(const std::string& notification)> callback);

    /**

     * Cancels registered callbacks. Must be called in case references/pointers
     * provided through callbacks get invalid. If callback id isn't present internally
     * anymore, i.e. if callback was replaced, already canceled or
     * automatically removed (in case of the Call method), it is simply ignored.
     *
     * @param[in] callback_id Callback id, returned by methods which take a callback
     * as parameter.
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
    void DoRead(State state, int bytes_remaining = UNSET);

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
		 int bytes_remaining = UNSET);

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
    long current_callback_id_;

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
    std::map<long, std::function<void(const std::string& content)>> pending_callbacks_;

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

  }; // end: class PCICClient

} // end: namespace ifm3d

#endif // __IFM3D_PCICCLIENT_PCICCLIENT_H__
