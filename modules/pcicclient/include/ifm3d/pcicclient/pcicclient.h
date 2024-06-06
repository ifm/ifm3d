// -*- c++ -*-
/*
 * Copyright (C) 2017 Kuhn & Völkel GmbH
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef IFM3D_PCICCLIENT_PCICCLIENT_H
#define IFM3D_PCICCLIENT_PCICCLIENT_H

#include <ifm3d/pcicclient/module_pcicclient.h>
#include <atomic>
#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <ifm3d/device.h>

namespace ifm3d
{
  /** @ingroup PCICClient
   *
   * The PCICClient is a class that, when given access to an
   * ifm3d::Camera::Ptr, it provides unbuffered communication with
   * the PCIC interface.
   */
  class IFM3D_EXPORT PCICClient
  {
  public:
    using Ptr = std::shared_ptr<PCICClient>;

    /**
     * Stores reference to the passed in camera and starts connect/receive
     * thread
     *
     * @param[in] cam The Device instance to grab frames from
     * @param[in] nat_pcic_port pcic port for NAT configuartion
     */
    PCICClient(ifm3d::LegacyDevice::Ptr cam,
               const std::uint16_t pcic_port = ifm3d::PCIC_PORT);

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
    class IFM3D_NO_EXPORT Impl;
    std::unique_ptr<Impl> pImpl;

  }; // end: class PCICClient

} // end: namespace ifm3d

#endif // IFM3D_PCICCLIENT_PCICCLIENT_H
