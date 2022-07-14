/*
 * Copyright 2022-present ifm electronic, gmbh

 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_FG_FRAMEGRABBER_H
#define IFM3D_FG_FRAMEGRABBER_H

#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <vector>
#include <variant>
#include <type_traits>
#include <ifm3d/device/device.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/organizer.h>
#include <ifm3d/fg/frame.h>

namespace ifm3d
{
  /**
   * Implements a TCP FrameGrabber connected to the device passed to its ctor
   */
  class FrameGrabber
  {
  public:
    using Ptr = std::shared_ptr<FrameGrabber>;
    using NewFrameCallback = std::function<void(Frame::Ptr)>;
    using AsynErrorCallback =
      std::function<void(const int, const std::string&)>;
    using BufferList =
      std::vector<std::variant<std::uint64_t, int, ifm3d::buffer_id>>;

    /**
     * Stores a reference to the passed in Device shared pointer
     *
     * @param[in] cam The Device instance to grab frames from
     * @param[in] pcic_port TCP port for the pcic connection
     */
    FrameGrabber(ifm3d::Device::Ptr cam,
                 std::optional<std::uint16_t> pcic_port = std::nullopt);

    /**
     * Cleans up resources held by the framegrabbing thread object and blocks
     * until the operating system thread stops.
     */
    virtual ~FrameGrabber();

    // disable copy/move semantics
    FrameGrabber(FrameGrabber&&) = delete;
    FrameGrabber& operator=(FrameGrabber&&) = delete;
    FrameGrabber(FrameGrabber&) = delete;
    FrameGrabber& operator=(const FrameGrabber&) = delete;

    /**
     * Triggers the device for image acquisition
     *
     * You should be sure to set the `TriggerMode` for your application to
     * `SW` in order for this to be effective. This function
     * simply does the triggering, data are still received asynchronously via
     * `WaitForFrame()`.
     *
     * Calling this function when the device is not in `SW` trigger mode or on
     * a device that does not support software-trigger should result in a NOOP
     * and no error will be returned (no exceptions thrown). However, we do not
     * recommend calling this function in a tight framegrabbing loop when you
     * know it is not needed. The "cost" of the NOOP is undefined and incurring
     * it is not recommended.
     */
    std::shared_future<void> SWTrigger();

    /**
     * The callback will be executed whenever a new frame is available.
     * It receives a Frame::Ptr to the received frame as an argument.
     */
    void OnNewFrame(NewFrameCallback callback = nullptr);

    /**
     * Starts the worker thread for streaming in pixel data from the device
     *
     * @param[in] buffers set of buffer_ids for receiving, passing in an empty
     * set will received all available images. The buffer_ids are specific to
     * the current Organizer. See buffer_id for a list of buffer_ids available
     * with the default Organizer
     *
     * @param[in] schema allows to manually set a PCIC schema for
     * asynchronous results. See ifm3d::make_schema for generation logic of the
     * default schema. Manually setting the schema should rarely be needed and
     * most usecases should be covered by the default generated schema.
     *
     * Note: The FrameGrabber is relying on some specific formatting rules, if
     * they are missing from the schema the FrameGrabber will not be able to
     * extract the image data.
     */
    bool Start(const BufferList& buffers,
               const std::optional<json>& schema = std::nullopt);

    /**
     * Stops the worker thread for streaming in pixel data from the device
     */
    bool Stop();

    /**
     * Returns true if the worker thread is currently running
     */
    bool IsRunning();

    /**
     * Returns a future that will resolve when a new frame is available
     */
    std::shared_future<Frame::Ptr> WaitForFrame();

    /**
     * This allows to override the Organizer which is used for extracting the
     * data from the raw PCIC stream.
     *
     * @param organizer The new organizer to be used
     */
    void SetOrganizer(std::unique_ptr<Organizer> organizer);

    /**
     * This function will enable the async error messages on device.
     * The callback will be executed whenever a async error
     * are avaliable. It receives a  error code and error string
     * to the received async error as an argument.
     */
    void OnAsyncError(AsynErrorCallback callback = nullptr);

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

  }; // end: class FrameGrabber

} // end: namespace ifm3d

#endif // IFM3D_FG_FRAMEGRABBER_H
