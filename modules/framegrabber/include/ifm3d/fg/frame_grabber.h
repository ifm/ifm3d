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
#include <type_traits>
#include <ifm3d/camera/camera.h>
#include <ifm3d/fg/image.h>
#include <ifm3d/fg/organizer.h>
#include <ifm3d/fg/frame.h>

namespace ifm3d
{
  /**
   * Implements a TCP FrameGrabber connected to the camera passed to its ctor
   */
  class FrameGrabber
  {
  public:
    using Ptr = std::shared_ptr<FrameGrabber>;
    using NewFrameCallback = std::function<void(Frame::Ptr)>;

    /**
     * Stores a reference to the passed in camera shared pointer
     *
     * @param[in] cam The camera instance to grab frames from
     * @param[in] pcic_port TCP port for the pcic connection
     */
    FrameGrabber(ifm3d::CameraBase::Ptr cam,
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
     * Triggers the camera for image acquisition
     *
     * You should be sure to set the `TriggerMode` for your application to
     * `SW` in order for this to be effective. This function
     * simply does the triggering, data are still received asynchronously via
     * `WaitForFrame()`.
     *
     * Calling this function when the camera is not in `SW` trigger mode or on
     * a device that does not support software-trigger should result in a NOOP
     * and no error will be returned (no exceptions thrown). However, we do not
     * recommend calling this function in a tight framegrabbing loop when you
     * know it is not needed. The "cost" of the NOOP is undefined and incurring
     * it is not recommended.
     */
    void SWTrigger();

    /**
     * The callback will be executed whenever a new frame is available.
     * It receives a Frame::Ptr to the received frame as an argument.
     */
    void OnNewFrame(NewFrameCallback callback = nullptr);

    /**
     * Starts the worker thread for streaming in pixel data from the device
     *
     * @param[in] images set of ImageIds for receiving, passing in an empty set
     * will received all available images.
     * The ImageIds are specific to the current Organizer. See image_id for a
     * list of ImageIds available with the default Organizer
     */
    bool Start(const std::set<ImageId>& images = {});

    /**
     * Starts the worker thread for streaming in pixel data from the device
     *
     * @param[in] images set of ImageIds for receiving, passing in an empty set
     * will received all available images.
     * The ImageIds are specific to the current Organizer. See image_id for a
     * list of ImageIds available with the default Organizer
     */
    template <typename T, typename... Args>
    typename std::enable_if_t<std::is_enum_v<T>, bool>
    Start(std::set<ImageId>& images, T id, Args... args)
    {
      images.insert(static_cast<ImageId>(id));
      return Start(images, args...);
    }

    /**
     * Starts the worker thread for streaming in pixel data from the device
     *
     * @param[in] images set of ImageIds for receiving, passing in an empty set
     * will received all available images.
     * The ImageIds are specific to the current Organizer. See image_id for a
     * list of ImageIds available with the default Organizer
     */
    template <typename T, typename... Args>
    typename std::enable_if_t<std::is_enum_v<T>, bool>
    Start(T id, Args... args)
    {
      std::set<ImageId> images;
      images.insert(static_cast<ImageId>(id));
      return Start(images, args...);
    }

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

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

  }; // end: class FrameGrabber

} // end: namespace ifm3d

#endif // IFM3D_FG_FRAMEGRABBER_H
