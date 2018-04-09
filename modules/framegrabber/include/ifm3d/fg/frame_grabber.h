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

#ifndef __IFM3D_FG_FRAME_GRABBER_H__
#define __IFM3D_FG_FRAME_GRABBER_H__

#include <cstdint>
#include <memory>
#include <vector>
#include <ifm3d/camera/camera.h>
#include <ifm3d/fg/byte_buffer.h>
#include <ifm3d/fg/schema.h>

namespace ifm3d
{
  /**
   * Implements a TCP FrameGrabber connected to the camera passed to its ctor
   */
  class FrameGrabber
  {
  public:
    using Ptr = std::shared_ptr<FrameGrabber>;

    /**
     * Stores a reference to the passed in camera shared pointer and starts a
     * worker thread to stream in pixel data from the device.
     *
     * @param[in] cam The camera instance to grab frames from
     * @param[in] mask A bitmask encoding the image acquisition schema
     *                 to stream in from the camera.
     */
    FrameGrabber(ifm3d::Camera::Ptr cam,
                 std::uint16_t mask = ifm3d::DEFAULT_SCHEMA_MASK);

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
     * This function is used to grab and parse out time synchronized image data
     * from the camera. It will call `SetBytes` on the passed in `ByteBuffer`
     * as well as (optionally, but by default) call `Organize`. Calling
     * `Organize` is the default behavior so the `buff` output parameter is
     * assumed to be synchronized and ready for analysis provided this function
     * returns true. In certain applications, it may be a performance
     * enhancement to not call `Organize` but rather handle that outside of the
     * `FrameGrabber`.
     *
     * @param[out] buff A pointer to an `ifm3d::ByteBuffer<Dervied>` object to
     *                  update with the latest data from the camera.
     *
     * @param[in] timeout_millis Timeout in millis to wait for new image data
     *                           from the FrameGrabber. If `timeout_millis` is
     *                           set to 0, this function will block
     *                           indefinitely.
     * @param[in] copy_buff Flag indicating whether the framegrabber's internal
     *                           buffer should be copied (O(n)) or swapped
     *                           (O(1)) with the raw bytes of the passed in
     *                           `buff`. You should only flag this as `true` if
     *                           you are planning to use multiple clients with
     *                           a single `FrameGrabber` -- even then, think
     *                           carefully before copying data around.
     * @param[in] organize Flag indicating whether or not `Organize` should be
     *                           called on the `ByteBuffer` before returning.
     *
     * @return true if a new buffer was acquired w/in `timeout_millis`, false
     *              otherwise.
     */
    template <typename T>
    bool WaitForFrame(ifm3d::ByteBuffer<T>* buff,
                      long timeout_millis = 0,
                      bool copy_buff = false,
                      bool organize = true)
    {
      bool retval =
        this->WaitForFrame(timeout_millis,
                           [buff, copy_buff]
                           (std::vector<std::uint8_t>& frame_data)
                           {
                             buff->SetBytes(frame_data, copy_buff);
                           });

      // NOTE: it is an optimization to keep the call to Organize() outside of
      //       the lambda.
      if (retval && organize)
        {
          buff->Organize();
        }

      return retval;
    }

  protected:
    /**
     * This is a convenience/wrapper function used to proxy `WaitForFrame`
     * calls through to the pimpl class w/o requiring the pimpl to know about
     * the ByteBuffer CRTP class hierarchy -- i.e., it operates on a vector of
     * bytes not an `ifm3d::ByteBuffer<Derived>`.
     *
     * @param[in] timeout_millis Timeout in millis to wait for new image data
     *                           from the FrameGrabber. If `timeout_millis` is
     *                           set to 0, this function will block
     *                           indefinitely.
     *
     * @param[in] set_bytes A mutator function that will be called with the
     *                      latest frame data bytes from the camera.
     */
    bool
    WaitForFrame(long timeout_millis,
                 std::function<void(std::vector<std::uint8_t>&)> set_bytes);

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

  }; // end: class FrameGrabber

} // end: namespace ifm3d

#endif // __IFM3D_FG_FRAME_GRABBER_H__
