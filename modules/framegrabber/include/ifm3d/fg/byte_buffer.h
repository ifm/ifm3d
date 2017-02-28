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

#ifndef __IFM3D_FG_BYTE_BUFFER_H__
#define __IFM3D_FG_BYTE_BUFFER_H__

#include <cstdint>
#include <memory>
#include <vector>

namespace ifm3d
{
  extern const std::size_t IMG_TICKET_SZ; // bytes

  /**
   * Validates the passed in "ticket" from the sensor. This is a low-level
   * PCIC protocol detail.
   *
   * @param[in] buff The raw ticket bytes from the sensor
   *
   * @return true if the ticket buffer is valid
   */
  bool verify_ticket_buffer(const std::vector<std::uint8_t>& buff);

  /**
   * Verifies that the passed in image buffer is valid.
   *
   * @param[in] buff The raw bytes from the sensor including all
   *                 framing and chunk headers
   *
   * @return true if the image buffer is valid
   */
  bool verify_image_buffer(const std::vector<std::uint8_t>& buff);

  /**
   * Extracts the image buffer size from an image ticket buffer received from
   * the sensor.
   *
   * NOTE: The size of the passed in buffer is not checked. It is assumed
   * that you have already called `verify_ticket_buffer` on the passed
   * in buff.
   *
   * @param[in] buff A verified image ticket buffer
   *
   * @return The expected size of the image buffer
   */
  std::size_t get_image_buffer_size(const std::vector<std::uint8_t>& buff);

  /**
   * The ByteBuffer class is used to hold a validated byte buffer from the
   * sensor that represents a single timesynchronized set of images based on
   * the current schema mask set on the active framegrabber.
   *
   * The ByteBuffer imposes no specific image or point cloud data
   * structure. Further, no specific coordinate frame conventions are
   * enforced. This class is intended to be subclassed where more user-friendly
   * data structures can be used to gain access to the bytes in a semantically
   * meaningful manner.
   *
   * NOTE: The ByteBuffer is NOT thread safe!
   */
  class ByteBuffer
  {
  public:
    using Ptr = std::shared_ptr<ByteBuffer>;

    /**
     * Default initializes instance vars
     */
    ByteBuffer();

    /**
     * RAII dealloc
     */
    virtual ~ByteBuffer();

    // no move semantics
    ByteBuffer(ByteBuffer&&) = delete;
    ByteBuffer& operator=(ByteBuffer&&) = delete;

    // copy ctor/assignment operator
    ByteBuffer(const ByteBuffer& src_buff);
    ByteBuffer& operator=(const ByteBuffer& src_buff);

    /**
     * Returns a copy of the underlying byte buffer read from the camera
     */
    std::vector<std::uint8_t> Bytes();

    /**
     * Returns the state of the `dirty' flag
     */
    bool Dirty() const noexcept;

    /**
     * Intended for subclasses to override. It provides a hook to synchronize
     * the internally wrapped byte buffer with the semantically meaningful
     * image/cloud data structures used by the subclass.
     */
    virtual void Organize();

    /**
     * Sets the data from the passed in `buff' to the internally wrapped byte
     * buffer. This function assumes the passed in `buff' is valid.
     *
     * By default, this function will take in `buff` and `swap` contents with
     * its internal buffer so that the operation is O(1) and requires no data
     * copies. If you want copy behavior, specify the `copy` flag and
     * complexity will be linear in the size of the byte buffer which is driven
     * by the schema mask currently applied to the running framegrabber.
     *
     * @param[in] buff Raw data bytes to copy/swap to internal buffers
     * @param[in] copy If true, the data are copied from `buff` to the
     *                 internally wrapped buffer and `buff` will remain
     *                 unchanged.
     */
    void SetBytes(std::vector<std::uint8_t>& buff, bool copy = false);

  protected:
    /**
     * Flag used to indicate if the wrapped byte buffer needs to be
     * `Organized'. I.e., in a subclass, this would indicate if your parsed out
     * image data structures need to be synchronized to the underlying byte
     * buffer or not.
     */
    bool dirty_;

    /**
     * Raw bytes read off the wire from the camera.
     */
    std::vector<std::uint8_t> bytes_;

    /**
     * Mutates the dirty flag
     */
    void _SetDirty(bool flg) noexcept;

  }; // end: class ByteBuffer

} // end: namespace ifm3d

#endif // __IFM3D_FG_BYTE_BUFFER_H__
